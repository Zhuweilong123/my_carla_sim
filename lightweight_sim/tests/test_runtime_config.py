import importlib.util
import math
import re
from pathlib import Path

import pytest
import yaml

from lightweight_sim.engine.runtime_config import DEFAULT_RUNTIME_CONFIG
from lightweight_sim.tests.configuration import CONFIG_DIR, CONFIG_FILES, load_configuration


def test_shared_runtime_defaults_match_fixed_step_planning():
    runtime = DEFAULT_RUNTIME_CONFIG

    assert runtime.physics_dt == pytest.approx(0.05)
    assert runtime.dynamic_max_substep_s == pytest.approx(0.0025)
    assert runtime.local_transition_distance_m == pytest.approx(12.0)
    assert runtime.control_period == pytest.approx(runtime.physics_dt)
    assert runtime.plan_period == pytest.approx(runtime.physics_dt)


def test_configured_speed_policy_is_valid():
    parameters = load_configuration()
    # ROS overrides are authoritative; RuntimeConfig supplies missing values.
    simulator = parameters["simulator_node"]
    for kind in ("default", "straight", "curve", "intersection", "lane_change", "parking"):
        value = float(simulator[kind + "_speed_limit_kmh"])
        assert math.isfinite(value) and value > 0, kind
    ratio = float(simulator["target_speed_ratio"])
    assert math.isfinite(ratio) and 0 < ratio <= 1
    for node, name in (
        ("simulator_node", "max_lateral_accel_mps2"),
        ("controller_node", "speed_profile_lookahead_m"),
        ("safe_stop_node", "planned_path_timeout_s"),
        ("controller_manager", "safety_stop_timeout_s"),
    ):
        value = float(parameters[node][name])
        assert math.isfinite(value) and value > 0, (node, name)


def test_parking_planner_selection_is_configured_for_ros_node():
    config_path = Path(__file__).parents[1] / "config" / "default.yaml"
    yaml_text = "\n".join(line.split("#", 1)[0] for line in config_path.read_text(encoding="utf-8").splitlines())

    assert "/**/parking_controller_node:" in yaml_text
    assert re.search(r"(?m)^\s+planner_type:\s*hybrid_astar\s*$", yaml_text)


def test_layered_configuration_preserves_timing_and_safety_contract():
    parameters = load_configuration()
    assert parameters["simulator_node"]["use_sim_time"] is False
    assert parameters["safe_stop_node"]["use_sim_time"] is False
    for node in ("planner_node", "controller_node", "controller_manager"):
        assert parameters[node]["use_sim_time"] is True
    topic = parameters["reference_line_node"]["reference_topic"]
    assert parameters["planner_node"]["routing_reference_topic"] == topic
    assert parameters["controller_node"]["routing_reference_topic"] == topic
    assert parameters["safe_stop_node"]["reference_topic"] == topic


def test_launch_uses_common_configuration_for_scenario_defaults(tmp_path, monkeypatch):
    pytest.importorskip("launch_ros.actions")
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    config_dir = tmp_path / "config"
    config_dir.mkdir()
    for name in CONFIG_FILES:
        (config_dir / name).write_text((CONFIG_DIR / name).read_text(encoding="utf-8"), encoding="utf-8")
    document = yaml.safe_load((config_dir / "default.yaml").read_text(encoding="utf-8"))
    document["/**/simulator_node"]["ros__parameters"].update(
        scenario="figure_eight", steering_profile="assumed",
    )
    (config_dir / "default.yaml").write_text(yaml.safe_dump(document), encoding="utf-8")
    spec = importlib.util.spec_from_file_location(
        "layered_config_launch", CONFIG_DIR / "launch" / "lightweight_sim.launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(tmp_path))
    arguments = {
        action.name: perform_substitutions(LaunchContext(), action.default_value)
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments["scenario"] == "figure_eight"
    assert arguments["steering_profile"] == "assumed"


@pytest.mark.parametrize('namespace', ['/', '/config_audit', '/nested/config_audit'])
def test_ros_loads_all_layered_parameters_under_namespace(namespace):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.node import Node

    args = ['--ros-args', '-r', '__ns:='+namespace]
    for name in CONFIG_FILES:
        args += ['--params-file', str(CONFIG_DIR / name)]
    rclpy.init(args=args)
    try:
        for name, expected in load_configuration().items():
            node = Node(name, automatically_declare_parameters_from_overrides=True)
            try:
                for parameter, value in expected.items():
                    assert node.get_parameter(parameter).value == value, (namespace, name, parameter)
            finally:
                node.destroy_node()
        node = Node('unconfigured_node', automatically_declare_parameters_from_overrides=True)
        try:
            assert not node.has_parameter('vehicle_a_m')
            assert not node.has_parameter('lateral_q_weights')
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('namespace', ['/', '/config_audit'])
def test_control_period_follows_runtime_context_instead_of_fallback(tmp_path, namespace):
    rclpy = pytest.importorskip('rclpy')
    from dataclasses import asdict
    import json
    from std_msgs.msg import String
    from lightweight_sim_msgs.msg import ReferenceLine, PathPoint, RouteSegment
    from lightweight_sim.engine.ros_nodes.simulator_node import SimulatorNode
    from lightweight_sim.engine.ros_nodes.controller_node import ControllerNode

    # Change the actual period while retaining the compatibility fallback.
    document = yaml.safe_load((CONFIG_DIR / 'default.yaml').read_text(encoding='utf-8'))
    document['/**/simulator_node']['ros__parameters'].update(
        physics_dt=.025, steering_profile='assumed')
    common = tmp_path / 'default.yaml'
    common.write_text(yaml.safe_dump(document), encoding='utf-8')
    args = ['--ros-args', '-r', '__ns:='+namespace]
    for name in CONFIG_FILES:
        args += ['--params-file', str(common if name == 'default.yaml' else CONFIG_DIR / name)]
    rclpy.init(args=args)
    sim = control = None
    try:
        sim, control = SimulatorNode(), ControllerNode()
        assert sim.engine.physics_dt == .025
        expected = document['/**/simulator_node']['ros__parameters']
        assert sim.engine.config.target_speed_ratio == expected['target_speed_ratio']
        for kind, limit in sim.engine.config.speed_limits.items():
            assert limit == expected[kind + '_speed_limit_kmh']
        assert sim.engine.config.target_speed == pytest.approx(
            sim.engine.config.speed_limit_kmh * expected['target_speed_ratio'])
        assert control.controller.lat.ts == .05  # Initial compatibility value.
        for run in (sim.run_id, sim.run_id+1):
            config = sim.engine.config
            context = dict(schema_version=1, run_id=run, physics_dt=sim.engine.physics_dt,
                           target_speed_kmh=config.target_speed,
                           dynamic_max_substep_s=config.dynamic_max_substep_s,
                           vehicle_parameters=asdict(config.vehicle_params),
                           steering_parameters=asdict(config.steering))
            control._on_context(String(data=json.dumps(context)))
            control._on_routing_reference(ReferenceLine(
                request_id=run, success=True,
                segments=[RouteSegment(length_m=80., speed_limit_kmh=40., maneuver='straight')],
                points=[PathPoint(x=0., y=0., theta=0., kappa=0.),
                        PathPoint(x=80., y=0., theta=0., kappa=0.)]))
            assert control.active_run == run
            assert control.controller.lat.ts == control.controller.lon.dt == sim.engine.physics_dt
            assert control.controller.lat.max_substep_s == sim.engine.dynamic_max_substep_s
            assert len(control.controller.lat.command_history) == 2
            sim.engine.reset()
    finally:
        for node in (control, sim):
            if node is not None:
                node.destroy_node()
        rclpy.shutdown()
