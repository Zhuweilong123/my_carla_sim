import importlib.util
import re
from pathlib import Path

import pytest
import yaml

from lightweight_sim.engine.runtime_config import DEFAULT_RUNTIME_CONFIG


CONFIG_DIR = Path(__file__).parents[1] / "config"
CONFIG_FILES = (
    "system.yaml", "vehicle.yaml", "algorithms.yaml", "baseline.yaml",
    "compatibility.yaml", "default.yaml",
)


def load_configuration():
    merged = {}
    for name in CONFIG_FILES:
        document = yaml.safe_load((CONFIG_DIR / name).read_text(encoding="utf-8"))
        for selector, section in document.items():
            assert selector.startswith('/**/'), f'Namespace-dependent selector: {selector}'
            node = selector.removeprefix('/**/')
            target = merged.setdefault(node, {})
            parameters = section["ros__parameters"]
            assert not target.keys() & parameters.keys(), f"Duplicate parameters for {node} in {name}"
            target.update(parameters)
    return merged


def test_shared_runtime_defaults_match_fixed_step_planning():
    runtime = DEFAULT_RUNTIME_CONFIG

    assert runtime.physics_dt == pytest.approx(0.05)
    assert runtime.dynamic_max_substep_s == pytest.approx(0.0025)
    assert runtime.local_transition_distance_m == pytest.approx(12.0)
    assert runtime.control_period == pytest.approx(runtime.physics_dt)
    assert runtime.plan_period == pytest.approx(runtime.physics_dt)


def test_shared_runtime_speed_defaults_match_ros_configuration():
    runtime = DEFAULT_RUNTIME_CONFIG
    parameters = load_configuration()

    shared_parameters = {
        "default_speed_limit_kmh": runtime.default_speed_limit_kmh,
        "straight_speed_limit_kmh": runtime.straight_speed_limit_kmh,
        "curve_speed_limit_kmh": runtime.curve_speed_limit_kmh,
        "intersection_speed_limit_kmh": runtime.intersection_speed_limit_kmh,
        "lane_change_speed_limit_kmh": runtime.lane_change_speed_limit_kmh,
        "parking_speed_limit_kmh": runtime.parking_speed_limit_kmh,
        "target_speed_ratio": runtime.target_speed_ratio,
        "speed_profile_lookahead_m": runtime.speed_profile_lookahead_m,
        "max_lateral_accel_mps2": runtime.max_lateral_accel_mps2,
        "planned_path_timeout_s": runtime.safe_stop_plan_timeout_s,
        "safety_stop_timeout_s": runtime.controller_safety_heartbeat_timeout_s,
    }
    for name, expected in shared_parameters.items():
        matches = [values[name] for values in parameters.values() if name in values]
        assert matches, f"{name} is missing from layered configuration"
        assert float(matches[0]) == pytest.approx(expected), name


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
