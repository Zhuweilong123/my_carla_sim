"""Real DDS lifecycle acceptance; run with the ROS overlay sourced.

Physics is explicitly stepped for a bounded tracking interval. The actual
simulator, router, reference-line, planner and controller callbacks use DDS.
"""
import math
import json
import os
import signal
import subprocess
import time
import gzip
import hashlib
from dataclasses import asdict
from pathlib import Path as FilePath
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("lightweight_sim_msgs.msg")
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from lightweight_sim_msgs.msg import (
    Path,
    PathPoint,
    ReferenceLine,
    VehicleState as RosState,
    ControlCommand as RosCommand,
)
from lightweight_sim_msgs.msg import SimulationStatus
from lightweight_sim_msgs.srv import EditScene
from std_msgs.msg import String
from std_srvs.srv import SetBool
from lightweight_sim.engine.ros_nodes.simulator_node import SimulatorNode
from lightweight_sim.engine.ros_nodes.planner_node import PlannerNode
from lightweight_sim.engine.ros_nodes.controller_node import ControllerNode
from lightweight_sim.engine.routing.routing_node import RoutingNode
from lightweight_sim.engine.reference_line.reference_line_node import ReferenceLineNode
from lightweight_sim.engine.ros_nodes.route_session import encode_sequence
from lightweight_sim.engine.ros_nodes.qos import status_qos, sensor_data_qos, latched_path_qos, command_qos
from lightweight_sim.engine.ros_nodes.message_conversions import message_to_state
from lightweight_sim.engine.simulator.data_types import ControlCommand
from lightweight_sim.engine.analysis.evaluation import provenance, archive_sources
from lightweight_sim.engine.analysis.tracking import TrackingMonitor
from parking_module.ros_node import ParkingControllerNode
from parking_module.planning import HybridAStarPlanner, ReverseParkingPlanner


def test_switching_to_reverse_parking_keeps_reverse_capable_engine():
    rclpy.init(args=[])
    node = SimulatorNode()
    try:
        result = node.set_parameters([Parameter("scenario", value="reverse_parking")])
        assert result[0].successful

        state = node.engine.step(
            ControlCommand(gear=-1, throttle=1.0), dt=node.physics_dt
        )

        assert state.vx < 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_edited_scene_keeps_publishing_state_while_paused():
    rclpy.init(args=[])
    sim = SimulatorNode()
    observer = rclpy.create_node("paused_scene_observer")
    executor = SingleThreadedExecutor()
    received = []
    try:
        sim._on_pause(SetBool.Request(data=True), SetBool.Response())
        request = EditScene.Request()
        request.ego_x = 12.0
        request.ego_y = 0.5
        request.ego_yaw = 0.25
        response = sim._on_edit_scene(request, EditScene.Response())
        assert response.success, response.message

        # Subscribe after the one-shot edit response.  Volatile state must
        # still arrive while simulation time and physics steps are frozen.
        observer.create_subscription(RosState, "vehicle/state", received.append, sensor_data_qos())
        executor.add_node(sim)
        executor.add_node(observer)
        deadline = time.monotonic() + 3.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received, "paused scene did not republish vehicle/state"
        assert received[-1].x == pytest.approx(12.0)
        assert received[-1].y == pytest.approx(0.5)
        assert received[-1].yaw == pytest.approx(0.25)
        assert sim.engine.step_count == 0
        assert sim.paused
    finally:
        executor.shutdown()
        observer.destroy_node()
        sim.destroy_node()
        rclpy.shutdown()


def test_edit_scene_rejects_offroad_ego_pose():
    rclpy.init(args=[])
    sim = SimulatorNode()
    try:
        sim._on_pause(SetBool.Request(data=True), SetBool.Response())
        request = EditScene.Request()
        request.ego_x = 12.0
        request.ego_y = -20.0
        request.ego_yaw = 0.0
        response = sim._on_edit_scene(request, EditScene.Response())
        assert not response.success
        assert "outside the drivable road" in response.message
        assert sim.engine.get_state().x == pytest.approx(20.0)
    finally:
        sim.destroy_node()
        rclpy.shutdown()


def test_planner_replays_reference_received_before_scene_context():
    rclpy.init(args=[])
    node = PlannerNode()
    try:
        node.plan_timer.cancel()
        node.poll_timer.cancel()
        reference = ReferenceLine()
        reference.request_id = 123
        reference.success = True
        reference.reference_lane_index = 0
        reference.target_lane = 0
        reference.points = [
            PathPoint(x=0.0, y=0.0, theta=0.0, kappa=0.0),
            PathPoint(x=20.0, y=0.0, theta=0.0, kappa=0.0),
        ]

        # DDS can deliver the reference generated from the edit before the
        # matching context callback.  It must survive that ordering.
        node._on_routing_reference(reference)
        assert node._pending_routing_reference is reference
        node._on_context(String(data=json.dumps({
            "schema_version": 1,
            "run_id": 123,
            "lane_width": 3.5,
            "num_lanes": 2,
        })))
        assert node.routing_reference_run == 123
        assert node.active_run == 123
        assert node.planner is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_parking_node_uses_scenario_target_speed():
    rclpy.init(args=[])
    node = ParkingControllerNode()
    try:
        node._on_context(String(data=json.dumps({
            "run_id": 1,
            "maneuver": "reverse_parking",
            "parking_goal": [46.0, 7.5, -math.pi / 2.0],
            "target_speed_kmh": 10.2,
        })))
        assert node._planner.config.approach_speed == pytest.approx(10.2 / 3.6)
        assert isinstance(node._planner, HybridAStarPlanner)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_physical_parameters_propagate_to_planning_control_and_parking_across_runs():
    from lightweight_sim.engine.simulator.data_types import VehicleParams
    from lightweight_sim_msgs.msg import RouteSegment
    rclpy.init(args=[])
    planner, control, parking = PlannerNode(), ControllerNode(), ParkingControllerNode()
    try:
        for run, params in [(901, VehicleParams(a=1.2, b=2., width=2.3,
                                              body_overhang=1.2, max_steer=.3)),
                            (902, VehicleParams())]:
            context = dict(schema_version=1, run_id=run, lane_width=3.5, num_lanes=3,
                           target_speed_kmh=10.2, physics_dt=.1, dynamic_max_substep_s=.005,
                           vehicle_parameters=asdict(params), maneuver='reverse_parking',
                           parking_goal=[46., 7.5, -math.pi/2])
            reference = ReferenceLine(request_id=run, success=True, reference_lane_index=1,
                                      target_lane=1,
                                      segments=[RouteSegment(length_m=80., speed_limit_kmh=40., maneuver='straight')],
                                      points=[
                                          PathPoint(x=0., y=0., theta=0., kappa=0.),
                                          PathPoint(x=80., y=0., theta=0., kappa=0.)])
            for node in (planner, control, parking):
                node._on_context(String(data=json.dumps(context)))
            planner._on_routing_reference(reference)
            control._on_routing_reference(reference)
            assert planner.planner.vehicle_length_m == pytest.approx(params.length)
            assert planner.planner.vehicle_width_m == params.width
            assert control.controller.params == params
            assert control.controller.lat.ts == control.controller.lon.dt == .1
            assert control.controller.lat.max_substep_s == .005
            assert control.controller.lat.max_steer == params.max_steer
            for config in (parking._planner.config, parking._controller.config):
                assert config.vehicle_length == pytest.approx(params.length)
                assert config.vehicle_width == params.width
                assert config.wheelbase == pytest.approx(params.wheelbase)
                assert config.max_steer == params.max_steer
    finally:
        planner._stop_planner()
        for node in (planner, control, parking):
            node.destroy_node()
        rclpy.shutdown()


def test_simulator_rejects_period_change_without_rebuilding_timer_and_actuator():
    rclpy.init(args=[])
    node = SimulatorNode()
    try:
        old_dt = node.physics_dt
        result = node.set_parameters([Parameter('physics_dt', value=old_dt*2)])
        assert not result[0].successful
        assert 'restart' in result[0].reason
        assert node.get_parameter('physics_dt').value == node.engine.physics_dt == old_dt
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("planner_type", "planner_class"),
    [("baseline", ReverseParkingPlanner), ("hybrid_astar", HybridAStarPlanner)],
)
def test_parking_node_selects_planner_from_ros_parameter(planner_type, planner_class):
    rclpy.init(args=["--ros-args", "-p", f"planner_type:={planner_type}"])
    node = ParkingControllerNode()
    try:
        assert isinstance(node._planner, planner_class)
        assert node._planner_type == planner_type
    finally:
        node.destroy_node()
        rclpy.shutdown()


def archive_acceptance(name, payload, passed):
    directory = os.environ.get("LIGHTWEIGHT_SIM_ROS_ARCHIVE")
    if not directory:
        return
    directory = FilePath(directory)
    directory.mkdir(parents=True, exist_ok=True)
    meta = provenance()
    meta["source_sha256"]["tests/test_ros_lifecycle.py"] = hashlib.sha256(FilePath(__file__).read_bytes()).hexdigest()
    meta["source_tree_sha256"] = hashlib.sha256(json.dumps(meta["source_sha256"], sort_keys=True).encode()).hexdigest()
    archive_sources(meta, directory)
    with gzip.open(directory/(name+".json.gz"), "xt", encoding="utf-8") as stream:
        json.dump(payload, stream)
    with (directory/(name+".json")).open("x", encoding="utf-8") as stream:
        json.dump(dict(passed=passed, provenance=meta, payload=name+".json.gz",
                       counts={k: len(v) for k, v in payload.items() if isinstance(v, list)}), stream, indent=2)


def test_ros_tracking_reset_switch_and_stale_plan():
    rclpy.init(args=["--ros-args", "-p", "scenario:=figure_eight", "-p", "steering_profile:=ideal",
                     "-p", "dynamic_max_substep_s:=0.005"])
    nodes = []
    payload = dict(states=[], measured=[], events=[])
    passed = False
    executor = SingleThreadedExecutor()
    try:
        # The production graph routes each run through the independent router
        # and reference-line adapter.  Keep those nodes in this DDS acceptance
        # test as well; otherwise planner/controller can never activate.
        routing = RoutingNode()
        reference_line = ReferenceLineNode()
        sim, planner, control = SimulatorNode(), PlannerNode(), ControllerNode()
        nodes = [routing, reference_line, sim, planner, control]
        # This deterministic unit graph omits ControllerManager/SafeStopNode;
        # wire the controller candidate directly to the simulator actuator.
        control.command_pub = sim.create_publisher(
            RosCommand, "control_command", command_qos()
        )
        for node in (planner, control):
            node.set_parameters([Parameter("use_sim_time", value=True)])
        sim.timer.cancel()
        planner.plan_timer.cancel()
        planner.poll_timer.cancel()
        control.timer.cancel()
        measured = []
        sim.create_subscription(String, "tracking/metrics",
                                lambda msg: measured.append(json.loads(msg.data)), sensor_data_qos())
        for node in nodes:
            executor.add_node(node)

        def spin_until(predicate, timeout=4.0):
            deadline = time.monotonic()+timeout
            while not predicate():
                assert time.monotonic() < deadline, "DDS callback timeout"
                executor.spin_once(timeout_sec=0.002)
                # _advance_once is deliberately stepped without publishing
                # /clock, so ROS timers are frozen; poll the planner's
                # worker future explicitly as part of this deterministic
                # test driver.
                if planner.plan_pending:
                    planner._poll_result()

        spin_until(lambda: control.active_run == sim.run_id and planner.active_run == sim.run_id)
        assert control.controller.lon.target_speed == pytest.approx(sim.engine.config.target_speed)
        assert planner.planner.num_lanes == 3
        assert control.route_context["steering_parameters"]["mode"] == sim.engine.config.steering.mode
        assert control.controller.lat.actuator_params == sim.engine.config.steering
        assert control.controller.lat.max_substep_s == sim.engine.dynamic_max_substep_s
        assert control.controller.lat.max_substep_s == pytest.approx(0.005)
        # Prime the planner from the initial pose before stepping physics. This
        # avoids intentionally dropping the first dynamic-actuator history tick
        # while the asynchronous local planner is still producing its first path.
        sim._publish_state()
        spin_until(lambda: control.plan_ready)
        global_monitor = TrackingMonitor(control.reference_path)
        initial_run = sim.run_id
        payload.update(context=control.route_context.copy(), reference=sim.engine.world.ref_path_as_tuples,
                       plant_parameters=asdict(sim.engine.ego.params),
                       controller=dict(Q=control.controller.lat.Q.tolist(), R=control.controller.lat.R.tolist(),
                                       feedback_horizon_s=control.controller.lat.feedback_horizon_s,
                                       discretization=control.controller.lat.discretization))
        previous_s = 0.0
        previous_xy = (sim.engine.get_state().x, sim.engine.get_state().y)
        travelled_m = 0.0
        for step in range(300):
            previous_command_time = sim.last_command_time
            applied = ControlCommand(brake=1.) if sim._command_is_stale() else sim.command
            sim._advance_once()
            stamp = sim.engine.sim_time
            spin_until(
                lambda: control.state is not None
                and abs(control.state.timestamp-stamp) < 1e-6
            )
            spin_until(
                lambda: abs(control.get_clock().now().nanoseconds / 1e9-stamp) < 1e-6
            )
            # Wait for delivery, not an arbitrary count of executor callbacks:
            # /clock adds callbacks and a fixed count can starve commands.
            spin_until(lambda: sim.last_command_time != previous_command_time)
            expected = control.controller.lat.last_ed
            if step % 10 == 0:
                planner._request_plan()
            tracker = global_monitor.tracker
            state = sim.engine.get_state()
            progress = global_monitor.update(state)["route_s_m"]
            travelled_m += math.hypot(state.x-previous_xy[0], state.y-previous_xy[1])
            previous_xy = (state.x, state.y)
            payload["states"].append(dict(state=asdict(state),
                applied=asdict(applied), next_command=asdict(sim.command), route_s_m=progress,
                control_ed_m=control.controller.lat.last_ed, control_ephi_rad=control.controller.lat.last_ephi,
                actuator_delayed_rad=sim.engine.steering.delayed,
                actuator_peak_rate_rad_s=sim.engine.steering.peak_rate_rad_s,
                actuator_rate_limited=sim.engine.steering.rate_limited,
                collision=sim.engine.collision_occurred, offroad=sim.engine.offroad_occurred))
            assert progress-previous_s < 3.0, "branch jump"
            assert progress-previous_s > -2.0, "unexpected reverse progress"
            previous_s = progress
            assert math.isfinite(expected)
            assert not sim.engine.is_done
        assert not sim.engine.is_done
        print(f"DDS acceptance: steps={step+1} sim_s={sim.engine.sim_time:.2f} "
              f"route_s_m={progress:.3f} collision=False offroad=False")
        assert measured[-1]["protocol"] == "route_projection_v2"
        assert travelled_m > 5.0

        old_plan = Path(sequence=encode_sequence(initial_run, 999))
        sim._on_reset(None, object())
        spin_until(lambda: control.active_run == sim.run_id and planner.active_run == sim.run_id)
        assert sim.run_id != initial_run
        assert control.controller.lat.max_substep_s == pytest.approx(0.005)
        assert control.controller.lat.route_s < 2.0
        accepted_sequence = control.last_sequence
        control._on_planned(old_plan)
        assert control.last_sequence == accepted_sequence
        assert not control.planned_path
        assert sim.engine.steering.queue is None
        assert all(v == 0 for v in control.controller.lat.command_history)
        payload["events"].append(dict(event="reset_and_old_plan_rejection", context=control.route_context.copy()))

        result = sim.set_parameters([Parameter("scenario", value="default")])
        assert result[0].successful
        spin_until(lambda: control.active_run == sim.run_id and planner.active_run == sim.run_id)
        assert control.controller.lon.target_speed == sim.engine.config.target_speed
        assert planner.planner.num_lanes == sim.engine.config.road.num_lanes
        assert sim.engine.dynamic_max_substep_s == pytest.approx(0.005)
        assert control.controller.lat.max_substep_s == pytest.approx(0.005)
        assert control.controller.lat.ts == sim.physics_dt
        assert not control.planned_path
        payload["events"].append(dict(event="switch_to_default", context=control.route_context.copy()))
        payload["measured"] = measured
        passed = True
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        archive_acceptance("dds_lifecycle_ideal", payload, passed)


@pytest.mark.parametrize("steering_profile", ["ideal", "assumed"])
def test_installed_launch_routes_and_diagnostics(tmp_path, steering_profile):
    """Exercise separately launched processes and real /clock timers."""
    rclpy.init()
    observer = rclpy.create_node("p1_acceptance_observer", namespace="p1_acceptance")
    contexts, metrics, statuses = [], [], []
    states, commands = [], []
    passed = False
    observer.create_subscription(String, "sim/context", lambda m: contexts.append(json.loads(m.data)), latched_path_qos())
    observer.create_subscription(String, "tracking/metrics", lambda m: metrics.append(json.loads(m.data)), sensor_data_qos())
    observer.create_subscription(SimulationStatus, "sim/status", statuses.append, status_qos())
    observer.create_subscription(RosState, "vehicle/state", lambda m: states.append(asdict(message_to_state(m))), sensor_data_qos())
    observer.create_subscription(RosCommand, "control_command", lambda m: commands.append(dict(
        time_s=m.header.stamp.sec+m.header.stamp.nanosec/1e9, steer=m.steering_angle,
        throttle=m.throttle, brake=m.brake)), command_qos())
    process = None
    try:
        with (tmp_path/"launch.log").open("w") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "lightweight_sim", "lightweight_sim.launch.py",
                 "scenario:=figure_eight", "gui:=false", "namespace:=p1_acceptance",
                 "steering_profile:="+steering_profile],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+70
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.05)
                assert process.poll() is None, (tmp_path/"launch.log").read_text()
                distance_travelled = sum(
                    math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
                    for previous, current in zip(states, states[1:])
                )
                if metrics and metrics[-1]["timestamp"] >= 30 and distance_travelled > 20:
                    break
            assert contexts and metrics and statuses, (tmp_path/"launch.log").read_text()
            # The figure-eight is a curve-speed scenario: its 40 km/h limit
            # with the configured 0.85 target ratio gives 34 km/h.
            assert contexts[-1]["target_speed_kmh"] == pytest.approx(34.0)
            assert contexts[-1]["num_lanes"] == 3
            assert contexts[-1]["vehicle_model"] == "dynamic"
            assert metrics[-1]["timestamp"] >= 30
            assert contexts[-1]["steering_parameters"]["mode"] == ("dynamic" if steering_profile == "assumed" else "ideal")
            assert sum(
                math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
                for previous, current in zip(states, states[1:])
            ) > 20
            assert not any(s.collision or s.offroad for s in statuses)
            assert states and commands
            if steering_profile == "assumed":
                # A timestamp/distance check alone can pass despite repeated
                # stops. The unobstructed curve must keep moving after startup.
                # ST planning starts with a bounded-jerk ramp after the initial
                # readiness brake; allow that ramp to finish before checking.
                assert min(math.hypot(s["vx"], s["vy"]) for s in states
                           if s["timestamp"] > 4.0) > 0.5
                for previous, current in zip(states, states[1:]):
                    elapsed = current["timestamp"]-previous["timestamp"]
                    if elapsed > 0:
                        assert abs(current["steer"]-previous["steer"])/elapsed <= 0.6+1e-6
            passed = True
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        observer.destroy_node()
        rclpy.shutdown()
        archive_acceptance("installed_launch_"+steering_profile, dict(contexts=contexts, measured=metrics,
            states=states, commands=commands,
            statuses=[dict(time_s=s.sim_time, running=s.running, collision=s.collision,
                           offroad=s.offroad, scenario=s.scenario) for s in statuses],
            launch_log=(tmp_path/"launch.log").read_text()), passed)


def test_dynamic_controller_latches_timing_gap_until_reset():
    from lightweight_sim.engine.algorithms.controller.combined import VehicleController
    from lightweight_sim.engine.simulator.steering import steering_profile
    from lightweight_sim.engine.simulator.data_types import VehicleState
    rclpy.init()
    node = ControllerNode()
    try:
        node.timer.cancel()
        path = [(0., 0., 0., 0.), (1000., 0., 0., 0.)]
        node.controller = VehicleController(steering_params=steering_profile("assumed"))
        node.controller.update_ref_path(path)
        node.reference_path = path
        node.active_run = 1
        # _on_timer intentionally holds the vehicle until a matching local
        # plan is ready.  This test isolates timing-gap behavior after that
        # normal activation gate.
        node.plan_ready = True
        node.last_control_stamp = 1.0
        node.state = VehicleState(x=20, vx=10, steer=0.02, timestamp=1.10)
        node.state_time = node.get_clock().now()
        sent = []
        node._publish_command = lambda *command: sent.append(command)
        node._on_timer()
        assert node.actuator_timing_fault
        assert sent[-1] == (0.02, 0.0, 1.0)
        node.state.timestamp = 1.15
        node._on_timer()
        assert node.actuator_timing_fault
        assert sent[-1][2] == 1.0
        node._on_context(String(data=json.dumps(dict(schema_version=1, run_id=2))))
        assert not node.actuator_timing_fault
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dynamic_controller_synchronizes_during_plan_stop_and_recovers_with_feedback():
    from lightweight_sim.engine.algorithms.controller.combined import VehicleController
    from lightweight_sim.engine.simulator.steering import steering_profile
    from lightweight_sim.engine.simulator.data_types import VehicleState
    rclpy.init()
    node = ControllerNode()
    try:
        node.timer.cancel()
        path = [(0., 0., 0., 0.), (1000., 0., 0., 0.)]
        node.controller = VehicleController(steering_params=steering_profile('assumed'))
        node.controller.update_ref_path(path)
        node.reference_path = path
        node.measurement_path = path
        node.tracking_monitor = TrackingMonitor(path)
        node.active_run = 1
        node.plan_ready = False
        sent = []
        node._publish_command = lambda *command: sent.append(command)
        for stamp, queue in [(1., [.2]), (1.05, [0.]), (1.1, [0.])]:
            node.state = VehicleState(x=20., vx=5., timestamp=stamp, steering_delay_queue=queue)
            node.state_time = node.get_clock().now()
            node._on_timer()
            assert node.last_control_stamp == stamp
            assert node.controller.lat.command_history == queue
            assert not node.actuator_timing_fault and sent[-1][2] == 1.
        # Missing intermediate state messages is recoverable with a complete,
        # timestamped plant FIFO; a candidate FIFO alone cannot recover it.
        node.plan_ready = True
        node.state = VehicleState(x=20., vx=5., timestamp=1.3, steering_delay_queue=[-.1])
        node.state_time = node.get_clock().now()
        observed = []
        def step(*args, **kwargs):
            observed.append(list(node.controller.lat.command_history))
            return .02, .2, 0.
        node.controller.step = step
        node._on_timer()
        assert observed == [[-.1]]
        assert not node.actuator_timing_fault and sent[-1] == (.02, .2, 0.)
        node._on_timer()
        assert len(observed) == 1  # Same state does not advance history twice.
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('queue', [[], [float('nan')], [.6]])
def test_dynamic_controller_latches_invalid_plant_queue(queue):
    from lightweight_sim.engine.algorithms.controller.combined import VehicleController
    from lightweight_sim.engine.simulator.steering import steering_profile
    from lightweight_sim.engine.simulator.data_types import VehicleState
    rclpy.init()
    node = ControllerNode()
    try:
        node.timer.cancel()
        node.controller = VehicleController(steering_params=steering_profile('assumed'))
        node.active_run = 1
        node.state = VehicleState(timestamp=1., steering_delay_queue=queue)
        node.state_time = node.get_clock().now()
        sent = []
        node._publish_command = lambda *command: sent.append(command)
        node._on_timer()
        assert node.actuator_timing_fault and sent[-1][2] == 1.
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dynamic_controller_resumes_after_empty_path_without_run_reset():
    from lightweight_sim.engine.algorithms.controller.combined import VehicleController
    from lightweight_sim.engine.simulator.steering import steering_profile, SteeringActuator
    rclpy.init()
    node = ControllerNode()
    try:
        node.timer.cancel()
        route = [(0., 0., 0., 0.), (1000., 0., 0., 0.)]
        params = steering_profile('assumed')
        actuator = SteeringActuator(params, .5)
        node.controller = VehicleController(steering_params=params)
        node.controller.update_ref_path(route)
        node.active_run = 1
        node.reference_path = route
        node.measurement_path = route
        node.tracking_monitor = TrackingMonitor(route)
        sent = []
        node._publish_command = lambda *command: sent.append(command)
        def plan(version, points):
            message = Path()
            message.header.stamp = node.get_clock().now().to_msg()
            message.sequence = encode_sequence(1, version)
            message.points = [PathPoint(x=p[0], y=p[1], theta=p[2], kappa=p[3]) for p in points]
            node._on_planned(message)
        def state(tick, actual_command):
            actuator.begin_period(actual_command, .05)
            actuator.advance(.05)
            message = RosState(x=20., vx=5., steering_angle=actuator.angle,
                               steering_history_valid=True,
                               steering_delay_queue=actuator.history_snapshot(.05))
            message.header.stamp.sec = 1
            message.header.stamp.nanosec = tick*50_000_000
            node._on_state(message)
        plan(1, route)
        state(0, .2)
        assert sent[-1][1] > 0.
        plan(2, [])
        for tick in range(1, 5):
            state(tick, 0.)  # Arbiter actually executes the safety brake.
            assert sent[-1] == (0., 0., 1.)
            assert node.controller.lat.command_history == [0.]
            assert not node.actuator_timing_fault
        plan(3, route)
        state(5, 0.)
        assert node.active_run == 1 and not node.actuator_timing_fault
        assert node.plan_ready and sent[-1][1] > 0. and sent[-1][2] == 0.
    finally:
        node.destroy_node()
        rclpy.shutdown()
