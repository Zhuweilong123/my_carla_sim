"""Real DDS lifecycle acceptance; run with the ROS overlay sourced.

Physics is explicitly stepped (no wall-time waiting for ten simulated laps).
The actual simulator, planner and controller callbacks communicate over DDS.
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
from lightweight_sim.engine.ros_nodes.route_session import encode_sequence
from lightweight_sim.engine.ros_nodes.qos import status_qos, sensor_data_qos, latched_path_qos, command_qos
from lightweight_sim.engine.ros_nodes.message_conversions import message_to_state
from lightweight_sim.engine.simulator.data_types import ControlCommand
from lightweight_sim.engine.analysis.evaluation import provenance, archive_sources
from parking_module.ros_node import ParkingControllerNode


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
        request.ego_y = 3.0
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
        assert received[-1].y == pytest.approx(3.0)
        assert received[-1].yaw == pytest.approx(0.25)
        assert sim.engine.step_count == 0
        assert sim.paused
    finally:
        executor.shutdown()
        observer.destroy_node()
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


@pytest.mark.parametrize("steering_profile", ["ideal", "assumed"])
def test_ros_ten_laps_reset_switch_and_stale_plan(steering_profile):
    rclpy.init(args=["--ros-args", "-p", "scenario:=figure_eight", "-p", "steering_profile:="+steering_profile])
    nodes = []
    payload = dict(states=[], measured=[], events=[])
    passed = False
    executor = SingleThreadedExecutor()
    try:
        sim, planner, control = SimulatorNode(), PlannerNode(), ControllerNode()
        nodes = [sim, planner, control]
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

        spin_until(lambda: control.active_run == sim.run_id and planner.active_run == sim.run_id)
        assert control.controller.lon.target_speed == 50.0
        assert planner.planner.num_lanes == 3
        assert control.route_context["steering_parameters"]["mode"] == sim.engine.config.steering.mode
        assert control.controller.lat.actuator_params == sim.engine.config.steering
        initial_run = sim.run_id
        payload.update(context=control.route_context.copy(), reference=sim.engine.world.ref_path_as_tuples,
                       plant_parameters=asdict(sim.engine.ego.params),
                       controller=dict(Q=control.controller.lat.Q.tolist(), R=control.controller.lat.R.tolist(),
                                       feedback_horizon_s=control.controller.lat.feedback_horizon_s,
                                       discretization=control.controller.lat.discretization))
        previous_s = 0.0
        for step in range(10000):
            previous_command_time = sim.last_command_time
            applied = ControlCommand(brake=1.) if sim._command_is_stale() else sim.command
            sim._advance_once()
            stamp = sim.engine.sim_time
            spin_until(lambda: control.last_control_stamp is not None
                       and abs(control.last_control_stamp-stamp) < 1e-6)
            # Wait for delivery, not an arbitrary count of executor callbacks:
            # /clock adds callbacks and a fixed count can starve commands.
            spin_until(lambda: sim.last_command_time != previous_command_time)
            expected = control.controller.lat.last_ed
            if step % 10 == 0:
                planner._request_plan()
            tracker = control.controller.lat.tracker
            progress = control.controller.lat.route_s
            payload["states"].append(dict(state=asdict(sim.engine.get_state()),
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
            if steering_profile == "assumed":
                assert sim.engine.steering.peak_rate_rad_s <= sim.engine.config.steering.rate_limit_rad_s+1e-9
            if step == 200:
                assert progress > 80, (progress, sim.command, control.controller.lon.target_speed)
            if progress >= 10*tracker.geometry.length:
                break
        else:
            pytest.fail("did not finish ten laps")
        print(f"DDS acceptance: laps=10 steps={step+1} sim_s={sim.engine.sim_time:.2f} "
              f"route_s_m={progress:.3f} collision=False offroad=False")
        assert measured[-1]["protocol"] == "route_projection_v2"
        assert measured[-1]["route_s_m"] > 9*tracker.geometry.length

        old_plan = Path(sequence=encode_sequence(initial_run, 999))
        sim._on_reset(None, object())
        spin_until(lambda: control.active_run == sim.run_id and planner.active_run == sim.run_id)
        assert sim.run_id != initial_run
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
        archive_acceptance("dds_ten_laps_"+steering_profile, payload, passed)


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
            deadline = time.monotonic()+40
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.05)
                assert process.poll() is None, (tmp_path/"launch.log").read_text()
                if metrics and metrics[-1]["timestamp"] >= 15:
                    break
            assert contexts and metrics and statuses, (tmp_path/"launch.log").read_text()
            assert contexts[-1]["target_speed_kmh"] == 50
            assert contexts[-1]["num_lanes"] == 3
            assert contexts[-1]["vehicle_model"] == "dynamic"
            assert metrics[-1]["timestamp"] >= 15
            assert contexts[-1]["steering_parameters"]["mode"] == ("dynamic" if steering_profile == "assumed" else "ideal")
            assert metrics[-1]["route_s_m"] > 20
            assert not any(s.collision or s.offroad for s in statuses)
            assert states and commands
            if steering_profile == "assumed":
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
