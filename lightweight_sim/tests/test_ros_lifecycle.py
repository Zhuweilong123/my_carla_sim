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
from lightweight_sim_msgs.msg import Path, VehicleState as RosState, ControlCommand as RosCommand
from lightweight_sim_msgs.msg import SimulationStatus
from std_msgs.msg import String
from lightweight_sim.engine.ros_nodes.simulator_node import SimulatorNode
from lightweight_sim.engine.ros_nodes.planner_node import PlannerNode
from lightweight_sim.engine.ros_nodes.controller_node import ControllerNode
from lightweight_sim.engine.ros_nodes.route_session import encode_sequence
from lightweight_sim.engine.ros_nodes.qos import status_qos, sensor_data_qos, latched_path_qos, command_qos
from lightweight_sim.engine.ros_nodes.message_conversions import message_to_state
from lightweight_sim.engine.simulator.data_types import ControlCommand
from lightweight_sim.engine.analysis.evaluation import provenance, archive_sources


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
