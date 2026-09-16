"""Real DDS lifecycle acceptance; run with the ROS overlay sourced.

Physics is explicitly stepped (no wall-time waiting for ten simulated laps).
The actual simulator, planner and controller callbacks communicate over DDS.
"""
import math
import time
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("lightweight_sim_msgs.msg")
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from lightweight_sim_msgs.msg import Path
from lightweight_sim.engine.ros_nodes.simulator_node import SimulatorNode
from lightweight_sim.engine.ros_nodes.planner_node import PlannerNode
from lightweight_sim.engine.ros_nodes.controller_node import ControllerNode
from lightweight_sim.engine.ros_nodes.route_session import encode_sequence


def test_ros_ten_laps_reset_switch_and_stale_plan():
    rclpy.init(args=["--ros-args", "-p", "scenario:=figure_eight"])
    nodes = []
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
        initial_run = sim.run_id
        previous_s = 0.0
        for step in range(10000):
            previous_command_time = sim.last_command_time
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
            assert progress-previous_s < 3.0, "branch jump"
            assert progress-previous_s > -2.0, "unexpected reverse progress"
            previous_s = progress
            assert math.isfinite(expected)
            assert not sim.engine.is_done
            if step == 200:
                assert progress > 80, (progress, sim.command, control.controller.lon.target_speed)
            if progress >= 10*tracker.geometry.length:
                break
        else:
            pytest.fail("did not finish ten laps")
        print(f"DDS acceptance: laps=10 steps={step+1} sim_s={sim.engine.sim_time:.2f} "
              f"route_s_m={progress:.3f} collision=False offroad=False")

        old_plan = Path(sequence=encode_sequence(initial_run, 999))
        sim._on_reset(None, object())
        spin_until(lambda: control.active_run == sim.run_id and planner.active_run == sim.run_id)
        assert sim.run_id != initial_run
        assert control.controller.lat.route_s < 2.0
        accepted_sequence = control.last_sequence
        control._on_planned(old_plan)
        assert control.last_sequence == accepted_sequence
        assert not control.planned_path

        result = sim.set_parameters([Parameter("scenario", value="default")])
        assert result[0].successful
        spin_until(lambda: control.active_run == sim.run_id and planner.active_run == sim.run_id)
        assert control.controller.lon.target_speed == sim.engine.config.target_speed
        assert planner.planner.num_lanes == sim.engine.config.road.num_lanes
        assert control.controller.lat.ts == sim.physics_dt
        assert not control.planned_path
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
