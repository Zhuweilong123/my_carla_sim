"""ROS speed/path pairing and asynchronous planning acceptance."""
import json
import math
import os
import signal
import subprocess
import time
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from lightweight_sim_msgs.msg import (ControlCommand, Path, PathPoint, SpeedProfile, SpeedPoint,
                                      SimulationStatus, VehicleState, ReferenceLine, RouteSegment)
from std_msgs.msg import String
from lightweight_sim.engine.ros_nodes.controller_node import ControllerNode
from lightweight_sim.engine.ros_nodes.safe_stop_node import SafeStopNode
from lightweight_sim.engine.ros_nodes.speed_planner_node import SpeedPlannerNode, projected_speed_limits
from lightweight_sim.engine.ros_nodes.route_session import encode_sequence
from lightweight_sim.engine.ros_nodes.qos import latched_path_qos, sensor_data_qos, status_qos


def profile(node, version, stamp=None):
    message = SpeedProfile(run_id=1, path_sequence=encode_sequence(1, version), valid=True, status='solved')
    message.header.stamp = stamp or node.get_clock().now().to_msg()
    message.points = [SpeedPoint(time_from_start=t, s=5*t, speed=5.) for t in (0., 1., 2.)]
    return message


def path(node, version, empty=False):
    message = Path(sequence=encode_sequence(1, version))
    message.header.stamp = node.get_clock().now().to_msg()
    message.points = [] if empty else [PathPoint(x=x) for x in (0., 100.)]
    return message


def test_controller_activates_only_coherent_pairs_and_invalidates_empty_paths():
    rclpy.init()
    node = ControllerNode()
    try:
        node.timer.cancel()
        node.set_parameters([Parameter('speed_planning_enabled', value=True)])
        node.active_run = 1
        node.reference_path = [(0., 0., 0., 0.), (100., 0., 0., 0.)]
        node._on_planned(path(node, 1))
        assert not node.plan_ready
        node._on_speed(profile(node, 1))
        assert node.plan_ready and node.last_sequence == encode_sequence(1, 1)
        previous = node.speed_reference
        node._on_planned(path(node, 2))
        assert node.speed_reference is previous and node.last_sequence == encode_sequence(1, 1)
        node._on_speed(profile(node, 2))
        assert node.last_sequence == encode_sequence(1, 2)
        node._on_speed(profile(node, 3))  # DDS can deliver speed before geometry.
        assert node.last_sequence == encode_sequence(1, 2)
        node._on_planned(path(node, 3))
        assert node.last_sequence == encode_sequence(1, 3)
        node._on_planned(path(node, 4, empty=True))
        assert not node.plan_ready and node.speed_reference is None
        node._on_speed(profile(node, 3))
        assert not node.plan_ready
        node._on_speed(profile(node, 5))
        node._on_planned(path(node, 5))
        assert node.plan_ready
        stale = profile(node, 6)
        stale.header.stamp.sec -= 2
        node._on_speed(stale)
        assert node.last_sequence == encode_sequence(1, 5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_full_pid_braking_preserves_history_but_protective_braking_resets():
    rclpy.init()
    node = ControllerNode()
    try:
        node.timer.cancel()
        pid = node.controller.lon
        pid._previous_accel = -6.
        pid._previous_error = -1.
        pid._filtered_derivative = -.5
        node._publish_command(0., 0., 1., False)
        assert pid._previous_accel == -6.
        assert pid._previous_error == -1.
        assert pid._filtered_derivative == -.5
        node._publish_command(0., 0., 1.)
        assert pid._previous_accel == 0.
        assert pid._previous_error is None
        assert pid._filtered_derivative == 0.
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_supervisor_requires_speed_heartbeat_only_for_cruise():
    rclpy.init()
    node = SafeStopNode()
    try:
        node.set_parameters([Parameter('require_speed_plan', value=True)])
        node._on_context(String(data=json.dumps(dict(schema_version=1, run_id=1))))
        node._on_plan(path(node, 1))
        assert 'speed' in node._stop_reason()
        node._on_speed(profile(node, 1))
        assert not node._stop_reason()
        node._speed_received_at -= 1.
        assert 'speed' in node._stop_reason()
        node._run_context['maneuver'] = 'reverse_parking'
        assert not node._stop_reason()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_speed_reuse_is_bounded_and_invalidated_by_obstacle_changes():
    from types import SimpleNamespace
    from lightweight_sim.engine.algorithms.planner.st_speed import STSpeedPlanner
    from lightweight_sim.engine.simulator.data_types import VehicleState as State, Obstacle
    rclpy.init()
    node = SpeedPlannerNode()
    sent = []
    try:
        node.pub = SimpleNamespace(publish=sent.append)
        node._now = lambda: 100.
        node.state = State(vx=5, timestamp=100.)
        node.path = path(node, 1)
        node.obstacle_time = 100.
        node.request_obstacles = ()
        plan = STSpeedPlanner().plan([(0., 0., 0., 0.), (100., 0., 0., 0.)], node.state)
        assert plan.valid
        node._publish(plan, 1, encode_sequence(1, 1), 100., 'solved')
        node._now = lambda: 100.2
        node._publish(None, 1, encode_sequence(1, 2), 100., 'late_result')
        assert sent[-1].valid and sent[-1].status == 'reused:late_result'
        assert sent[-1].path_sequence == encode_sequence(1, 1)
        assert sent[-1].header.stamp.sec == 100
        node.obstacles = [Obstacle(id=1, x=10., y=0.)]
        node._publish(None, 1, encode_sequence(1, 3), 100.2, 'dp_infeasible')
        assert not sent[-1].valid
        node.obstacles = []
        node.obstacle_time = 100.61
        node.state.timestamp = 100.61
        node._now = lambda: 100.61
        node._publish(None, 1, encode_sequence(1, 4), 100.61, 'late_result')
        assert not sent[-1].valid
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_vectorized_route_limit_projection_crosses_closed_route_seam():
    reference = ReferenceLine(success=True)
    reference.points = [PathPoint(x=x, y=y, theta=h) for x, y, h in
        [(0., 0., 0.), (10., 0., math.pi/2), (10., 10., math.pi),
         (0., 10., -math.pi/2), (0., 0., 0.)]]
    reference.segments = [RouteSegment(length_m=10., speed_limit_kmh=float(v), maneuver='straight')
                          for v in (10, 20, 30, 40)]
    local = [(0., 8., -math.pi/2, 0.), (0., 1., -math.pi/2, 0.),
             (0., 0., 0., 0.), (5., 0., 0., 0.), (9., 0., 0., 0.)]
    limits = projected_speed_limits(local, reference, dict(target_speed_ratio=.8))
    assert [limit for _, _, limit in limits] == [32., 32., 8., 8., 8.]


@pytest.mark.parametrize('scenario,steering_profile', [
    ('obstacle', 'ideal'), ('figure_eight', 'ideal'), ('figure_eight', 'assumed'), ('default', 'ideal')])
def test_installed_speed_planning_graph(tmp_path, scenario, steering_profile):
    """Actual separate processes, DDS, simulation clock and standard config."""
    rclpy.init()
    observer = rclpy.create_node('speed_acceptance', namespace='st_acceptance')
    tracking, speed_profiles, statuses, diagnostics = [], [], [], []
    paths, states = {}, {}
    metrics, commands = [], []
    observer.create_subscription(String, 'tracking/metrics', lambda m: metrics.append(json.loads(m.data)), sensor_data_qos())
    observer.create_subscription(ControlCommand, 'control_command', lambda m: commands.append(dict(
        timestamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
        throttle=m.throttle, brake=m.brake)), sensor_data_qos())
    observer.create_subscription(Path, 'planned_path', lambda m: paths.update({m.sequence:
        [[p.x, p.y, p.theta, p.kappa] for p in m.points]}), latched_path_qos())
    observer.create_subscription(VehicleState, 'vehicle/state', lambda m: states.update({
        round(m.header.stamp.sec+m.header.stamp.nanosec/1e9, 5):
        dict(x=m.x, y=m.y, phi=m.yaw, vx=m.vx, vy=m.vy, r=m.yaw_rate, accel=m.acceleration)}), sensor_data_qos())
    observer.create_subscription(String, 'speed/tracking', lambda m: tracking.append(json.loads(m.data)), sensor_data_qos())
    observer.create_subscription(SpeedProfile, 'speed_profile', speed_profiles.append, latched_path_qos())
    observer.create_subscription(SimulationStatus, 'sim/status', statuses.append, status_qos())
    observer.create_subscription(String, 'speed/diagnostics', lambda m: diagnostics.append(json.loads(m.data)), sensor_data_qos())
    process = None
    try:
        with (tmp_path/'launch.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'lightweight_sim', 'lightweight_sim.launch.py',
                'namespace:=st_acceptance', 'gui:=false', 'scenario:='+scenario, 'steering_profile:='+steering_profile],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+65
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=.025)
                if statuses and (statuses[-1].reached or statuses[-1].sim_time >= 30.):
                    break
            assert process.poll() is None
            failures = []
            seen = set()
            for p in speed_profiles:
                stamp = round(p.header.stamp.sec+p.header.stamp.nanosec/1e9, 5)
                if not p.valid and p.status not in seen and stamp in states and p.path_sequence in paths:
                    failures.append(dict(status=p.status, state=states[stamp], path=paths[p.path_sequence]))
                    seen.add(p.status)
            (tmp_path/'speed.json').write_text(json.dumps(dict(tracking=tracking, diagnostics=diagnostics,
                failures=failures, metrics=metrics, commands=commands, states=states)))
            assert tracking and speed_profiles and statuses, (tmp_path/'launch.log').read_text()
            assert max(t['measured_speed_mps'] for t in tracking) > 2.
            assert statuses[-1].sim_time >= 15.
            assert not any(s.collision or s.offroad for s in statuses)
            if scenario == 'obstacle':
                assert max(s['x'] for s in states.values()) > 65., 'did not pass scene 2 obstacle'
                errors = [abs(m['speed_error_kmh']) for m in metrics if m['timestamp'] > 4.]
                assert errors, 'missing executed-reference speed metrics'
                assert math.sqrt(sum(e*e for e in errors)/len(errors)) < .6
                assert sorted(errors)[int(.95*(len(errors)-1))] < 1.
                assert max(errors) < 2., 'large longitudinal oscillation'
            recent = [t['measured_speed_mps'] for t in tracking[-60:]]
            if scenario == 'default' or statuses[-1].reached:
                assert statuses[-1].reached
                assert min(recent) < .2
            else:
                assert min(recent) > .5, diagnostics[-20:]
            if scenario == 'figure_eight':
                assert min(math.hypot(s['vx'], s['vy']) for t, s in states.items() if t > 4.) > .5
            assert any(p.valid for p in speed_profiles)
            print('ST graph', scenario, 'sim_s', statuses[-1].sim_time,
                  'recent_speed', min(recent), max(recent), 'statuses', {d['status'] for d in diagnostics})
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        observer.destroy_node()
        rclpy.shutdown()
