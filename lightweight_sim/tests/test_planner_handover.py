import math

import numpy as np
import pytest

from lightweight_sim.engine.algorithms.planner.handover import PathHandover
from lightweight_sim.engine.simulator.data_types import VehicleState, Obstacle
from lightweight_sim.engine.reference_line.dp_qp_planner import DPQPPathPlanner


def straight():
    return [(x*.5, 0., 0., 0.) for x in range(161)]


def manager():
    h = PathHandover()
    h.accept(straight(), 0.)
    return h


def test_prediction_uses_measured_progress_acceleration_and_keeps_committed_prefix():
    h = manager()
    state = VehicleState(x=10., vx=10., accel=2.)
    predicted = h.prepare(state, .05)
    assert predicted.x == 12.  # 1.5225m travel, rounded forward to an old vertex.
    assert predicted.vx == pytest.approx(10.3)
    assert state.x == 10.  # Input snapshot is never mutated.
    tail = [(12., 0., 0., 0.), (12.5, .001, .004, .008), (13., .005, .01, .01)]
    stitched = h.splice(tail, VehicleState(x=11., vx=10.), .15)
    assert stitched[:5] == straight()[20:25]
    assert stitched[5:] == tail[1:]


def test_curved_path_prediction_preserves_tangent_and_curvature():
    radius = 50.
    angles = np.linspace(0., .8, 81)
    path = [(radius*math.sin(a), radius*(1-math.cos(a)), a, 1/radius) for a in angles]
    h = PathHandover()
    h.accept(path, 0.)
    state = VehicleState(x=path[10][0], y=path[10][1], phi=path[10][2], vx=10., r=.2)
    predicted = h.prepare(state, .05)
    assert predicted is not None
    join = h.request.join
    assert predicted.position == join[:2]
    assert predicted.phi == join[2]
    assert predicted.r/predicted.speed == pytest.approx(join[3])


def test_late_passed_and_mismatched_results_are_rejected():
    h = manager()
    h.prepare(VehicleState(x=10., vx=10.), .05)
    join = h.request.join
    tail = [join, (join[0]+.5, 0., 0., 0.)]
    assert h.splice(tail, VehicleState(x=11., vx=10.), .25) is None
    assert h.status == 'late_result'
    assert h.splice(tail, VehicleState(x=join[0]+.1, vx=10.), .1) is None
    assert h.status == 'late_result'
    assert h.splice(tail, VehicleState(x=11., y=.5, vx=10.), .1) is None
    assert h.status == 'tracking_mismatch'
    assert h.splice([join[:2]+(.4, .2), tail[1]], VehicleState(x=11., vx=10.), .1) is None
    assert h.status == 'invalid_join'


def test_old_path_reuse_has_finite_age_and_requires_tracking_and_remaining_distance():
    h = manager()
    assert h.reusable(VehicleState(x=11., vx=10.), .2)
    assert not h.reusable(VehicleState(x=11., vx=10.), .51)
    assert not h.reusable(VehicleState(x=11., y=.5, vx=10.), .2)
    assert not h.reusable(VehicleState(x=79., vx=10.), .2)


def test_bounded_recovery_does_not_extend_old_path_lifetime():
    h = manager()
    state = VehicleState(x=11., y=.4, vx=10.)
    assert not h.reusable(state, .2)
    assert h.reusable(state, .2, recovery=True)
    assert not h.reusable(state, .51, recovery=True)
    assert not h.reusable(VehicleState(x=11., y=.8, vx=10.), .2, recovery=True)


def test_recovery_waits_for_three_stable_requests_before_nominal_prediction():
    h = manager()
    assert h.prepare(VehicleState(x=10., y=.4, vx=10.), .05) is None
    assert h.recovering
    state = VehicleState(x=10., vx=10.)
    assert h.prepare(state, .1) is None
    assert h.prepare(state, .15) is None
    assert h.prepare(state, .2) is not None
    assert not h.recovering


def test_latency_adapts_without_exceeding_configured_prediction_horizon():
    h = manager()
    h.observe_latency(.3, 0.)
    h.prepare(VehicleState(x=10., vx=10.), .3)
    assert h.request.deadline-h.request.requested_at == pytest.approx(.35)
    h.observe_latency(1., 0.)
    h.prepare(VehicleState(x=10., vx=10.), .3)
    assert h.request.deadline-h.request.requested_at == pytest.approx(.4)
    h.observe_latency(1.01, 1.)
    assert h.latency > .8


def test_spliced_path_is_revalidated_against_new_obstacles_and_road_edges():
    p = DPQPPathPlanner(straight(), lane_width=3.5, num_lanes=3, reference_lane_index=1, target_lane=1)
    assert p.validate_path(straight(), [])
    # A newly detected obstacle in the retained prefix must invalidate the result.
    assert not p.validate_path(straight(), [Obstacle(1, 5., 0.)])
    assert not p.validate_path([(0., 5., 0., 0.), (1., 5., 0., 0.)], [])


def test_real_qp_tail_starts_at_future_committed_state_and_keeps_prefix():
    p = DPQPPathPlanner(straight(), lane_width=3.5, num_lanes=3, reference_lane_index=1, target_lane=1)
    h = manager()
    predicted = h.prepare(VehicleState(x=10., vx=10.), .05)
    p._planning_state = (predicted.phi, predicted.vx, predicted.vy, predicted.r)
    p._planning_handover = True
    tail = p._plan(predicted.position, predicted.position, [(35., 0., 4., 2., 0., 0.)])
    assert tail and p.last_status == 'solved'
    stitched = h.splice(tail, VehicleState(x=10.5, vx=10.), .1)
    assert stitched is not None
    assert stitched[:len(h.request.prefix)] == h.request.prefix
    assert p.validate_path(stitched, [Obstacle(1, 35., 0., length=4.)])


def test_qp_output_preserves_start_direction_and_curvature_on_curved_reference():
    angles = np.linspace(0., 1.8, 181)
    ref = [(50*math.sin(a), 50*(1-math.cos(a)), a, .02) for a in angles]
    p = DPQPPathPlanner(ref, lane_width=3.5, num_lanes=3, reference_lane_index=1, target_lane=1)
    p._planning_state = (.04, 10., 0., .1)
    p._planning_handover = True
    result = p._plan((0., 1.), (0., 1.), [])
    assert result, p.last_status
    assert result[0][2] == pytest.approx(.04, abs=1e-5)
    assert result[0][3] == pytest.approx(.01, abs=1e-5)


def test_scene_two_delayed_handover_closed_loop():
    from lightweight_sim.engine.algorithms.controller.combined import VehicleController
    from lightweight_sim.engine.simulator.vehicle import EgoVehicle, VehicleParams
    from lightweight_sim.engine.simulator.obstacle import ObstacleManager

    ref = [(float(x), -1.75, 0., 0.) for x in range(201)]
    p = DPQPPathPlanner(ref, lane_width=3.5, num_lanes=2,
        reference_lane_index=0, target_lane=0,
        drivable_left_boundary=[(x, 3.5) for x in range(201)],
        drivable_right_boundary=[(x, -3.5) for x in range(201)])
    state = VehicleState(x=20., y=-1.75, vx=10.)
    vehicle = EgoVehicle(state, VehicleParams())
    controller = VehicleController(target_speed_kmh=34.)
    obstacles = [Obstacle(1, 60., -1.75)]
    manager = ObstacleManager()
    manager.add_obstacle(obstacles[0])
    h = PathHandover()
    p._planning_state = (state.phi, state.vx, state.vy, state.r)
    path = p._plan(state.position, state.position, [(60., -1.75, 4.5, 2., 0., 0.)])
    assert path
    h.accept(path, 0.)
    controller.update_ref_path(path, reset=False)
    pending = None
    accepted = 0
    max_y = state.y
    for tick in range(440):
        now = tick*.05
        if pending is not None and tick % 3 == 2:
            # Two physics steps (100ms) pass while the old path remains active.
            stitched = h.splice(pending, state, now)
            assert stitched is not None, (tick, h.status, state)
            assert p.validate_path(stitched, obstacles), tick
            h.observe_latency(now, h.request.requested_at)
            h.accept(stitched, now)
            controller.update_ref_path(stitched, reset=False)
            accepted += 1
            pending = None
        if tick % 3 == 0:
            predicted = h.prepare(state, now)
            assert predicted is not None, (tick, h.status, state)
            p._planning_state = (predicted.phi, predicted.vx, predicted.vy, predicted.r)
            p._planning_handover = True
            pending = p._plan(predicted.position, predicted.position, [(60., -1.75, 4.5, 2., 0., 0.)])
            assert pending, (tick, p.last_status, p.last_qp_diagnostics)
        curvature = max((abs(point[3]) for point in h.path
                         if math.dist(point[:2], state.position) <= 20.), default=0.)
        controller.set_target_speed(min(34., math.sqrt(2./curvature)*3.6)
                                    if curvature > 1e-6 else 34.)
        steer, throttle, brake = controller.step(state.x, state.y, state.phi, state.vx, state.vy, state.r)
        state = vehicle.step(steer, throttle*3.-brake*6., .05)
        max_y = max(max_y, state.y)
        assert not manager.check_collision(state.x, state.y, vehicle.length, vehicle.width, state.phi)
        half_extent = vehicle.width/2*abs(math.cos(state.phi))+vehicle.length/2*abs(math.sin(state.phi))
        assert -3.5 <= state.y-half_extent and state.y+half_extent <= 3.5
        if state.x > 170.: break
    assert accepted > 30
    assert state.x > 170. and max_y > .5
    assert abs(state.y+1.75) < .25


def test_installed_scene_two_handover(tmp_path):
    """Real /clock and independently launched nodes exercise result admission."""
    rclpy = pytest.importorskip('rclpy')
    import signal
    import subprocess
    import time
    from lightweight_sim_msgs.msg import Path as RosPath, SimulationStatus, VehicleState as RosState
    from lightweight_sim.engine.ros_nodes.qos import sensor_data_qos, latched_path_qos

    rclpy.init()
    observer = rclpy.create_node('handover_acceptance_observer', namespace='handover_acceptance')
    statuses, states = [], []
    counts = dict(valid=0, empty_after_valid=0)
    def receive_path(message):
        if message.points:
            counts['valid'] += 1
        elif counts['valid']:
            counts['empty_after_valid'] += 1
    observer.create_subscription(RosPath, 'planned_path', receive_path, latched_path_qos())
    observer.create_subscription(RosState, 'vehicle/state', states.append, sensor_data_qos())
    observer.create_subscription(SimulationStatus, 'sim/status', statuses.append, sensor_data_qos())
    process = None
    try:
        with (tmp_path/'launch.log').open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'lightweight_sim', 'lightweight_sim.launch.py',
                 'scenario:=obstacle', 'gui:=false', 'namespace:=handover_acceptance'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+45
            while time.monotonic() < deadline and not (statuses and statuses[-1].done):
                rclpy.spin_once(observer, timeout_sec=.05)
                assert process.poll() is None, (tmp_path/'launch.log').read_text()
            assert statuses and statuses[-1].reached, (tmp_path/'launch.log').read_text()
            assert not any(s.collision or s.offroad for s in statuses)
            assert counts['valid'] > 30 and counts['empty_after_valid'] == 0, counts
            assert states and states[-1].x > 170.
    finally:
        observer.destroy_node()
        rclpy.shutdown()
        if process is not None and process.poll() is None:
            import os
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
