import math
import time

import numpy as np
import pytest

from lightweight_sim.engine.reference_line.dp_qp_planner import (
    DPQPPathPlanner, create_local_planner,
)
from lightweight_sim.engine.reference_line.route_aware_planner import BaselinePathPlanner
from lightweight_sim.engine.algorithms.planner.dp_qp import (
    Quadratic_planning, jerk_matrices, adaptive_qp_knots, adaptive_dp_knots, quintic_edge,
)
from lightweight_sim.engine.simulator.data_types import VehicleState


def planner(**kwargs):
    options = dict(lane_width=3.5, num_lanes=3,
                   reference_lane_index=1, target_lane=1)
    options.update(kwargs)
    return DPQPPathPlanner([(float(x), 0., 0., 0.) for x in range(201)], **options)


def test_factory_preserves_baseline_and_rejects_unknown_algorithm():
    args = dict(global_frenet_path=[(0, 0, 0, 0), (100, 0, 0, 0)],
                lane_width=3.5, num_lanes=2, reference_lane_index=0, target_lane=0)
    assert isinstance(create_local_planner('baseline', **args), BaselinePathPlanner)
    assert isinstance(create_local_planner('dp_qp', **args), DPQPPathPlanner)
    with pytest.raises(ValueError):
        create_local_planner('invalid', **args)


def test_dp_qp_detours_and_returns_to_reference():
    p = planner()
    obstacles = [(35., 0., 4., 2., 0., 0.)]
    result = p._plan((0., 0.), (0., 0.), obstacles)
    assert p.last_status == 'solved'
    assert result and p._trajectory_is_safe(result, obstacles)
    assert max(abs(point[1]) for point in result) > 2.5
    assert abs(result[-1][1]) < 1.
    assert all(abs(point[1]) < 4.15 for point in result)
    assert max(math.dist(a[:2], b[:2]) for a, b in zip(result, result[1:])) < .55


def test_full_width_blockage_fails_instead_of_publishing_collision_path():
    p = planner()
    assert p._plan((0., 0.), (0., 0.), [(25., 0., 5., 20., 0., 0.)]) == []
    assert p.last_status == 'dp_infeasible'


def test_lane_free_corridor_can_use_non_lane_center_gap():
    # Exactly one nominal lane, but map boundaries provide a wider drivable road.
    # A 2.6m detour is feasible; the old candidate set only has l=0.
    p = planner(num_lanes=1, reference_lane_index=-1, target_lane=-1,
                drivable_left_boundary=[(x, 4.8) for x in range(201)],
                drivable_right_boundary=[(x, -1.5) for x in range(201)])
    obstacles = [(35., 0., 4., 1., 0., 0.)]
    result = p._plan((0., 0.), (0., 0.), obstacles)
    assert p.last_status == 'solved'
    assert max(point[1] for point in result) > 2.4
    assert p._trajectory_is_safe(result, obstacles)


def test_qp_is_constrained_optimization_and_keeps_initial_derivatives():
    knots = np.linspace(0, 40, 11)
    samples = np.linspace(0, 40, 81)
    lower, upper = np.full(81, -4.), np.full(81, 4.)
    lower[(samples >= 15) & (samples <= 25)] = 2.
    start = (0., .05, 0.)
    result = Quadratic_planning(knots, samples, start, np.where(lower > 0, 3., 0.), lower, upper, 0.)
    assert result is not None
    l, dl, ddl = result
    assert np.all(l >= lower-1e-5) and np.all(l <= upper+1e-5)
    assert (l[0], dl[0], ddl[0]) == pytest.approx(start)
    assert not np.allclose(l, (lower+upper)/2)
    lower[0] = 1.
    assert Quadratic_planning(knots, samples, start, l, lower, upper, 0.) is None


def test_piecewise_jerk_integration_is_c2():
    knots = np.array([0., 4., 8.])
    samples = np.array([4.-1e-8, 4., 4.+1e-8])
    matrices, constants = jerk_matrices(knots, samples, (1., .2, -.1))
    for matrix, constant in zip(matrices, constants):
        values = constant+matrix@np.array([.02, -.03])
        assert max(values)-min(values) < 1e-7


def test_async_request_carries_actual_vehicle_state():
    p = planner()
    p.start()
    try:
        assert p.plan(VehicleState(phi=.04, vx=5.), [])
        deadline = time.monotonic()+3
        while not p.poll_result() and time.monotonic() < deadline:
            time.sleep(.01)
        assert p.get_result()
        assert p._planning_state == (.04, 5., 0., 0.)
    finally:
        p.stop()


def test_sparse_reference_uses_distance_not_vertex_number():
    p = DPQPPathPlanner([(0, 0, 0, 0), (100, 0, 0, 0)],
                        lane_width=3.5, num_lanes=3, reference_lane_index=1, target_lane=1)
    result = p._plan((22, 0), (20, 0), [(55, 0, 4, 2, 0, 0)])
    assert p.last_status == 'solved'
    assert result[0][:2] == pytest.approx((20, 0))
    assert max(abs(point[1]) for point in result) > 2.5


def test_obstacles_outside_local_horizon_are_not_clamped_to_endpoints():
    p = planner()
    result = p._plan((0, 0), (0, 0), [(-100, 0, 4, 2, 0, 0), (200, 0, 4, 2, 0, 0)])
    assert p.last_status == 'solved'
    assert max(abs(point[1]) for point in result) < 1e-6


def test_curved_reference_and_closed_route_seam():
    radius = 50.
    angles = np.linspace(0, 2*math.pi, 315)
    ref = [(radius*math.sin(a), radius*(1-math.cos(a)), a, 1/radius) for a in angles]
    p = DPQPPathPlanner(ref, lane_width=3.5, num_lanes=3,
                        reference_lane_index=1, target_lane=1)
    start = ref[-10][:2]
    result = p._plan(start, start, [])
    assert p.last_status == 'solved'
    assert len(result) > 100
    assert max(math.dist(a[:2], b[:2]) for a, b in zip(result, result[1:])) < .51
    assert all(math.isfinite(value) for point in result for value in point)


def test_narrowing_drivable_corridor_is_a_hard_constraint():
    left = [(x, 5.25 if x < 50 else 2.) for x in range(201)]
    right = [(x, -5.25 if x < 50 else -2.) for x in range(201)]
    p = planner(drivable_left_boundary=left, drivable_right_boundary=right)
    result = p._plan((0, 0), (0, 0), [(25, 0, 4, 2, 0, 0)])
    assert p.last_status == 'solved'
    assert max(abs(point[1]) for point in result if point[0] >= 50) <= .9+1e-5


def test_qp_failure_does_not_switch_to_baseline(monkeypatch):
    import lightweight_sim.engine.reference_line.dp_qp_planner as module
    monkeypatch.setattr(module.LateralQpSmoother, 'solve', lambda *a, **k: None)
    p = planner()
    assert p._plan((0, 0), (0, 0), []) == []
    assert p.last_status == 'qp_failed'


def test_replanning_through_detour_with_lqr_tracking():
    from lightweight_sim.engine.algorithms.controller.combined import VehicleController
    from lightweight_sim.engine.simulator.vehicle import EgoVehicle, VehicleParams
    from lightweight_sim.engine.simulator.obstacle import ObstacleManager
    from lightweight_sim.engine.simulator.data_types import Obstacle

    p = planner()
    vehicle = EgoVehicle(VehicleState(vx=5.56), VehicleParams())
    controller = VehicleController(target_speed_kmh=20)
    manager = ObstacleManager()
    manager.add_obstacle(Obstacle(id=1, x=35, y=0, length=4, width=2))
    state = VehicleState(vx=5.56)
    for tick in range(360):
        if tick % 10 == 0:
            p._planning_state = (state.phi, state.vx, state.vy, state.r)
            path = p._plan(state.position, state.position, [(35, 0, 4, 2, 0, 0)])
            assert path, (tick, p.last_status)
            controller.update_ref_path(path, reset=False)
        steer, throttle, brake = controller.step(state.x, state.y, state.phi, state.vx, state.vy, state.r)
        state = vehicle.step(steer, throttle*3-brake*6, .05)
        assert not manager.check_collision(state.x, state.y, vehicle.length, vehicle.width, state.phi)
        assert abs(state.y) < 4.15
    assert state.x > 95
    assert abs(state.y) < .1


def test_adaptive_mesh_preserves_start_and_obstacle_resolution():
    knots = adaptive_qp_knots(79., 4., [35.])
    assert np.max(np.diff(knots)[knots[:-1] < 12.]) <= .5
    assert np.max(np.diff(knots)[abs(knots[:-1]-35.) < 12.]) <= 1.
    assert knots[0] == 0. and knots[-1] == 79.
    dp = adaptive_dp_knots(79., 8., [35.])
    assert max(np.diff(dp)) <= 8.


def test_quintic_dp_edge_preserves_nonzero_terminal_slope():
    start, end = (.3, .1, -.02), (2., .2, .03)
    l, dl, ddl = quintic_edge(8., start, end, np.array([0., 8.]))
    assert (l[0], dl[0], ddl[0]) == pytest.approx(start)
    assert (l[-1], dl[-1], ddl[-1]) == pytest.approx(end)


def test_qp_reports_infeasibility_and_honors_solver_budget():
    samples = np.linspace(0, 40, 81)
    knots = adaptive_qp_knots(40, 4.)
    diagnostics = {}
    result = Quadratic_planning(knots, samples, (0., 0., 0.), np.zeros(81),
                                np.ones(81), np.full(81, 4.), 0., diagnostics=diagnostics)
    assert result is None
    assert 'infeasible' in diagnostics['status']
    assert diagnostics['solve_time_s'] < .1


def test_scene_two_full_replanning_detour_and_return():
    from lightweight_sim.engine.algorithms.controller.combined import VehicleController
    from lightweight_sim.engine.simulator.vehicle import EgoVehicle, VehicleParams
    from lightweight_sim.engine.simulator.obstacle import ObstacleManager
    from lightweight_sim.engine.simulator.data_types import Obstacle

    ref = [(float(x), -1.75, 0., 0.) for x in range(201)]
    p = DPQPPathPlanner(ref, lane_width=3.5, num_lanes=2,
        reference_lane_index=0, target_lane=0,
        drivable_left_boundary=[(x, 3.5) for x in range(201)],
        drivable_right_boundary=[(x, -3.5) for x in range(201)])
    state = VehicleState(x=20., y=-1.75, vx=10.)
    vehicle = EgoVehicle(state, VehicleParams())
    controller = VehicleController(target_speed_kmh=34.)
    manager = ObstacleManager()
    manager.add_obstacle(Obstacle(id=1, x=60, y=-1.75, length=4.5, width=2))
    max_y = state.y
    for tick in range(400):
        if tick % 2 == 0:
            p._planning_state = (state.phi, state.vx, state.vy, state.r)
            path = p._plan(state.position, state.position, [(60, -1.75, 4.5, 2, 0, 0)])
            assert path, (tick, p.last_status, p.last_qp_diagnostics)
            controller.update_ref_path(path, reset=False)
            curvature = max((abs(point[3]) for point in path
                             if math.dist(point[:2], state.position) <= 20.), default=0.)
            controller.set_target_speed(min(34., math.sqrt(2./curvature)*3.6)
                                        if curvature > 1e-6 else 34.)
        steer, throttle, brake = controller.step(state.x, state.y, state.phi, state.vx, state.vy, state.r)
        state = vehicle.step(steer, throttle*3.-brake*6., .05)
        max_y = max(max_y, state.y)
        assert not manager.check_collision(state.x, state.y, vehicle.length, vehicle.width, state.phi)
        # Entire rotated vehicle stays inside the physical two-lane road.
        half_extent = vehicle.width/2*abs(math.cos(state.phi))+vehicle.length/2*abs(math.sin(state.phi))
        assert -3.5 <= state.y-half_extent and state.y+half_extent <= 3.5
        if state.x > 170.: break
    assert state.x > 170.
    assert max_y > .5
    assert abs(state.y+1.75) < .25
