"""Physical constraints and lifecycle scenarios for the independent speed solver."""
import math
import numpy as np
import pytest
from lightweight_sim.engine.algorithms.planner.st_speed import STSpeedPlanner, SpeedConfig
from lightweight_sim.engine.simulator.data_types import VehicleState, Obstacle, VehicleParams
from lightweight_sim.engine.algorithms.controller.lon_pid import LongitudinalPIDController


def straight(length=100., curvature=0.):
    return [(float(s), 0., 0., curvature) for s in np.arange(0., length+.01, .5)]


def check_physics(plan):
    assert plan.valid, plan.status
    assert np.all(np.diff(plan.s) >= -.003)
    assert np.max(np.abs(plan.jerk)) <= 3.003
    for t in np.linspace(0, plan.time[-1], 601):
        s, v, a = plan.sample(t)
        assert v >= 0
        assert -3.003 <= a <= 2.003
        if plan.stop_s is not None:
            assert s <= plan.stop_s+.003


def test_start_cruise_and_curvature_cap():
    solver = STSpeedPlanner()
    start = solver.plan(straight(), VehicleState())
    check_physics(start)
    assert start.sample(.1)[2] > .1
    cruise = solver.plan(straight(), VehicleState(vx=8), target_speed_kmh=36)
    check_physics(cruise)
    assert cruise.speed[-1] == pytest.approx(10., abs=.01)
    curved = solver.plan(straight(curvature=.1), VehicleState(vx=3), max_lateral_accel=2)
    check_physics(curved)
    assert max(curved.speed) <= math.sqrt(20)+.01


def test_static_obstacle_stop_hold_and_restart():
    solver = STSpeedPlanner()
    obstacle = Obstacle(id=1, x=30, y=0)
    stop = solver.plan(straight(), VehicleState(vx=5), [obstacle])
    check_physics(stop)
    assert stop.s[-1] < 30-(VehicleParams().length+obstacle.length)/2
    assert stop.speed[-1] == pytest.approx(0, abs=.003)
    hold = solver.plan(straight(), VehicleState(x=stop.stop_s, accel=-6), [obstacle])
    check_physics(hold)
    assert max(hold.speed) < .003
    resume = solver.plan(straight(), VehicleState(x=stop.stop_s, accel=-6), [])
    check_physics(resume)
    assert resume.sample(.1)[2] > 0


def test_true_destination_differs_from_truncated_local_path():
    solver = STSpeedPlanner()
    path = straight(40)
    local = solver.plan(path, VehicleState(vx=5), destination=(100, 0))
    check_physics(local)
    assert local.stop_s is None and local.speed[-1] > 5
    goal = solver.plan(path, VehicleState(vx=5), destination=(25, 0))
    check_physics(goal)
    assert goal.s[-1] == pytest.approx(25, abs=.05)
    assert goal.speed[-1] < .003
    closed = solver.plan(path, VehicleState(vx=5), destination=(25, 0), closed_route=True)
    assert closed.valid and closed.stop_s is None


def test_detoured_static_obstacle_is_not_a_stop_boundary():
    result = STSpeedPlanner().plan(straight(), VehicleState(vx=5),
                                  [Obstacle(id=1, x=30, y=4)])
    check_physics(result)
    assert result.stop_s is None


def test_spatial_speed_limit_and_overspeed_recovery():
    solver = STSpeedPlanner()
    limited = solver.plan(straight(), VehicleState(vx=5), speed_limits=[(20., 100., 18.)])
    check_physics(limited)
    for t in np.linspace(0, limited.time[-1], 200):
        s, v, _ = limited.sample(t)
        if s >= 20:
            assert v <= 5.12
    overspeed = solver.plan(straight(curvature=.1), VehicleState(vx=8), max_lateral_accel=2)
    check_physics(overspeed)
    assert overspeed.speed[0] == pytest.approx(8)
    assert overspeed.speed[-1] <= math.sqrt(20)+.01


def test_infeasible_and_moving_conflicts_fail_closed():
    solver = STSpeedPlanner()
    near = solver.plan(straight(), VehicleState(vx=10), [Obstacle(id=1, x=5, y=0)])
    assert not near.valid
    moving = solver.plan(straight(), VehicleState(), [Obstacle(id=1, x=30, y=0, speed=2)])
    assert not moving.valid and moving.status == 'unsupported_moving_conflict'
    invalid = solver.plan(straight(), VehicleState(vx=float('nan')))
    assert not invalid.valid
    with pytest.raises(ValueError):
        SpeedConfig(max_jerk=0)


def test_pid_feedforward_and_setpoint_step_without_derivative_kick():
    pid = LongitudinalPIDController(K_P=0, K_I=0, K_D=1)
    pid.set_target(18)
    assert pid.control(5, reference_accel=1) == pytest.approx(1)
    pid.set_target(36)
    # Constant measured speed and zero feedforward: no setpoint derivative kick.
    assert pid.control(5, reference_accel=0) == pytest.approx(0)
    with pytest.raises(ValueError):
        pid.control(5, reference_accel=float('nan'))


def test_replanning_closed_loop_stops_and_restarts_with_actual_acceleration():
    from lightweight_sim.engine.simulator.vehicle import EgoVehicle
    solver, pid = STSpeedPlanner(), LongitudinalPIDController()
    ego = EgoVehicle(VehicleState())
    obstacle = Obstacle(id=1, x=30, y=0)
    reference = None
    origin_time = 0.
    for tick in range(500):
        state = ego.get_state()
        if tick % 2 == 0:
            reference = solver.plan(straight(), state, [obstacle])
            assert reference.valid, (tick, state, reference.status)
            origin_time = state.timestamp
        _, speed, acceleration = reference.sample(state.timestamp-origin_time+.05)
        if speed < .05 and acceleration <= .01 and state.speed < .15:
            command = -6.
            pid.reset()
        else:
            pid.set_target(speed*3.6)
            command = pid.control(state.speed, reference_accel=acceleration)
        ego.kinematic_step(0., command, .05)
        assert ego.get_state().x < 30-(ego.length+obstacle.length)/2
        if state.x > 24.5 and state.speed < .15:
            break
    stopped = ego.get_state()
    assert stopped.x > 24.5 and stopped.speed < .15, stopped
    restart = solver.plan(straight(), stopped, [])
    assert restart.valid and restart.sample(.1)[2] > 0.


def test_initial_feedback_acceleration_is_preserved_above_comfort_limit():
    result = STSpeedPlanner().plan(straight(), VehicleState(vx=1, accel=2.1))
    assert result.valid, result.status
    assert result.accel[0] == pytest.approx(2.1, abs=.003)
    assert result.accel[1] <= 2.003
    assert max(np.abs(result.jerk)) <= 3.003


def test_qp_can_decelerate_before_future_curve_without_dp_timing_lower_bound():
    path = [(float(s), 0., 0., .01+.07*math.exp(-(s-20.)**2/200.))
            for s in np.arange(0., 80., .5)]
    result = STSpeedPlanner().plan(path, VehicleState(vx=6.775, accel=-.814), target_speed_kmh=34.)
    check_physics(result)
    assert result.speed[-1] < result.speed[0]
