from collections import deque
from dataclasses import replace
import math

import numpy as np
import pytest

from lightweight_sim.engine.simulator.steering import SteeringParams, SteeringActuator, steering_profile
from lightweight_sim.engine.simulator.data_types import VehicleState, ControlCommand
from lightweight_sim.engine.simulator.vehicle import EgoVehicle
from lightweight_sim.engine.simulator.engine import SimulationEngine
from lightweight_sim.engine.simulator.scenarios import make_scenario
from lightweight_sim.engine.algorithms.controller.combined import VehicleController


def test_step_delay_and_lag_analytic_response():
    params = SteeringParams(mode="dynamic", rate_limit_rad_s=100)
    actuator = SteeringActuator(params, 0.5)
    actuator.begin_period(0.1, 0.05)
    assert actuator.advance(0.05) == 0
    actuator.begin_period(0.1, 0.05)
    assert actuator.advance(0.05) == pytest.approx(0.1*(1-math.exp(-0.05/0.15)))


def test_slew_is_bounded_including_transition_to_exponential_and_reversal():
    params = SteeringParams(mode="dynamic", delay_s=0)
    actuator = SteeringActuator(params, 0.5)
    for command in [2.0]*30 + [-2.0]*60:
        actuator.begin_period(command, 0.05)
        before = actuator.angle
        actuator.advance(0.05)
        assert abs(actuator.angle-before) <= params.rate_limit_rad_s*0.05+1e-12
        assert actuator.peak_rate_rad_s <= params.rate_limit_rad_s
        assert abs(actuator.angle) <= 0.5
    # Exact saturated-lag solution must be independent of substep partition.
    a, b = SteeringActuator(params, 0.5), SteeringActuator(params, 0.5)
    a.begin_period(0.3, 0.5)
    b.begin_period(0.3, 0.5)
    a.advance(0.5)
    for _ in range(50):
        b.advance(0.01)
    assert a.angle == pytest.approx(b.angle)


def test_invalid_parameters_and_unrepresentable_delay_rejected():
    for change in (dict(delay_s=-1), dict(time_constant_s=0), dict(rate_limit_rad_s=float("nan"))):
        with pytest.raises(ValueError):
            SteeringParams(**change)
    with pytest.raises(ValueError):
        SteeringParams(mode="dynamic", delay_s=0.075).delay_steps(0.05)
    actuator = SteeringActuator(steering_profile("assumed"), 0.5)
    with pytest.raises(ValueError):
        actuator.begin_period(float("nan"), 0.05)


def transition(error, command, speed, dt, params):
    vehicle = EgoVehicle(VehicleState(vx=speed, y=error[0], vy=error[1]-speed*error[2],
                                     phi=error[2], r=error[3], steer=error[4]))
    actuator = SteeringActuator(params, 0.5, error[4])
    actuator.queue = deque(error[5:])
    actuator.begin_period(command, dt)
    n = max(1, math.ceil(speed*dt/0.5))
    for _ in range(n):
        state = vehicle.step(actuator.advance(dt/n), 0, dt/n, "dynamic")
    return np.array([state.y, state.vy+speed*state.phi, state.phi, state.r,
                     actuator.angle, *actuator.queue])


@pytest.mark.parametrize("speed", [6., 50/3.6, 60/3.6])
@pytest.mark.parametrize("delay", [0., 0.05, 0.1])
def test_augmented_model_matches_independent_physics_difference(speed, delay):
    params = replace(steering_profile("assumed"), delay_s=delay)
    controller = VehicleController(steering_params=params).lat
    controller.update_lqr_gain(speed)
    dim, eps = controller.A.shape[0], 1e-6
    basis = np.eye(dim)
    A = np.column_stack([(transition(basis[i]*eps, 0, speed, 0.05, params)
                         -transition(-basis[i]*eps, 0, speed, 0.05, params))/(2*eps) for i in range(dim)])
    B = (transition(np.zeros(dim), eps, speed, 0.05, params)
         -transition(np.zeros(dim), -eps, speed, 0.05, params))/(2*eps)
    assert np.allclose(controller.A, A, atol=1e-7)
    assert np.allclose(controller.B[:, 0], B, atol=1e-7)
    assert controller.riccati_converged
    cost = np.zeros_like(controller.A)
    cost[:4, :4] = controller.Q
    A, B, P, R = controller.A, controller.B, controller.P, controller.R
    residual = A.T@P@A-A.T@P@B@np.linalg.solve(R+B.T@P@B, B.T@P@A)+cost-P
    assert np.max(abs(residual)) < 1e-7
    assert max(abs(np.linalg.eigvals(A-controller.B@controller.K))) < 1


def test_reset_clears_plant_and_controller_history_and_requires_feedback():
    config = make_scenario("figure_eight")
    config.steering = steering_profile("assumed")
    engine = SimulationEngine(config)
    controller = VehicleController(steering_params=config.steering)
    path = engine.world.ref_path_as_tuples
    controller.update_ref_path(path)
    with pytest.raises(ValueError):
        controller.step(78, 0, math.pi/2, 6, 0, 0)
    engine.step(ControlCommand(steer=0.3))
    engine.step(ControlCommand(steer=0.3))
    assert engine.get_state().steer > 0
    engine.reset()
    assert engine.steering.queue is None
    assert engine.get_state().steer == 0
    controller.lat.command_history = [0.3]
    controller.update_ref_path(path)
    assert controller.lat.command_history == [0.0]


def test_path_refresh_preserves_delay_history_and_ideal_remains_instantaneous():
    controller = VehicleController(steering_params=steering_profile("assumed"))
    path = [(0., 0., 0., 0.), (1000., 0., 0., 0.)]
    controller.update_ref_path(path)
    controller.step(20, 0.01, 0, 10, 0, 0, actual_steer=0)
    history = controller.lat.command_history[:]
    controller.update_ref_path(list(path), reset=False)
    assert controller.lat.command_history == history
    actuator = SteeringActuator(steering_profile("ideal"), 0.5)
    actuator.begin_period(0.3, 0.05)
    assert actuator.advance(0.025) == 0.3


def test_vehicle_limits_are_shared_and_nonfinite_commands_rejected():
    from lightweight_sim.engine.simulator.data_types import VehicleParams
    params = VehicleParams(max_steer=0.2, max_accel=2., max_decel=4.)
    controller = VehicleController(vehicle_params=params)
    assert controller.lat.max_steer == 0.2
    assert controller.lon.max_accel == 2
    assert controller.lon.max_decel == 4
    with pytest.raises(ValueError):
        VehicleParams(max_accel=0)
    engine = SimulationEngine(make_scenario("figure_eight"))
    for command in (ControlCommand(steer=float("nan")), ControlCommand(throttle=float("inf"))):
        with pytest.raises(ValueError):
            engine.step(command)
