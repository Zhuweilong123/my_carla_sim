"""Regression tests against the actual simulator transition, not only DARE."""
import math

import numpy as np
import pytest

from lightweight_sim.engine.algorithms.controller.lat_lqr import LateralLQRController
from lightweight_sim.engine.algorithms.utils.route import RouteGeometry
from lightweight_sim.engine.simulator.data_types import VehicleState
from lightweight_sim.engine.simulator.vehicle import EgoVehicle

PARAMS = (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0)


def transition(error, steer, speed, dt):
    # e=[y, vy+v*phi, phi, r] on a straight reference.
    vehicle = EgoVehicle(VehicleState(vx=speed, y=error[0],
                          vy=error[1]-speed*error[2], phi=error[2], r=error[3]))
    substeps = max(1, math.ceil(speed*dt/0.5))
    for _ in range(substeps):
        state = vehicle.step(steer, 0., dt/substeps, model="dynamic")
    return np.array([state.y, state.vy+speed*state.phi, state.phi, state.r])


@pytest.mark.parametrize("speed", [6., 30/3.6, 50/3.6, 60/3.6])
@pytest.mark.parametrize("dt", [0.02, 0.05, 0.1])
def test_dare_plant_model_matches_actual_held_input_transition(speed, dt):
    controller = LateralLQRController(PARAMS, ts=dt)
    controller.update_lqr_gain(speed)
    eps = 1e-6
    numerical_A = np.column_stack([
        (transition(np.eye(4)[i]*eps, 0, speed, dt)
         - transition(-np.eye(4)[i]*eps, 0, speed, dt))/(2*eps)
        for i in range(4)])
    numerical_B = (transition(np.zeros(4), eps, speed, dt)
                   - transition(np.zeros(4), -eps, speed, dt))/(2*eps)
    assert np.allclose(controller.A, numerical_A, atol=1e-7)
    assert np.allclose(controller.B[:, 0], numerical_B, atol=1e-7)
    assert max(abs(np.linalg.eigvals(numerical_A-controller.B@controller.K))) < 1


def test_full_feedback_damps_small_perturbation_without_saturation():
    controller = LateralLQRController(PARAMS)
    state = VehicleState(x=20, y=0.05, vx=50/3.6, phi=0.01)
    vehicle = EgoVehicle(state)
    path = [(0., 0., 0., 0.), (10000., 0., 0., 0.)]
    steering = []
    for _ in range(100):
        state = vehicle.get_state()
        steer = controller.control(state.x, state.y, state.phi, state.vx, state.vy, state.r, path)
        steering.append(steer)
        n = math.ceil(state.speed*0.05/0.5)
        for _ in range(n):
            vehicle.step(steer, 0, 0.05/n, "dynamic")
    assert max(abs(s) for s in steering) < 0.2
    assert abs(vehicle.get_state().y) < 1e-4
    assert abs(vehicle.get_state().r) < 1e-4


def test_reference_tangent_interpolation_reduces_vertex_command_jump():
    path = [(0., 0., 0., 0.), (10., 0., 0.1, 0.02),
            (20., 2., 0.2, 0.02)]
    # The same measurement geometry remains unchanged by control smoothing.
    geometry = RouteGeometry(path)
    commands = []
    for x in (9.999, 10.001):
        ctrl = LateralLQRController(PARAMS)
        commands.append(ctrl.control(x, 0., 0.1, 10., 0., 0., path))
    assert abs(commands[1]-commands[0]) < 0.01
    assert geometry.path == tuple(path)
