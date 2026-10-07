"""Numerical and closed-loop checks for the migrated CARLA MPC."""
import itertools
import math

import numpy as np
import pytest

from lightweight_sim.engine.algorithms.controller.box_qp import solve_box_qp
from lightweight_sim.engine.algorithms.controller.combined import VehicleController
from lightweight_sim.engine.algorithms.controller.lat_mpc import LateralMPCController
from lightweight_sim.engine.algorithms.controller.lateral_model import BicycleLateralModel
from lightweight_sim.engine.simulator.data_types import VehicleState
from lightweight_sim.engine.simulator.steering import SteeringActuator, SteeringParams
from lightweight_sim.engine.simulator.vehicle import EgoVehicle

PARAMS = (1.015, 1.895, 1412., -148970., -82204., 1537.)


def enumerate_box_optimum(H, f, bound):
    # Independent small-problem oracle: check all bound/free combinations.
    candidates = []
    for status in itertools.product((-1, 0, 1), repeat=len(f)):
        fixed = np.array(status) != 0
        free = ~fixed
        u = np.array(status, dtype=float)*bound
        if np.any(free):
            u[free] = np.linalg.solve(H[np.ix_(free, free)],
                                     -f[free]-H[np.ix_(free, fixed)] @ u[fixed])
        if np.all(abs(u) <= bound+1e-9):
            candidates.append((0.5*u @ H @ u+f @ u, u))
    return min(candidates, key=lambda candidate: candidate[0])[1]


def test_numpy_qp_matches_independent_active_bound_oracle():
    rng = np.random.default_rng(42)
    for size in (1, 2, 4):
        for _ in range(15):
            data = rng.normal(size=(size, size))
            H = data.T @ data+np.eye(size)*0.01
            f = rng.normal(size=size)*3
            actual, backend = solve_box_qp(H, f, 0.3, backend='numpy')
            assert backend == 'numpy'
            assert actual == pytest.approx(enumerate_box_optimum(H, f, 0.3), abs=1e-8)


def test_optional_cvxopt_matches_numpy():
    pytest.importorskip('cvxopt')
    H = np.array([[4., 1.], [1., 2.]])
    f = np.array([-4., 1.])
    expected, _ = solve_box_qp(H, f, 0.3, backend='numpy')
    actual, backend = solve_box_qp(H, f, 0.3, backend='cvxopt')
    assert backend == 'cvxopt'
    assert actual == pytest.approx(expected, abs=1e-6)


def test_prediction_matches_rollout_with_last_move_held():
    controller = LateralMPCController(PARAMS, N=6, P=2)
    A = np.array([[1., 0.1], [0., 0.8]])
    B = np.array([[0.01], [0.2]])
    drift = np.array([0.02, -0.03])
    initial, moves = np.array([0.3, -0.1]), np.array([0.2, -0.3])
    M, C, offset = controller.prediction_matrices(A, B, drift)
    state = initial.copy()
    expected = [state.copy()]
    for k in range(6):
        state = A @ state+B[:, 0]*moves[min(k, 1)]+drift
        expected.append(state.copy())
    assert C.shape == (14, 2)
    assert (M @ initial+C @ moves+offset).reshape(7, 2) == pytest.approx(np.asarray(expected))


@pytest.mark.parametrize('discretization', ['plant', 'bilinear'])
def test_cost_matches_direct_horizon_rollout(discretization):
    controller = LateralMPCController(PARAMS, N=6, P=2, R=3,
                                     F=[10, 2, 20, 1], solver='numpy',
                                     discretization=discretization)
    initial = np.array([0.2, 0.05, 0.01, 0.02])
    controller.control_from_error(initial, 0.01, 10)
    def direct_cost(moves):
        state, cost = initial.copy(), 0.0
        for k in range(controller.N):
            cost += state @ controller.Q @ state
            u = moves[min(k, controller.P-1)]
            cost += 3*u*u
            state = controller.A @ state+controller.B[:, 0]*u+controller.curvature_drift
        return cost+state @ controller.F @ state
    optimum = controller.last_solution
    assert optimum == pytest.approx(enumerate_box_optimum(controller.H, controller.f, 0.5))
    for moves in (np.array([0.1, -0.05]), np.array([-0.2, 0.3])):
        # The condensed objective omits only a control-independent constant.
        difference = 0.5*moves @ controller.H @ moves+controller.f @ moves
        assert difference == pytest.approx(direct_cost(moves)-direct_cost(np.zeros(2)))
    assert direct_cost(optimum) < direct_cost(np.zeros(2))


def test_curvature_disturbance_matches_physical_substep():
    speed, curvature, dt = 10., 0.01, 0.05
    model = BicycleLateralModel(PARAMS, dt)
    A, B, drift = model.plant(speed, kappa=curvature)
    vehicle = EgoVehicle(VehicleState(vx=speed, r=speed*curvature))
    state = vehicle.step(0, 0, dt, 'dynamic')
    heading_error = state.phi-speed*curvature*dt
    measured = [state.y, state.vy+speed*heading_error,
                heading_error, state.r-speed*curvature]
    assert drift == pytest.approx(measured, abs=1e-10)
    continuous_A, _ = model.continuous(speed)
    _, _, bilinear_drift = model.bilinear(speed, curvature)
    recovered = (np.eye(4)-dt*continuous_A/2) @ bilinear_drift/dt
    expected = np.zeros(4)
    expected[1] = ((PARAMS[0]*PARAMS[3]-PARAMS[1]*PARAMS[4])/(PARAMS[2]*speed)-speed)*speed*curvature
    expected[3] = (PARAMS[0]**2*PARAMS[3]+PARAMS[1]**2*PARAMS[4])/(PARAMS[5]*speed)*speed*curvature
    assert recovered == pytest.approx(expected)



def test_mpc_constraints_curvature_and_actuator_fifo():
    controller = LateralMPCController(PARAMS, solver='numpy')
    controller.max_steer = 0.2
    steer = controller.control_from_error([10, 0, 0, 0], 0, 10)
    assert steer == pytest.approx(-0.2)
    assert np.all(abs(controller.last_solution) <= 0.2)
    assert controller.control_from_error([0, 0, 0, 0], 0.02, 10) > 0
    controller.configure_actuator(SteeringParams(mode='dynamic', delay_s=0.1))
    controller.actual_steer = 0.03
    controller.command_history = [0.01, 0.02]
    steer = controller.control_from_error([0.1, 0, 0, 0], 0.01, 10)
    assert controller.predicted_states.shape == (7, 7)
    assert controller.predicted_states[0, 4:] == pytest.approx([0.03, 0.01, 0.02])
    assert controller.predicted_states[1, -2:] == pytest.approx([0.02, steer])
    assert controller.command_history == pytest.approx([0.02, steer])
    controller.reset_tracking()
    assert controller.command_history == [0., 0.]
    assert controller.last_solution is None


@pytest.mark.parametrize('kwargs', [{'N': 0}, {'P': 7}, {'P': 1.5}, {'R': 0},
                                    {'Q': [1, 2, 3]}, {'F': [1, -1, 1, 1]},
                                    {'solver': 'bad'}, {'ts': 0}])
def test_invalid_mpc_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        LateralMPCController(PARAMS, **kwargs)


def test_qp_failure_does_not_publish_or_advance_history(monkeypatch):
    import lightweight_sim.engine.algorithms.controller.lat_mpc as module
    controller = LateralMPCController(PARAMS)
    controller.configure_actuator(SteeringParams(mode='dynamic'))
    before = controller.command_history.copy()
    def fail(*args, **kwargs):
        raise RuntimeError('injected solver failure')
    monkeypatch.setattr(module, 'solve_box_qp', fail)
    with pytest.raises(RuntimeError, match='MPC solve failed'):
        controller.control_from_error([0.1, 0, 0, 0], 0, 10)
    assert controller.solver_status == 'failed'
    assert controller.last_solution is None
    assert controller.command_history == before


@pytest.mark.parametrize('dynamic_actuator', [False, True])
def test_closed_loop_straight_and_circle_tracking(dynamic_actuator):
    steering = SteeringParams(mode='dynamic' if dynamic_actuator else 'ideal')
    for curved in (False, True):
        radius = 60.
        path = ([(radius*math.sin(t), radius*(1-math.cos(t)), t, 1/radius)
                 for t in np.linspace(0, 2*math.pi, 501)] if curved else
                [(float(i), 0., 0., 0.) for i in range(200)])
        controller = VehicleController(PARAMS, controller_type='MPC_controller',
                                       target_speed_kmh=36, steering_params=steering,
                                       mpc_params={'solver': 'numpy'})
        controller.update_ref_path(path)
        vehicle = EgoVehicle(VehicleState(y=0.5, phi=0.02, vx=10))
        actuator = SteeringActuator(steering, 0.5)
        errors = []
        for _ in range(160):
            state = vehicle.get_state()
            steer, _, _ = controller.step(state.x, state.y, state.phi, state.vx,
                                          state.vy, state.r, actual_steer=state.steer)
            actuator.begin_period(steer, 0.05)
            for _ in range(20):
                state = vehicle.step(actuator.advance(0.0025), -state.r*state.vy,
                                     0.0025, 'dynamic')
            error = abs(math.hypot(state.x, state.y-radius)-radius) if curved else abs(state.y)
            errors.append(error)
        assert max(errors) < 1.0
        assert max(errors[-40:]) < 0.15
        assert controller.lat.solver_status == 'optimal'

@pytest.mark.parametrize('speed', [0., 0.1, 0.5, 1., 3., 20.])
def test_low_speed_model_remains_finite_and_corrects_straight_line_error(speed):
    controller = LateralMPCController(PARAMS, solver='numpy')
    steer = controller.control_from_error([0.1, 0, 0, 0], 0, speed)
    assert np.isfinite(steer) and -0.5 <= steer < 0
    assert np.all(np.isfinite(controller.predicted_states))


def test_auto_backend_handles_missing_optional_dependency(monkeypatch):
    import builtins
    original = builtins.__import__
    def without_cvxopt(name, *args, **kwargs):
        if name == 'cvxopt':
            raise ImportError('not installed')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', without_cvxopt)
    result, backend = solve_box_qp(np.eye(2), np.array([2., -2.]), 0.3)
    assert backend == 'numpy'
    assert result == pytest.approx([-0.3, 0.3])
    with pytest.raises(RuntimeError, match='not installed'):
        solve_box_qp(np.eye(2), np.zeros(2), 0.3, backend='cvxopt')


@pytest.mark.parametrize('status, solution', [('unknown', [0., 0.]),
                                              ('optimal', [float('nan'), 0.]),
                                              ('optimal', [1., 0.])])
def test_cvxopt_failure_or_invalid_solution_is_rejected(monkeypatch, status, solution):
    import sys
    from types import SimpleNamespace
    fake = SimpleNamespace(matrix=lambda value: value,
                           solvers=SimpleNamespace(qp=lambda *args, **kwargs:
                                                   {'status': status, 'x': solution}))
    monkeypatch.setitem(sys.modules, 'cvxopt', fake)
    with pytest.raises(RuntimeError):
        solve_box_qp(np.eye(2), np.zeros(2), 0.3, backend='cvxopt')
