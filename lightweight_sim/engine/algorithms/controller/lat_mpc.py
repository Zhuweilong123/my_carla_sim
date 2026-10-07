"""Finite-horizon lateral MPC migrated from carla_legacy/controller/Controller.py.

Retains the condensed prediction/cost/QP approach, with scalar steering
moves, a true control horizon, physical angle bounds and no CARLA dependency.
"""

import math
import numpy as np

from .box_qp import solve_box_qp
from .lateral_model import BicycleLateralModel
from .reference_tracking import ProjectedLateralController
from ...runtime_config import DEFAULT_RUNTIME_CONFIG


def _state_weight(value, default, name):
    weight = np.asarray(default if value is None else value, dtype=float)
    if weight.shape == (4,):
        weight = np.diag(weight)
    if (weight.shape != (4, 4) or not np.all(np.isfinite(weight))
            or not np.allclose(weight, weight.T)
            or np.min(np.linalg.eigvalsh(weight)) < -1e-10):
        raise ValueError(name + ' must be a finite positive semidefinite 4x4 matrix')
    return weight.copy()


class LateralMPCController(ProjectedLateralController):
    """Optimize N predicted steps using P scalar steering moves.

    The last move is held for steps P..N-1. Curvature and speed are held
    constant across a solve. F weights the terminal tracking error; dynamic
    steering augments the model with measured angle and the delay FIFO.
    """

    def __init__(self, vehicle_para, Q=None, F=None, R=1.0, N=6, P=2,
                 ts=0.05, *, discretization='plant', solver='auto',
                 max_substep_s=DEFAULT_RUNTIME_CONFIG.dynamic_max_substep_s):
        super().__init__(ts=ts)
        self.vehicle_para = tuple(vehicle_para)
        BicycleLateralModel(self.vehicle_para, ts)  # Validate units/parameters.
        if (isinstance(N, bool) or isinstance(P, bool) or not isinstance(N, (int, np.integer))
                or not isinstance(P, (int, np.integer)) or not 1 <= P <= N):
            raise ValueError('MPC horizons must be integers with 1 <= P <= N')
        if not math.isfinite(max_substep_s) or max_substep_s <= 0:
            raise ValueError('MPC max_substep_s must be positive and finite')
        self.max_substep_s = float(max_substep_s)
        self.N, self.P = int(N), int(P)
        self.Q = _state_weight(Q, np.diag([200., 1., 50., 1.]), 'Q')
        self.F = _state_weight(F, self.Q, 'F')
        if not math.isfinite(float(R)) or float(R) <= 0:
            raise ValueError('MPC R must be positive and finite')
        self.R = np.array([[float(R)]])
        if discretization not in ('plant', 'bilinear'):
            raise ValueError('unknown MPC discretization')
        if solver not in ('auto', 'numpy', 'cvxopt'):
            raise ValueError('unknown MPC QP backend')
        self.discretization, self.solver = discretization, solver
        self.last_error_state = np.zeros(4)
        self.last_feedforward = self.last_feedback = self.last_unclipped_steer = 0.0
        self.last_solution = None
        self.predicted_states = None
        self.solver_status = 'not_run'
        self.solver_backend = None

    def reset_tracking(self):
        super().reset_tracking()
        self.last_solution = self.predicted_states = None
        self.solver_status = 'not_run'
        self.solver_backend = None

    def prediction_matrices(self, A, B, drift):
        """Stack x0..xN = M*x0 + C*moves + offset (CARLA condensed form)."""
        size = A.shape[0]
        M = np.zeros(((self.N+1)*size, size))
        C = np.zeros(((self.N+1)*size, self.P))
        offset = np.zeros((self.N+1)*size)
        M[:size] = np.eye(size)
        for k in range(self.N):
            previous = slice(k*size, (k+1)*size)
            current = slice((k+1)*size, (k+2)*size)
            M[current] = A @ M[previous]
            C[current] = A @ C[previous]
            C[current, min(k, self.P-1)] += B[:, 0]
            offset[current] = A @ offset[previous] + drift
        return M, C, offset

    def control_from_error(self, error_state, kappa, vx):
        error = np.asarray(error_state, dtype=float).reshape(4)
        if not np.all(np.isfinite(error)) or not math.isfinite(kappa) or not math.isfinite(vx):
            raise ValueError('MPC inputs must be finite')
        self.last_error_state = error.copy()
        model = BicycleLateralModel(self.vehicle_para, self.ts, self.actuator_params)
        dynamic = self.actuator_params.mode == 'dynamic'
        if self.discretization == 'bilinear':
            if dynamic:
                raise ValueError('actuator-aware MPC requires plant discretization')
            A, B, drift = model.bilinear(vx, kappa)
        else:
            A, B, drift = model.plant(vx, actuator=dynamic, kappa=kappa,
                                    max_substep_s=self.max_substep_s)
        if dynamic:
            delay = self.actuator_params.delay_steps(self.ts)
            if (len(self.command_history) != delay or not math.isfinite(self.actual_steer)
                    or not np.all(np.isfinite(self.command_history))):
                raise ValueError('MPC actuator feedback/history is invalid; reset required')
            error = np.concatenate((error, [self.actual_steer], self.command_history))
        self.A, self.B, self.curvature_drift = A, B, drift
        size = A.shape[0]
        M, C, offset = self.prediction_matrices(A, B, drift)
        weights = np.zeros(((self.N+1)*size, (self.N+1)*size))
        for k in range(self.N+1):
            start = k*size
            weights[start:start+4, start:start+4] = self.F if k == self.N else self.Q
        # Penalize every applied steering input, including held tail moves.
        counts = np.bincount(np.minimum(np.arange(self.N), self.P-1), minlength=self.P)
        R_bar = np.diag(counts*float(self.R[0, 0]))
        free_prediction = M @ error + offset
        H = 2*(C.T @ weights @ C + R_bar)
        H = (H+H.T)/2
        f = 2*C.T @ weights @ free_prediction
        self.H, self.f = H, f
        self.last_solution = self.predicted_states = None
        self.solver_status = 'solving'
        try:
            solution, backend = solve_box_qp(H, f, self.max_steer, backend=self.solver)
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as exc:
            self.solver_status = 'failed'
            raise RuntimeError('lateral MPC solve failed') from exc
        self.solver_status, self.solver_backend = 'optimal', backend
        self.last_solution = solution
        self.predicted_states = (free_prediction+C @ solution).reshape(self.N+1, size)
        command = float(solution[0])
        # Curvature compensation is included in the affine prediction, rather
        # than added as a separate steering feedforward after optimization.
        self.last_feedforward = 0.0
        self.last_feedback = command
        self.last_unclipped_steer = float(np.linalg.solve(H, -f)[0])
        if self.command_history:
            self.command_history = self.command_history[1:]+[command]
        return command
