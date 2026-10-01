"""Projected lateral LQR controller with an explicit discrete Riccati solver."""

import math

import numpy as np
from .reference_tracking import ProjectedLateralController
from .lateral_model import BicycleLateralModel


class LateralLQRController(ProjectedLateralController):
    """Solve a speed-dependent bicycle-model DARE and apply ``-Kx`` feedback.

    ``vehicle_para`` follows the lightweight simulator convention:
    ``(a, b, m, Cf, Cr, Iz)``. Cornering stiffness values are expected to be
    negative, matching the sign convention used by :class:`EgoVehicle`.
    """

    def __init__(self, vehicle_para, Q=None, R=100.0, ts=0.05,
                 *, discretization="plant"):
        if len(vehicle_para) != 6:
            raise ValueError("vehicle_para must contain (a, b, m, Cf, Cr, Iz)")
        self.a, self.b, self.m, self.Cf, self.Cr, self.Iz = map(float, vehicle_para)
        if self.m <= 0.0 or self.Iz <= 0.0 or self.a + self.b <= 0.0:
            raise ValueError("vehicle geometry, mass, and inertia must be positive")

        super().__init__(ts=ts)
        self.Q = np.array(
            Q if Q is not None else np.diag([200.0, 1.0, 50.0, 1.0]),
            dtype=float,
        )
        self.R = np.array([[float(R)]], dtype=float)
        if self.Q.shape != (4, 4) or not np.allclose(self.Q, self.Q.T):
            raise ValueError("Q must be a symmetric 4x4 matrix")
        if np.any(np.linalg.eigvalsh(self.Q) < 0.0) or self.R[0, 0] <= 0.0:
            raise ValueError("Q must be positive semidefinite and R positive")

        self.A = np.zeros((4, 4), dtype=float)
        self.B = np.zeros((4, 1), dtype=float)
        self.P = self.Q.copy()
        self.K = np.zeros((1, 4), dtype=float)
        self.riccati_iterations = 0
        self.riccati_converged = False
        self.x_pre = 0.0
        self.y_pre = 0.0
        self.x_pro = 0.0
        self.y_pro = 0.0
        self.last_ed = 0.0
        self.last_ephi = 0.0
        self.last_error_state = np.zeros(4, dtype=float)
        self.discretization = str(discretization)
        self.last_feedforward = 0.0
        self.last_feedback = 0.0
        self.last_unclipped_steer = 0.0

    def _model(self):
        return BicycleLateralModel(
            (self.a, self.b, self.m, self.Cf, self.Cr, self.Iz),
            self.ts, self.actuator_params)

    def _continuous_model(self, vx):
        return self._model().continuous(vx)

    def _discretize(self, A: np.ndarray, B: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Discretize with the same bilinear transform as the CARLA controller."""
        identity = np.eye(4)
        left = identity - 0.5 * self.ts * A
        right = identity + 0.5 * self.ts * A
        A_d = np.linalg.solve(left, right)
        B_d = np.linalg.solve(left, self.ts * B)
        return A_d, B_d

    def _solve_dare(self, A_d: np.ndarray, B_d: np.ndarray) -> np.ndarray:
        """Iterate the discrete algebraic Riccati equation to convergence."""
        cost = np.zeros_like(A_d)
        cost[:4, :4] = self.Q
        P = cost.copy()
        self.riccati_converged = False
        max_iterations = 500
        tolerance = 1e-8
        for iteration in range(1, max_iterations + 1):
            control_cost = self.R + B_d.T @ P @ B_d
            feedback_term = A_d.T @ P @ B_d
            next_P = (
                A_d.T @ P @ A_d
                - feedback_term @ np.linalg.solve(control_cost, feedback_term.T)
                + cost
            )
            next_P = 0.5 * (next_P + next_P.T)
            if np.max(np.abs(next_P - P)) < tolerance:
                P = next_P
                self.riccati_iterations = iteration
                self.riccati_converged = True
                break
            P = next_P
        else:
            self.riccati_iterations = max_iterations
        return P

    def update_lqr_gain(self, vx: float) -> np.ndarray:
        """Recompute ``A_d``, ``B_d``, ``P`` and ``K`` for the current speed."""
        A_c, B_c = self._continuous_model(vx)
        if self.actuator_params.mode == "dynamic":
            if self.discretization != "plant":
                raise ValueError("augmented actuator LQR requires plant discretization")
            A_d, B_d = self._plant_discretize(vx, actuator=True)
        elif self.discretization == "plant":
            A_d, B_d = self._plant_discretize(vx)
        elif self.discretization == "bilinear":
            A_d, B_d = self._discretize(A_c, B_c)
        else:
            raise ValueError("unknown LQR discretization")
        P = self._solve_dare(A_d, B_d)
        self.A, self.B, self.P = A_d, B_d, P
        self.K = np.linalg.solve(
            self.R + B_d.T @ P @ B_d,
            B_d.T @ P @ A_d,
        )
        return self.K

    def _plant_discretize(self, vx, actuator=False):
        A, B, _ = self._model().plant(vx, actuator=actuator)
        return A, B

    def control_from_error(self, error_state, kappa: float, vx: float) -> float:
        """Return curvature feedforward plus Riccati state feedback."""
        error = np.asarray(error_state, dtype=float).reshape(4)
        self.last_error_state = error.copy()
        self.update_lqr_gain(vx)

        speed = max(abs(float(vx)), 0.5)
        wheelbase = self.a + self.b
        understeer = self.m * speed * speed / wheelbase * (
            self.b / self.Cf - self.a / self.Cr
        )
        delta_ff = math.atan(wheelbase * float(kappa) + understeer * float(kappa))
        if self.actuator_params.mode == "dynamic":
            error = np.concatenate((error, [self.actual_steer-delta_ff],
                                    np.asarray(self.command_history)-delta_ff))
        delta = delta_ff - float((self.K @ error)[0])
        self.last_feedforward = delta_ff
        self.last_feedback = -float((self.K @ error)[0])
        self.last_unclipped_steer = delta
        command = max(-self.max_steer, min(self.max_steer, delta))
        if self.command_history:
            self.command_history = self.command_history[1:] + [command]
        return command
