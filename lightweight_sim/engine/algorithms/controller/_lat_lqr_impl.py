"""Dynamic lateral LQR core with an explicit discrete Riccati solver."""

import math

import numpy as np
from ...simulator.steering import SteeringParams


class LateralLQRController:
    """Solve a speed-dependent bicycle-model DARE and apply ``-Kx`` feedback.

    ``vehicle_para`` follows the lightweight simulator convention:
    ``(a, b, m, Cf, Cr, Iz)``. Cornering stiffness values are expected to be
    negative, matching the sign convention used by :class:`EgoVehicle`.
    """

    def __init__(self, vehicle_para, Q=None, R=100.0, ts=0.05):
        if len(vehicle_para) != 6:
            raise ValueError("vehicle_para must contain (a, b, m, Cf, Cr, Iz)")
        self.a, self.b, self.m, self.Cf, self.Cr, self.Iz = map(float, vehicle_para)
        if self.m <= 0.0 or self.Iz <= 0.0 or self.a + self.b <= 0.0:
            raise ValueError("vehicle geometry, mass, and inertia must be positive")

        self.ts = float(ts)
        self.min_index = 0
        self.max_steer = 0.5
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
        self.discretization = "plant"
        self.last_feedforward = 0.0
        self.last_feedback = 0.0
        self.last_unclipped_steer = 0.0
        self.actuator_params = SteeringParams()
        self.actual_steer = 0.0
        self.command_history = []

    def configure_actuator(self, params):
        self.actuator_params = params
        self.command_history = [0.0] * params.delay_steps(self.ts)

    def reset_actuator_history(self):
        self.command_history = [0.0] * self.actuator_params.delay_steps(self.ts)
        self.actual_steer = 0.0

    def _continuous_model(self, vx: float) -> tuple[np.ndarray, np.ndarray]:
        """Build the linearized lateral bicycle model at the current speed."""
        speed = max(abs(float(vx)), 0.5)
        a, b, m, cf, cr, iz = self.a, self.b, self.m, self.Cf, self.Cr, self.Iz
        A = np.zeros((4, 4), dtype=float)
        B = np.zeros((4, 1), dtype=float)

        # Error state: [e_d, e_d_dot, e_phi, e_phi_dot].
        A[0, 1] = 1.0
        A[1, 1] = (cf + cr) / (m * speed)
        A[1, 2] = -(cf + cr) / m
        A[1, 3] = (a * cf - b * cr) / (m * speed)
        A[2, 3] = 1.0
        A[3, 1] = (a * cf - b * cr) / (iz * speed)
        A[3, 2] = -(a * cf - b * cr) / iz
        A[3, 3] = (a * a * cf + b * b * cr) / (iz * speed)
        B[1, 0] = -cf / m
        B[3, 0] = -a * cf / iz
        return A, B

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
        """Linearize the simulator's held-input substeps at constant speed.

        z=[y, vy, phi, r]; e=[y, vy+v*phi, phi, r]. The simulator uses
        explicit Euler for vy/r/position and the new r to advance heading.
        Accumulate these substeps before transforming to error coordinates.
        """
        v = max(abs(float(vx)), 0.5)
        n = max(1, math.ceil(v*self.ts/0.5))
        h = self.ts/n
        a, b, m, cf, cr, iz = self.a, self.b, self.m, self.Cf, self.Cr, self.Iz
        m11 = (cf+cr)/(m*v)
        m12 = (a*cf-b*cr)/(m*v)-v
        m21 = (a*cf-b*cr)/(iz*v)
        m22 = (a*a*cf+b*b*cr)/(iz*v)
        steer_vy, steer_r = -cf/m, -a*cf/iz
        F = np.eye(4)
        G = np.zeros((4, 1))
        F[0, 1], F[0, 2] = h, h*v
        F[1, 1], F[1, 3] = 1+h*m11, h*m12
        F[3, 1], F[3, 3] = h*m21, 1+h*m22
        F[2, 1], F[2, 3] = h*h*m21, h*(1+h*m22)
        G[1, 0], G[3, 0], G[2, 0] = h*steer_vy, h*steer_r, h*h*steer_r
        dimension = 4
        if actuator:
            # Updated actuator angle drives the vehicle in each substep.
            alpha = math.exp(-h/self.actuator_params.time_constant_s)
            augmented = np.eye(5)
            augmented[:4, :4] = F
            augmented[:4, 4] = G[:, 0]*alpha
            augmented[4, 4] = alpha
            input_matrix = np.zeros((5, 1))
            input_matrix[:4, 0] = G[:, 0]*(1-alpha)
            input_matrix[4, 0] = 1-alpha
            F, G, dimension = augmented, input_matrix, 5
        Ad, Bd = np.eye(dimension), np.zeros((dimension, 1))
        for _ in range(n):
            Ad, Bd = F@Ad, F@Bd+G
        T = np.eye(dimension)
        T[1, 2] = v
        Ad, Bd = T@Ad@np.linalg.inv(T), T@Bd
        delay = self.actuator_params.delay_steps(self.ts) if actuator else 0
        if delay:
            delayed_A = np.zeros((dimension+delay, dimension+delay))
            delayed_B = np.zeros((dimension+delay, 1))
            delayed_A[:dimension, :dimension] = Ad
            delayed_A[:dimension, dimension] = Bd[:, 0]
            for i in range(delay-1):
                delayed_A[dimension+i, dimension+i+1] = 1.0
            delayed_B[-1, 0] = 1.0
            return delayed_A, delayed_B
        return Ad, Bd

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

    def control(self, x, y, phi, vx, vy, r, ref_path):
        """Fallback point-based control for callers without projected matching."""
        if not ref_path:
            return 0.0
        horizon = max(0.0, self.ts)
        px = x + (vx * math.cos(phi) - vy * math.sin(phi)) * horizon
        py = y + (vx * math.sin(phi) + vy * math.cos(phi)) * horizon
        self.x_pre, self.y_pre = px, py
        start = min(max(0, self.min_index), len(ref_path) - 1)
        index = min(
            range(start, len(ref_path)),
            key=lambda i: (ref_path[i][0] - px) ** 2 + (ref_path[i][1] - py) ** 2,
        )
        self.min_index = index
        rx, ry, theta, kappa = ref_path[index]
        nx, ny = -math.sin(theta), math.cos(theta)
        ed = nx * (px - rx) + ny * (py - ry)
        ephi = math.atan2(math.sin(phi - theta), math.cos(phi - theta))
        ed_dot = vy * math.cos(ephi) + vx * math.sin(ephi)
        s_dot = vx * math.cos(ephi) - vy * math.sin(ephi)
        ephi_dot = r - kappa * s_dot
        self.x_pro, self.y_pro = rx, ry
        self.last_ed, self.last_ephi = ed, ephi
        return self.control_from_error((ed, ed_dot, math.sin(ephi), ephi_dot), kappa, vx)
