"""Bicycle dynamics shared by lateral controllers, independent of solvers."""

import math
import numpy as np
from ...simulator.steering import SteeringParams


class BicycleLateralModel:
    def __init__(self, vehicle_para, ts=0.05, actuator_params=None):
        if len(vehicle_para) != 6:
            raise ValueError("vehicle_para must contain (a, b, m, Cf, Cr, Iz)")
        self.a, self.b, self.m, self.Cf, self.Cr, self.Iz = map(float, vehicle_para)
        if not all(math.isfinite(value) for value in vehicle_para):
            raise ValueError("vehicle parameters must be finite")
        if min(self.a, self.b, self.m, self.Iz) <= 0 or max(self.Cf, self.Cr) >= 0:
            raise ValueError("positive geometry/mass/inertia and negative stiffness required")
        self.ts = float(ts)
        if not math.isfinite(self.ts) or self.ts <= 0:
            raise ValueError("control period must be positive and finite")
        self.actuator_params = actuator_params or SteeringParams()

    def continuous(self, vx: float) -> tuple[np.ndarray, np.ndarray]:
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

    def bilinear(self, vx, kappa=0.0):
        """CARLA-style discretization, with corrected curvature disturbance."""
        A, B = self.continuous(vx)
        speed = max(abs(float(vx)), 0.5)
        C = np.zeros(4)
        C[1] = (self.a*self.Cf-self.b*self.Cr)/(self.m*speed)-speed
        C[3] = (self.a**2*self.Cf+self.b**2*self.Cr)/(self.Iz*speed)
        left = np.eye(4) - self.ts*A/2
        return (np.linalg.solve(left, np.eye(4)+self.ts*A/2),
                np.linalg.solve(left, self.ts*B),
                np.linalg.solve(left, self.ts*C*speed*float(kappa)))

    def plant(self, vx, actuator=False, kappa=0.0, max_substep_s=None):
        """Linearize the simulator's held-input substeps at constant speed.

        z=[y, vy, phi, r]; e=[y, vy+v*phi, phi, r]. The simulator uses
        explicit Euler for vy/r/position and the new r to advance heading.
        Accumulate these substeps before transforming to error coordinates.
        """
        v = max(abs(float(vx)), 0.5)
        n = max(1, math.ceil(v*self.ts/0.5))
        if max_substep_s is not None:
            if not math.isfinite(max_substep_s) or max_substep_s <= 0:
                raise ValueError("model max_substep_s must be positive and finite")
            n = max(n, math.ceil(self.ts/max_substep_s))
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
        # Reference yaw rate is held over the prediction horizon. Transform
        # the physical substep into moving-reference error coordinates.
        reference = np.zeros(dimension)
        reference[3] = v * float(kappa)
        drift = F @ reference - reference
        drift[2] -= v * float(kappa) * h
        Ad, Bd = np.eye(dimension), np.zeros((dimension, 1))
        Cd = np.zeros(dimension)
        for _ in range(n):
            Cd = F @ Cd + drift
            Ad, Bd = F@Ad, F@Bd+G
        T = np.eye(dimension)
        T[1, 2] = v
        Ad, Bd, Cd = T@Ad@np.linalg.inv(T), T@Bd, T@Cd
        delay = self.actuator_params.delay_steps(self.ts) if actuator else 0
        if delay:
            delayed_A = np.zeros((dimension+delay, dimension+delay))
            delayed_B = np.zeros((dimension+delay, 1))
            delayed_A[:dimension, :dimension] = Ad
            delayed_A[:dimension, dimension] = Bd[:, 0]
            for i in range(delay-1):
                delayed_A[dimension+i, dimension+i+1] = 1.0
            delayed_B[-1, 0] = 1.0
            return delayed_A, delayed_B, np.concatenate((Cd, np.zeros(delay)))
        return Ad, Bd, Cd
