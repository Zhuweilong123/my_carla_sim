"""Bounded geometric lateral controller with the LQR-compatible interface."""

import math


class LateralLQRController:
    def __init__(self, vehicle_para, Q=None, R=1.0, ts=0.05):
        del Q, R
        self.a, self.b, self.m, self.Cf, self.Cr, self.Iz = vehicle_para
        self.ts = ts
        self.min_index = 0
        self.max_steer = 0.5
        self.k_lat = 2.0
        self.k_heading = 0.6
        self.x_pre = 0.0
        self.y_pre = 0.0
        self.x_pro = 0.0
        self.y_pro = 0.0
        self.last_ed = 0.0
        self.last_ephi = 0.0

    def control(self, x, y, phi, vx, vy, r, ref_path):
        del r
        if not ref_path:
            return 0.0

        horizon = max(0.0, self.ts)
        px = x + (vx * math.cos(phi) - vy * math.sin(phi)) * horizon
        py = y + (vx * math.sin(phi) + vy * math.cos(phi)) * horizon
        self.x_pre, self.y_pre = px, py

        start = min(max(0, self.min_index), len(ref_path) - 1)
        best = min(
            range(start, len(ref_path)),
            key=lambda i: (ref_path[i][0] - px) ** 2 + (ref_path[i][1] - py) ** 2,
        )
        self.min_index = best

        rx, ry, theta, kappa = ref_path[best]
        nx, ny = -math.sin(theta), math.cos(theta)
        ed = nx * (px - rx) + ny * (py - ry)
        ephi = math.atan2(math.sin(phi - theta), math.cos(phi - theta))
        self.x_pro, self.y_pro = rx, ry
        self.last_ed = ed
        self.last_ephi = ephi

        speed = max(abs(vx), 1.0)
        # ed is positive along the reference normal. The feedback must steer
        # toward the reference line, hence the negative lateral-error term.
        curvature_cmd = kappa - self.k_heading * ephi - self.k_lat * ed / (speed * speed)
        delta = math.atan((self.a + self.b) * curvature_cmd)
        return max(-self.max_steer, min(self.max_steer, delta))