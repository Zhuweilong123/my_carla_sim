"""Lateral controller with continuous reference-line projection."""

import math

from ._lat_lqr_impl import *  # noqa: F401,F403


_LegacyLateralLQRController = LateralLQRController


class LateralLQRController(_LegacyLateralLQRController):
    """Track a projected reference point instead of a sampled path point."""

    def __init__(self, vehicle_para, Q=None, R=1.0, ts=0.1):
        super().__init__(vehicle_para, Q=Q, R=R, ts=ts)
        self.k_lat = 12.0
        self.k_heading = 0.8
        self.heading_match_weight = 2.0

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def control(self, x, y, phi, vx, vy, r, ref_path):
        del r
        if len(ref_path) < 2:
            return 0.0

        horizon = max(0.0, self.ts)
        px = x + (vx * math.cos(phi) - vy * math.sin(phi)) * horizon
        py = y + (vx * math.sin(phi) + vy * math.cos(phi)) * horizon
        self.x_pre, self.y_pre = px, py

        start = min(max(0, self.min_index), len(ref_path) - 2)
        best = None
        for index in range(start, len(ref_path) - 1):
            ax, ay = ref_path[index][0], ref_path[index][1]
            bx, by = ref_path[index + 1][0], ref_path[index + 1][1]
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                continue
            ratio = ((px - ax) * dx + (py - ay) * dy) / length_sq
            ratio = max(0.0, min(1.0, ratio))
            rx, ry = ax + ratio * dx, ay + ratio * dy
            distance_sq = (px - rx) ** 2 + (py - ry) ** 2
            theta = math.atan2(dy, dx)
            ephi = self._wrap_angle(phi - theta)
            score = distance_sq + self.heading_match_weight * ephi * ephi
            if best is None or score < best[0]:
                kappa = (1.0 - ratio) * ref_path[index][3] + ratio * ref_path[index + 1][3]
                best = (score, index, rx, ry, theta, kappa, distance_sq)

        if best is None:
            return 0.0

        _, index, rx, ry, theta, kappa, _ = best
        self.min_index = index
        self.x_pro, self.y_pro = rx, ry
        nx, ny = -math.sin(theta), math.cos(theta)
        ed = nx * (px - rx) + ny * (py - ry)
        ephi = self._wrap_angle(phi - theta)
        self.last_ed = ed
        self.last_ephi = ephi

        speed = max(abs(vx), 1.0)
        curvature_cmd = (
            kappa
            - self.k_heading * ephi
            - self.k_lat * ed / (speed * speed)
        )
        delta = math.atan((self.a + self.b) * curvature_cmd)
        return max(-self.max_steer, min(self.max_steer, delta))
