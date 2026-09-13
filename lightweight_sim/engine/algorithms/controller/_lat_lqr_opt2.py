"""Projected-reference adapter for the dynamic Riccati LQR core."""

import math

from ._lat_lqr_impl import LateralLQRController as _DynamicLateralLQRController


class LateralLQRController(_DynamicLateralLQRController):
    """Use continuous projection while retaining the full LQR state feedback."""

    def __init__(self, vehicle_para, Q=None, R=1.0, ts=0.05):
        super().__init__(vehicle_para, Q=Q, R=R, ts=ts)
        self.heading_match_weight = 2.0

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def control(self, x, y, phi, vx, vy, r, ref_path):
        if len(ref_path) < 2:
            return 0.0

        horizon = max(0.0, self.ts)
        px = x + (vx * math.cos(phi) - vy * math.sin(phi)) * horizon
        py = y + (vx * math.sin(phi) + vy * math.cos(phi)) * horizon
        pphi = phi + r * horizon
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
            ephi = self._wrap_angle(pphi - theta)
            score = distance_sq + self.heading_match_weight * ephi * ephi
            if best is None or score < best[0]:
                kappa = (1.0 - ratio) * ref_path[index][3] + ratio * ref_path[index + 1][3]
                best = (score, index, rx, ry, theta, kappa)

        if best is None:
            return 0.0

        _, index, rx, ry, theta, kappa = best
        self.min_index = index
        self.x_pro, self.y_pro = rx, ry
        nx, ny = -math.sin(theta), math.cos(theta)
        ed = nx * (px - rx) + ny * (py - ry)
        ephi = self._wrap_angle(pphi - theta)
        ed_dot = vy * math.cos(ephi) + vx * math.sin(ephi)
        s_dot = (vx * math.cos(ephi) - vy * math.sin(ephi)) / max(
            1e-3, 1.0 - kappa * ed
        )
        ephi_dot = r - kappa * s_dot
        self.last_ed, self.last_ephi = ed, ephi
        error_state = (ed, ed_dot, math.sin(ephi), ephi_dot)
        return self.control_from_error(error_state, kappa, vx)
