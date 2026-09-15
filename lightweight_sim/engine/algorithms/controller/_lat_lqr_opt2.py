"""Projected-reference adapter for the dynamic Riccati LQR core."""

import math

from ._lat_lqr_impl import LateralLQRController as _DynamicLateralLQRController


class LateralLQRController(_DynamicLateralLQRController):
    """Use progress-locked projection while retaining full LQR feedback.

    A figure-eight has two physically different route branches at the same
    Cartesian position.  A nearest-point search over the whole path therefore
    cannot identify the intended branch.  This adapter keeps an unwrapped
    route progress, searches only a local forward window, and scores heading
    continuity so the projection remains on the branch selected previously.
    """

    def __init__(self, vehicle_para, Q=None, R=1.0, ts=0.05):
        super().__init__(vehicle_para, Q=Q, R=R, ts=ts)
        self.heading_match_weight = 2.0
        self.heading_continuity_weight = 1.0
        self.progress_weight = 0.02
        self.search_back_segments = 2
        self.search_forward_segments = 48
        self.route_progress = 0.0
        self.last_ref_heading = None

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def reset_tracking(self):
        """Reset the route identity when a new scenario or route is loaded."""
        self.min_index = 0
        self.route_progress = 0.0
        self.last_ref_heading = None

    @staticmethod
    def _is_closed_path(ref_path) -> bool:
        return math.hypot(
            ref_path[0][0] - ref_path[-1][0],
            ref_path[0][1] - ref_path[-1][1],
        ) < 1.0

    def _project_reference(self, px, py, pphi, ref_path):
        """Project onto a locally reachable route segment.

        The returned progress is continuous even when a closed path wraps
        from its last segment back to segment zero.
        """
        segment_count = len(ref_path) - 1
        closed = self._is_closed_path(ref_path)
        if closed:
            base_progress = self.route_progress
        else:
            base_progress = min(
                max(self.route_progress, 0.0), max(0.0, segment_count - 1.0)
            )

        base_segment = int(math.floor(base_progress))
        best = None
        for offset in range(
            -self.search_back_segments,
            self.search_forward_segments + 1,
        ):
            candidate_progress = base_segment + offset
            if closed:
                index = candidate_progress % segment_count
            else:
                if candidate_progress < 0 or candidate_progress >= segment_count:
                    continue
                index = candidate_progress

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
            heading_error = self._wrap_angle(pphi - theta)
            continuity_error = (
                0.0
                if self.last_ref_heading is None
                else self._wrap_angle(theta - self.last_ref_heading)
            )
            score = (
                distance_sq
                + self.heading_match_weight * heading_error * heading_error
                + self.heading_continuity_weight
                * continuity_error
                * continuity_error
                + self.progress_weight * offset * offset
            )
            projected_progress = candidate_progress + ratio
            if best is None or score < best[0]:
                kappa = (1.0 - ratio) * ref_path[index][3] + ratio * ref_path[index + 1][3]
                best = (
                    score,
                    index,
                    projected_progress,
                    rx,
                    ry,
                    theta,
                    kappa,
                )
        return best, closed, segment_count

    def control(self, x, y, phi, vx, vy, r, ref_path):
        if len(ref_path) < 2:
            return 0.0

        horizon = max(0.0, self.ts)
        px = x + (vx * math.cos(phi) - vy * math.sin(phi)) * horizon
        py = y + (vx * math.sin(phi) + vy * math.cos(phi)) * horizon
        pphi = phi + r * horizon
        self.x_pre, self.y_pre = px, py

        best, closed, segment_count = self._project_reference(px, py, pphi, ref_path)
        if best is None:
            return 0.0

        _, index, progress, rx, ry, theta, kappa = best
        self.route_progress = progress
        self.min_index = int(math.floor(progress)) % segment_count if closed else index
        self.last_ref_heading = theta
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
