"""Metric, branch-continuous reference adapter for the dynamic Riccati LQR."""
import math

from ._lat_lqr_impl import LateralLQRController as _DynamicLateralLQRController
from ..utils.route import RouteTracker, wrap_angle


class LateralLQRController(_DynamicLateralLQRController):
    def __init__(self, vehicle_para, Q=None, R=100.0, ts=0.05):
        super().__init__(vehicle_para, Q=Q, R=R, ts=ts)
        self.tracker = None
        self.route_progress = 0.0  # Legacy diagnostic: segment units.
        self.route_s = 0.0       # Unwrapped metres.
        self.last_ref_heading = None
        self._path = None
        self.feedback_horizon_s = 0.0  # None reproduces the historical ts preview.
        self.smooth_reference_heading = True

    def reset_tracking(self):
        self.tracker = None
        self._path = None
        self.min_index = 0
        self.route_progress = self.route_s = 0.0
        self.last_ref_heading = None

    def set_path(self, path, *, preserve=False):
        previous = self.tracker.projection if self.tracker else None
        self.tracker = RouteTracker(path)
        self._path = path
        if preserve and previous is not None:
            # Local plans use a different origin for s. Transfer the physical
            # anchor with heading agreement, instead of reusing their indices.
            projection = self.tracker.geometry.project(
                previous.x, previous.y, previous.theta)
            self.tracker.s = projection.s
        elif self.tracker.geometry.closed:
            self.tracker.s = 0.0

    def control(self, x, y, phi, vx, vy, r, ref_path):
        if len(ref_path) < 2:
            return 0.0
        if self._path is not ref_path:
            self.set_path(ref_path, preserve=self.tracker is not None)
        horizon = self.ts if self.feedback_horizon_s is None else self.feedback_horizon_s
        horizon = max(0.0, horizon)
        px = x + (vx*math.cos(phi)-vy*math.sin(phi))*horizon
        py = y + (vx*math.sin(phi)+vy*math.cos(phi))*horizon
        pphi = phi + r*horizon
        self.x_pre, self.y_pre = px, py
        projection = self.tracker.update(px, py, pphi, math.hypot(vx, vy), self.ts)
        self.route_s = projection.s
        self.route_progress = self.tracker.geometry.segment_progress(projection.s)
        self.min_index = projection.index
        self.last_ref_heading = projection.theta
        self.x_pro, self.y_pro = projection.x, projection.y
        ed, ephi = projection.errors(px, py, pphi)
        if self.smooth_reference_heading:
            # The route is sampled, not a sequence of instantaneous corners.
            # Interpolate the already-computed vertex tangents continuously;
            # independent evaluation still uses the original segment tangent.
            a, b = ref_path[projection.index], ref_path[projection.index+1]
            theta = a[2]+projection.ratio*wrap_angle(b[2]-a[2])
            self.last_ref_heading = theta
            ed = -math.sin(theta)*(px-projection.x)+math.cos(theta)*(py-projection.y)
            ephi = wrap_angle(pphi-theta)
        kappa = projection.kappa
        ed_dot = vy*math.cos(ephi) + vx*math.sin(ephi)
        s_dot = (vx*math.cos(ephi)-vy*math.sin(ephi))/max(1e-3, 1-kappa*ed)
        ephi_dot = r-kappa*s_dot
        self.last_ed, self.last_ephi = ed, ephi
        return self.control_from_error((ed, ed_dot, math.sin(ephi), ephi_dot), kappa, vx)
