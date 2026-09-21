"""Shared measurement definition, independent of controller prediction state."""
import math
from ..algorithms.utils.route import RouteTracker, wrap_angle

PROTOCOL = "route_projection_v2"


class TrackingMonitor:
    def __init__(self, path):
        self.tracker = RouteTracker(path)
        if self.tracker.geometry.closed:
            self.tracker.s = 0.0
        self.last_time = None

    def update(self, state):
        dt = 0.05 if self.last_time is None else max(0.0, state.timestamp-self.last_time)
        self.last_time = state.timestamp
        p = self.tracker.update(state.x, state.y, state.phi, state.speed, dt)
        ed, ephi = p.errors(state.x, state.y, state.phi)
        return dict(protocol=PROTOCOL, timestamp=state.timestamp,
                    ed_m=ed, ephi_deg=math.degrees(ephi), route_s_m=p.s,
                    route_length_m=self.tracker.geometry.length,
                    lap=max(0, math.floor(p.s/self.tracker.geometry.length)),
                    reference_heading_rad=p.theta, reference_index=p.index,
                    reference_x_m=p.x, reference_y_m=p.y)


class FigureEightOracle:
    """Independent ordered analytic-route oracle; never reads control indices.

    Fits the true position to the analytic x=78 cos(t), y=42 sin(2t)
    in a bounded phase neighbourhood. Used only for branch/lap checks, not
    tracking-error numbers (those use the actual archived polyline).
    """
    def __init__(self):
        self.phase = 0.0

    def update(self, x, y):
        lo, hi = self.phase-0.08, self.phase+0.08
        def cost(t):
            return (x-78*math.cos(t))**2 + (y-42*math.sin(2*t))**2
        for _ in range(28):
            a, b = lo+(hi-lo)/3, hi-(hi-lo)/3
            if cost(a) < cost(b):
                hi = b
            else:
                lo = a
        self.phase = (lo+hi)/2
        theta = math.atan2(84*math.cos(2*self.phase), -78*math.sin(self.phase))
        phase = self.phase % (2*math.pi)
        region = "other"
        if abs(wrap_angle(phase-math.pi/2)) < 0.15:
            region = "crossing_1"
        elif abs(wrap_angle(phase-3*math.pi/2)) < 0.15:
            region = "crossing_2"
        elif abs(wrap_angle(phase)) < 0.15:
            region = "seam"
        return self.phase, theta, region

    @staticmethod
    def wrong_branch(reference_heading, oracle_heading, region):
        return region.startswith("crossing") and abs(
            wrap_angle(reference_heading-oracle_heading)) > math.radians(45)
