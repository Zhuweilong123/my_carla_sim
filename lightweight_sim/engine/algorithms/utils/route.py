"""Metric route geometry and stateful, branch-continuous projection.

Progress is unwrapped arc length in metres. Only initialization/replanning may
search globally; normal tracking is bounded by travelled distance, not density.
"""
from bisect import bisect_right
from dataclasses import dataclass
import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Projection:
    index: int
    ratio: float
    s: float
    x: float
    y: float
    theta: float
    kappa: float
    distance: float

    def errors(self, x, y, heading):
        return (-math.sin(self.theta) * (x - self.x)
                + math.cos(self.theta) * (y - self.y),
                wrap_angle(heading - self.theta))


class RouteGeometry:
    def __init__(self, path):
        self.path = tuple(tuple(p) for p in path)
        if len(self.path) < 2:
            raise ValueError("route needs at least two points")
        self.s = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0]-a[0], b[1]-a[1]))
        self.length = self.s[-1]
        if self.length <= 1e-9:
            raise ValueError("route has no nonzero segments")
        self.closed = math.hypot(self.path[0][0]-self.path[-1][0],
                                 self.path[0][1]-self.path[-1][1]) < 1e-6

    def project(self, x, y, heading, *, near_s=None, back=2.0, ahead=5.0):
        best = None
        for i, (a, b) in enumerate(zip(self.path, self.path[1:])):
            length = self.s[i+1] - self.s[i]
            if length < 1e-9:
                continue
            cycles = [0]
            if self.closed and near_s is not None:
                lap = math.floor(near_s / self.length)
                cycles = [lap-1, lap, lap+1]
            for lap in cycles:
                start = self.s[i] + lap * self.length
                lo, hi = 0.0, 1.0
                if near_s is not None:
                    lo = max(0.0, (near_s-back-start)/length)
                    hi = min(1.0, (near_s+ahead-start)/length)
                    if lo > hi:
                        continue
                dx, dy = b[0]-a[0], b[1]-a[1]
                t = max(lo, min(hi, ((x-a[0])*dx+(y-a[1])*dy)/(length*length)))
                rx, ry = a[0]+t*dx, a[1]+t*dy
                theta = math.atan2(dy, dx)
                distance = math.hypot(x-rx, y-ry)
                progress = start + t*length
                score = distance**2 + 2.0*wrap_angle(heading-theta)**2
                if near_s is not None:
                    score += 1e-6*(progress-near_s)**2
                if best is None or score < best[0]:
                    best = (score, Projection(i, t, progress, rx, ry, theta,
                            (1-t)*a[3]+t*b[3], distance))
        if best is None:
            raise ValueError("no reachable reference segment")
        return best[1]

    def segment_progress(self, s):
        local = s % self.length if self.closed else max(0.0, min(s, self.length))
        i = min(len(self.path)-2, bisect_right(self.s, local)-1)
        ratio = (local-self.s[i])/max(1e-9, self.s[i+1]-self.s[i])
        lap = math.floor(s/self.length) if self.closed else 0
        return lap*(len(self.path)-1) + i + ratio


class RouteTracker:
    def __init__(self, path, start_s=None):
        self.geometry = RouteGeometry(path)
        self.s = start_s
        self.projection = None

    def update(self, x, y, heading, speed=0.0, dt=0.05):
        projection = self.geometry.project(
            x, y, heading, near_s=self.s, back=2.0,
            ahead=max(3.0, abs(speed)*dt*3.0+1.0))
        self.s = projection.s
        self.projection = projection
        return projection
