"""Latency-aware committed-path handover; all times are simulation seconds.

The committed path is a nominal prediction, not an exact plant rollout.
Tracking errors are bounded explicitly before requesting and accepting a tail.
"""
from dataclasses import dataclass, replace
import math

from ..utils.route import RouteGeometry, wrap_angle


@dataclass
class HandoverRequest:
    geometry: RouteGeometry
    start_s: float
    join_s: float
    requested_at: float
    deadline: float
    state: object
    prefix: list
    join: tuple


class PathHandover:
    def __init__(self, *, min_time=.15, max_time=.4, margin_time=.05,
                 position_tolerance=.35, heading_tolerance=.25,
                 curvature_tolerance=.12, reuse_time=.5):
        values = (min_time, max_time, margin_time, position_tolerance,
                  heading_tolerance, curvature_tolerance, reuse_time)
        if any(not math.isfinite(v) or v <= 0 for v in values) or max_time < min_time:
            raise ValueError('handover limits must be positive, finite and ordered')
        self.min_time, self.max_time, self.margin_time = min_time, max_time, margin_time
        self.position_tolerance, self.heading_tolerance = position_tolerance, heading_tolerance
        self.curvature_tolerance, self.reuse_time = curvature_tolerance, reuse_time
        self.latency = 0.
        self.path = []
        self.accepted_at = None
        self.request = None
        self.status = 'initial'
        self.tracking_errors = {}
        self.recovering = False
        self.stable_requests = 0

    @staticmethod
    def point(geometry, station):
        progress = geometry.segment_progress(station)
        i = min(len(geometry.path)-2, int(progress))
        ratio = progress-i
        a, b = geometry.path[i:i+2]
        return (a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1]),
                a[2]+ratio*wrap_angle(b[2]-a[2]), a[3]+ratio*(b[3]-a[3]))

    def tracking_ok(self, geometry, state, *, near_s=None, ahead=5.):
        direction = state.phi+math.atan2(state.vy, max(state.vx, .1))
        p = geometry.project(state.x, state.y, direction, near_s=near_s, ahead=ahead)
        point = self.point(geometry, p.s)
        curvature = state.r/state.speed if state.speed > .5 else point[3]
        self.tracking_errors = dict(position_m=p.distance,
                                    heading_rad=abs(wrap_angle(direction-point[2])),
                                    curvature_1pm=abs(curvature-point[3]))
        valid = (p.distance <= self.position_tolerance
                 and self.tracking_errors['heading_rad'] <= self.heading_tolerance
                 and self.tracking_errors['curvature_1pm'] <= self.curvature_tolerance)
        return p, valid

    def prepare(self, state, now):
        self.request = None
        if len(self.path) < 2:
            return state
        geometry = RouteGeometry(self.path)
        p, valid = self.tracking_ok(geometry, state)
        if not valid or state.vx < 0:
            self.recovering = True
            self.stable_requests = 0
            self.status = 'tracking_mismatch'
            return None
        if self.recovering:
            stable = (self.tracking_errors['position_m'] <= self.position_tolerance/2
                      and self.tracking_errors['heading_rad'] <= self.heading_tolerance/2
                      and self.tracking_errors['curvature_1pm'] <= self.curvature_tolerance/2)
            self.stable_requests = self.stable_requests+1 if stable else 0
            if self.stable_requests < 3:
                self.status = 'recovering'
                return None
            self.recovering = False
        horizon = min(self.max_time, max(self.min_time, self.latency+self.margin_time))
        # Actual speed/acceleration and heading error determine along-path progress.
        point = self.point(geometry, p.s)
        direction = state.phi+math.atan2(state.vy, max(state.vx, .1))
        lateral = -(state.x-point[0])*math.sin(point[2])+(state.y-point[1])*math.cos(point[2])
        along_speed = max(0., state.speed*math.cos(wrap_angle(direction-point[2]))
                          /max(.5, 1-point[3]*lateral))
        accel = max(-6., min(3., state.accel))
        travel_time = min(horizon, along_speed/-accel) if accel < 0 else horizon
        distance = max(0., along_speed*travel_time+.5*accel*travel_time**2)
        target = p.s+distance
        # Join at an existing vertex so the committed prefix remains unchanged.
        join_index = next((i for i, s in enumerate(geometry.s) if s >= target and s > p.s+1e-6), None)
        if join_index is None or join_index >= len(self.path)-1:
            self.status = 'insufficient_path'
            return None
        join_s, join = geometry.s[join_index], self.path[join_index]
        speed = max(.1, along_speed+accel*travel_time)
        predicted = replace(state, x=join[0], y=join[1], phi=join[2],
                            vx=speed, vy=0., r=join[3]*speed, timestamp=now+horizon)
        prefix = [point]+[self.path[i] for i, s in enumerate(geometry.s)
                          if p.s+1e-6 < s <= join_s]
        self.request = HandoverRequest(geometry, p.s, join_s, now, now+horizon,
                                      predicted, prefix, join)
        self.status = 'pending'
        return predicted

    def observe_latency(self, now, requested_at):
        elapsed = max(0., now-requested_at)
        # React immediately to overruns, decay slowly when computation gets faster.
        self.latency = max(elapsed, .8*self.latency+.2*elapsed)

    def splice(self, tail, state, now):
        request = self.request
        if request is None:
            return tail
        p, valid = self.tracking_ok(request.geometry, state, near_s=request.start_s,
                                   ahead=request.join_s-request.start_s+2.)
        if now > request.deadline or p.s >= request.join_s-1e-3:
            self.status = 'late_result'
            return None
        if not valid:
            self.status = 'tracking_mismatch'
            return None
        if not tail or math.dist(tail[0][:2], request.join[:2]) > 1e-4:
            self.status = 'invalid_join'
            return None
        if (abs(wrap_angle(tail[0][2]-request.join[2])) > min(.03, self.heading_tolerance)
                or abs(tail[0][3]-request.join[3]) > min(.02, self.curvature_tolerance)):
            self.status = 'invalid_join'
            return None
        # Keep old samples/tangents through the join. QP start matches its state.
        result = request.prefix+tail[1:]
        self.status = 'accepted'
        return result

    def reusable(self, state, now, *, recovery=False):
        if self.accepted_at is None or now-self.accepted_at > self.reuse_time or len(self.path) < 2:
            return []
        geometry = RouteGeometry(self.path)
        p, valid = self.tracking_ok(geometry, state)
        if recovery:
            # A bounded recovery request reanchors at the measured state; it
            # must not pretend that nominal future tracking remains accurate.
            valid = (self.tracking_errors['position_m'] <= 2*self.position_tolerance
                     and self.tracking_errors['heading_rad'] <= 2*self.heading_tolerance)
        if not valid or geometry.length-p.s < max(2., state.speed*self.min_time):
            return []
        return [self.point(geometry, p.s)]+[point for point, s in zip(self.path, geometry.s) if s > p.s+1e-6]

    def accept(self, path, now):
        self.path = list(path)
        self.accepted_at = now
        self.request = None
        if not path:
            self.recovering = False
            self.stable_requests = 0
