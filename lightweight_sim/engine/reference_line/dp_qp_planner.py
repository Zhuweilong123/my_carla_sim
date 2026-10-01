"""Route corridor adapter for the DP + QP algorithm."""
from __future__ import annotations

import math
import numpy as np

from .route_aware_planner import CorridorMotionPlanner, BaselinePathPlanner
from ..algorithms.planner.dp_qp import DP_algorithm, Quadratic_planning, quintic_edge, adaptive_qp_knots, adaptive_dp_knots
from ..algorithms.utils.frenet import find_match_points
from ..algorithms.utils.geometry import cal_heading_kappa


class DPQPPathPlanner(CorridorMotionPlanner):
    """Search continuous road width, then optimize within the DP homotopy."""

    def __init__(self, *args, dp_station_step_m=8., dp_lateral_step_m=.5,
                 qp_station_step_m=4., qp_time_limit_s=.08, **kwargs):
        try:
            import scipy.sparse
            import osqp  # Verify the selected backend before starting.
        except ImportError as exc:
            raise RuntimeError(
                'DP+QP requires SciPy and OSQP in the ROS Python environment. '
                'On Ubuntu/WSL run: sudo apt install python3-scipy; '
                'python3 -m pip install --user --break-system-packages "osqp>=1.0"'
            ) from exc
        super().__init__(*args, **kwargs)
        for value in (dp_station_step_m, dp_lateral_step_m, qp_station_step_m, qp_time_limit_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('DP/QP sampling steps must be positive and finite')
        self.dp_station_step_m = float(dp_station_step_m)
        self.dp_lateral_step_m = float(dp_lateral_step_m)
        self.qp_station_step_m = float(qp_station_step_m)
        self.qp_time_limit_s = float(qp_time_limit_s)
        self.last_qp_diagnostics = {}
        self._previous_profile = None
        self.last_status = 'not_started'

    def _plan(self, pred_loc, vehicle_loc, obstacles):
        self.last_qp_diagnostics = {}
        self.last_status = 'invalid_reference'
        if len(self.global_path) < 2:
            return []
        # Start at the actual vehicle: initial Frenet derivatives belong here.
        match, projections = find_match_points([vehicle_loc], self.global_path, True, 0)
        first = int(match[0])
        closed = math.dist(self.global_path[0][:2], self.global_path[-1][:2]) < 1e-6
        count = len(self.global_path)-int(closed)
        indices = [(first+i) % count for i in range(min(count, self.horizon_points))]
        if not closed:
            indices = [i for i in indices if i >= first]
        ref = np.asarray([self.global_path[i] for i in indices], dtype=float)
        if len(ref) < 2:
            return []
        ref[0] = projections[0]
        s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1))]
        keep = np.r_[True, np.diff(s)>1e-6]
        bounds = np.asarray([self._corridor_bounds(i, p) for i, p in zip(indices, ref)])
        ref, s, bounds = ref[keep], s[keep], bounds[keep]
        if len(s) < 2 or s[-1] < 1.:
            return []
        angles = np.unwrap(ref[:, 2])

        def geometry(t):
            return (np.interp(t, s, ref[:, 0]), np.interp(t, s, ref[:, 1]),
                    np.interp(t, s, angles))

        global_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(np.asarray(self.global_path)[:, :2], axis=0), axis=1))]
        anchor = global_s[first]+math.dist(self.global_path[first][:2], ref[0, :2])
        if closed and self._previous_profile is not None:
            previous_anchor = self._previous_profile[0]
            anchor += round((previous_anchor-anchor)/global_s[-1])*global_s[-1]
        normal = (-math.sin(ref[0, 2]), math.cos(ref[0, 2]))
        l0 = np.dot(np.asarray(vehicle_loc)-ref[0, :2], normal)
        dl0, ddl0 = 0., 0.
        state = getattr(self, '_planning_state', None)
        if state is not None:
            phi, vx, vy, yaw_rate = state
            delta = math.remainder(phi + math.atan2(vy, max(abs(vx), .1))-ref[0, 2], 2*math.pi)
            if abs(delta) > .7:
                self.last_status = 'unsupported_start_heading'
                return []
            factor = 1.-ref[0, 3]*l0
            dl0 = factor*math.tan(delta)
            curvature = yaw_rate/math.hypot(vx, vy) if math.hypot(vx, vy) > .5 else ref[0, 3]
            ddl0 = factor*(factor*curvature/math.cos(delta)-ref[0, 3])/math.cos(delta)**2
        start = (float(l0), dl0, ddl0)
        preferred = (self.lane_width*(self.target_lane-self.reference_lane_index)
                     if 0 <= self.target_lane < self.num_lanes else 0.)

        # Project rectangles onto the local reference using physical arc length.
        obstacle_sl = []
        for ox, oy, length, width, _, heading in obstacles:
            idx, projection = find_match_points([(ox, oy)], ref.tolist(), True, 0)
            i = min(int(idx[0]), len(ref)-2)
            px, py, theta, _ = projection[0]
            distance = (s[i]+math.hypot(px-ref[i, 0], py-ref[i, 1])
                        +(ox-px)*math.cos(theta)+(oy-py)*math.sin(theta))
            lateral = -(ox-px)*math.sin(theta)+(oy-py)*math.cos(theta)
            delta = heading-theta
            half_s = abs(math.cos(delta))*length/2+abs(math.sin(delta))*width/2
            half_l = abs(math.sin(delta))*length/2+abs(math.cos(delta))*width/2
            obstacle_sl.append((distance, lateral, half_s, half_l))

        def road_bounds(t):
            return (np.interp(t, s, bounds[:, 0]), np.interp(t, s, bounds[:, 1]))

        # One envelope shared by DP edge validation and QP corridor constraints.
        def slope_limit(t):
            return np.full(np.shape(t), np.tan(.35))

        def convex_bounds(t, lateral, offset=0.):
            lo, hi = road_bounds(t)
            lo, hi = np.broadcast_to(lo, np.shape(lateral)).copy(), np.broadcast_to(hi, np.shape(lateral)).copy()
            extent = self.vehicle_width_m/2
            half_s = self.vehicle_width_m/2*math.sin(.35)
            for os, ol, hs, hl in obstacle_sl:
                mask = abs(t+offset-os) <= hs+half_s+self.collision_margin_m
                left = mask & (lateral >= ol)
                right = mask & ~left
                lo = np.where(left, np.maximum(lo, ol+hl+extent+self.collision_margin_m+.02), lo)
                hi = np.where(right, np.minimum(hi, ol-hl-extent-self.collision_margin_m-.02), hi)
            return lo, hi

        body_offsets = np.linspace(-self.vehicle_length_m/2, self.vehicle_length_m/2, 5)
        def edge_safe(t, l, dl, ddl):
            valid = np.all((abs(dl) <= slope_limit(t)+1e-5), axis=-1)
            for offset in body_offsets:
                lower, upper = convex_bounds(t, l, offset)
                footprint = l+offset*dl
                valid &= np.all((footprint >= lower-1e-5) & (footprint <= upper+1e-5), axis=-1)
            return valid

        stations = adaptive_dp_knots(s[-1], self.dp_station_step_m,
                                     [os for os, _, _, _ in obstacle_sl])
        lateral_samples = []
        for station in stations:
            lo, hi = road_bounds(station)
            values = np.arange(lo, hi+1e-8, self.dp_lateral_step_m)
            extent = self.vehicle_width_m/2
            obstacle_edges = [ol+sign*(hl+extent+self.collision_margin_m+.02)
                              for _, ol, _, hl in obstacle_sl for sign in (-1., 1.)]
            continuation = l0+station*np.asarray([-.3, -.15, .15, .3])
            lateral_samples.append(np.unique(np.clip(np.r_[values, hi, preferred, l0, obstacle_edges, continuation], lo, hi)))
        def clearance_cost(t, l):
            cost = np.zeros_like(l)
            for os, ol, hs, hl in obstacle_sl:
                gap = np.maximum(abs(l-ol)-hl-self.vehicle_width_m/2, .5)
                cost += 10*np.exp(-((t-os)/(hs+8.))**2)/gap**2
            return cost
        values = DP_algorithm(stations, lateral_samples, start, preferred, edge_safe,
                              min(.5, self.sampling_resolution_m), edge_cost=clearance_cost)
        if values is None:
            self.last_status = 'dp_infeasible'
            return []
        # Dense DP curve supplies the obstacle side for each convex corridor.
        samples = np.linspace(0., s[-1], max(2, int(np.ceil(s[-1]/min(.5, self.sampling_resolution_m)))+1))
        dp_l = np.empty(len(samples))

        for i in range(len(stations)-1):
            mask = (samples >= stations[i]) & (samples <= stations[i+1])
            initial = values[i]
            dp_l[mask] = quintic_edge(
                stations[i+1]-stations[i], initial, values[i+1], samples[mask]-stations[i])[0]
        lower, upper = convex_bounds(samples, dp_l)
        if np.any(lower > upper):
            self.last_status = 'corridor_infeasible'
            return []
        knots = adaptive_qp_knots(s[-1], self.qp_station_step_m,
                                  [os for os, _, _, _ in obstacle_sl])
        target_profile = dp_l
        if self._previous_profile is not None:
            previous_anchor, previous_s, previous_l = self._previous_profile
            relative = anchor-previous_anchor+samples
            valid = (relative >= 0.) & (relative <= previous_s[-1])
            previous_target = np.interp(relative, previous_s, previous_l)
            target_profile = np.where(valid, (dp_l+5*previous_target)/6, dp_l)
        optimized = Quadratic_planning(knots, samples, start, target_profile, lower, upper, preferred,
                                       road_limits=road_bounds(samples),
                                       footprint_constraints=[(offset, *convex_bounds(samples, dp_l, offset))
                                                              for offset in body_offsets],
                                       half_length=self.vehicle_length_m/2,
                                       max_slope=slope_limit(samples),
                                       max_second_derivative=max(.08, abs(ddl0)+.005),
                                       time_limit_s=self.qp_time_limit_s,
                                       diagnostics=self.last_qp_diagnostics)
        if optimized is None:
            self.last_status = 'qp_failed'
            return []
        l, dl, ddl = optimized
        x, y, theta = geometry(samples)
        points = list(zip(x-l*np.sin(theta), y+l*np.cos(theta)))
        heading, curvature = cal_heading_kappa(points)
        result = [(float(px), float(py), float(h), float(k))
                  for (px, py), h, k in zip(points, heading, curvature)]
        if not edge_safe(samples, l, dl, ddl) or not self._trajectory_is_safe(result, obstacles):
            self.last_status = 'validation_failed'
            return []
        # Project all body corners onto all reference segments in one batch.
        xy = np.asarray(points)
        headings = np.asarray(heading)
        tangent = np.column_stack((np.cos(headings), np.sin(headings)))
        normal = np.column_stack((-np.sin(headings), np.cos(headings)))
        corners = np.concatenate([
            xy+longitudinal*tangent+lateral*normal
            for longitudinal in (-self.vehicle_length_m/2, self.vehicle_length_m/2)
            for lateral in (-self.vehicle_width_m/2, self.vehicle_width_m/2)])
        segments = np.diff(ref[:, :2], axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        delta = corners[:, None, :]-ref[None, :-1, :2]
        ratios = np.clip(np.sum(delta*segments[None, :, :], axis=-1)/lengths**2, 0., 1.)
        projected = ref[None, :-1, :2]+ratios[:, :, None]*segments[None, :, :]
        nearest = np.argmin(np.sum((corners[:, None, :]-projected)**2, axis=-1), axis=1)
        row = np.arange(len(corners))
        offset_xy = corners-projected[row, nearest]
        unit = segments[nearest]/lengths[nearest, None]
        offsets = -offset_xy[:, 0]*unit[:, 1]+offset_xy[:, 1]*unit[:, 0]
        corner_s = s[nearest]+ratios[row, nearest]*lengths[nearest]
        lo, hi = road_bounds(corner_s)
        if (np.any(offsets < lo-self.corridor_margin_m-1e-5)
                or np.any(offsets > hi+self.corridor_margin_m+1e-5)):
            self.last_status = 'body_outside_road'
            return []
        self._previous_profile = (anchor, samples.copy(), l.copy())
        self.last_status = 'solved'
        return result


def create_local_planner(algorithm, *args, **kwargs):
    """Explicit configuration selection; no algorithm substitution on failure."""
    if algorithm == 'dp_qp':
        return DPQPPathPlanner(*args, **kwargs)
    if algorithm == 'baseline':
        for name in ('dp_station_step_m', 'dp_lateral_step_m', 'qp_station_step_m', 'qp_time_limit_s'):
            kwargs.pop(name, None)
        return BaselinePathPlanner(*args, **kwargs)
    raise ValueError(f'unknown local planner algorithm: {algorithm}')
