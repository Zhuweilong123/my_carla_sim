"""Independent ST DP + jerk QP speed planning on a fixed geometric path."""
from dataclasses import dataclass
import math

import numpy as np

from ..utils.route import RouteGeometry
from ...simulator.data_types import VehicleParams


@dataclass(frozen=True)
class SpeedConfig:
    horizon_s: float = 6.0
    time_step_s: float = .2
    max_accel: float = 2.0
    max_decel: float = 3.0
    max_jerk: float = 3.0
    collision_margin_m: float = .3
    stop_gap_m: float = .5
    beam_width: int = 96
    qp_time_limit_s: float = .06

    def __post_init__(self):
        values = (self.horizon_s, self.time_step_s, self.max_accel, self.max_decel,
                  self.max_jerk, self.qp_time_limit_s)
        if (not all(math.isfinite(v) and v > 0 for v in values)
                or self.horizon_s < 3*self.time_step_s
                or not isinstance(self.beam_width, int) or self.beam_width < 8
                or not all(math.isfinite(v) and v >= 0 for v in
                           (self.collision_margin_m, self.stop_gap_m))):
            raise ValueError('invalid ST speed settings')


@dataclass
class SpeedPlan:
    time: np.ndarray
    s: np.ndarray
    speed: np.ndarray
    accel: np.ndarray
    jerk: np.ndarray
    origin_s: float
    stop_s: float | None
    status: str
    dp_cost: float = 0.

    @property
    def valid(self):
        return self.status == 'solved'

    def sample(self, elapsed):
        """Evaluate the same constant-jerk dynamics used by the optimizer."""
        if not self.valid or not math.isfinite(elapsed) or not -1e-9 <= elapsed <= self.time[-1]+1e-9:
            raise ValueError('speed reference is invalid or expired')
        elapsed = np.clip(elapsed, 0., self.time[-1])
        i = min(len(self.jerk)-1, max(0, int(np.searchsorted(self.time, elapsed, side='right')-1)))
        dt = elapsed-self.time[i]
        j = self.jerk[i]
        return (self.s[i]+self.speed[i]*dt+.5*self.accel[i]*dt**2+j*dt**3/6,
                max(0., self.speed[i]+self.accel[i]*dt+.5*j*dt**2), self.accel[i]+j*dt)


def _intersects(x, y, heading, length, width, obstacle, margin):
    """SAT footprints, including both headings and the physical ego dimensions."""
    ea = np.array([[math.cos(heading), math.sin(heading)],
                   [-math.sin(heading), math.cos(heading)]])
    oa = np.array([[math.cos(obstacle.heading), math.sin(obstacle.heading)],
                   [-math.sin(obstacle.heading), math.cos(obstacle.heading)]])
    delta = np.array([obstacle.x-x, obstacle.y-y])
    for axis in np.vstack((ea, oa)):
        ego_radius = length/2*abs(axis@ea[0])+width/2*abs(axis@ea[1])
        obs_radius = obstacle.length/2*abs(axis@oa[0])+obstacle.width/2*abs(axis@oa[1])
        if abs(axis@delta) > ego_radius+obs_radius+margin:
            return False
    return True


class STSpeedPlanner:
    """First version: static-obstacle/goal ST boundaries; moving conflicts fail closed.

    DP samples bounded jerk controls on a time lattice, retaining a beam of
    distinct (s,v,a) states. QP optimizes the DP corridor with exact constant
    jerk integration. Neither solver changes the supplied geometric path.
    """
    def __init__(self, config=None):
        self.config = config or SpeedConfig()
        try:
            import scipy.sparse as sp
            import osqp
        except ModuleNotFoundError as exc:
            raise RuntimeError('ST DP+QP requires SciPy and OSQP in the ROS Python environment; '
                               'sudo apt install python3-scipy; '
                               'python3 -m pip install --user --break-system-packages "osqp>=1.0"') from exc
        self.sparse, self.osqp = sp, osqp

    def plan(self, path, state, obstacles=(), *, vehicle=None, target_speed_kmh=40.,
             max_lateral_accel=2., destination=None, closed_route=False, speed_limits=()):
        c, vehicle = self.config, vehicle or VehicleParams()
        sp, osqp = self.sparse, self.osqp
        empty = lambda status: SpeedPlan(*(np.array([]) for _ in range(5)), 0., None, status)
        if (len(path) < 2 or not all(math.isfinite(v) for p in path for v in p)
                or any(len(p) != 4 for p in path)
                or not math.isfinite(target_speed_kmh) or target_speed_kmh <= 0
                or not math.isfinite(max_lateral_accel) or max_lateral_accel <= 0
                or not all(math.isfinite(v) for v in (state.x, state.y, state.phi,
                                                      state.vx, state.vy, state.accel))):
            return empty('invalid_input')
        if (any(not all(math.isfinite(v) for v in (o.x, o.y, o.heading, o.speed, o.length, o.width))
                or o.length <= 0 or o.width <= 0 for o in obstacles)
                or any(not all(math.isfinite(v) for v in row) or row[0] > row[1] or row[2] < 0
                       for row in speed_limits)
                or (destination is not None and (len(destination) < 2
                    or not all(math.isfinite(v) for v in destination[:2])))):
            return empty('invalid_input')
        if state.vx < -.05:
            return empty('reverse_motion')
        try:
            geometry = RouteGeometry(path)
        except ValueError:
            return empty('invalid_input')
        projection = geometry.project(state.x, state.y, state.phi)
        origin = projection.s
        remaining = geometry.length-origin
        if remaining < .05:
            return empty('path_exhausted')
        stations = np.arange(0., remaining, .25)
        stations = np.r_[stations, remaining]
        absolute = stations+origin
        points = np.asarray(path)
        headings = np.unwrap(points[:, 2])
        x, y, heading = (np.interp(absolute, geometry.s, points[:, i]) for i in (0, 1, 2))
        heading = np.interp(absolute, geometry.s, headings)
        kappa = np.abs(np.interp(absolute, geometry.s, points[:, 3]))
        caps = np.minimum(target_speed_kmh/3.6,
                          np.sqrt(max_lateral_accel/np.maximum(kappa, 1e-6)))
        # Route limits are projected onto this path by the ROS adapter.
        for begin, end, limit in speed_limits:
            caps[(absolute >= begin) & (absolute <= end)] = np.minimum(
                caps[(absolute >= begin) & (absolute <= end)], limit/3.6)
        road_caps = caps.copy()
        stop, terminal_stop = None, False
        for obstacle in obstacles:
            hits = [s for s, px, py, ph in zip(stations, x, y, heading)
                    if _intersects(px, py, ph, vehicle.length, vehicle.width,
                                   obstacle, c.collision_margin_m+.125)]
            if hits:
                if abs(obstacle.speed) > 1e-3:
                    return empty('unsupported_moving_conflict')
                candidate = max(0., min(hits)-c.stop_gap_m)
                stop = candidate if stop is None else min(stop, candidate)
        if destination is not None and not closed_route:
            goal = geometry.project(destination[0], destination[1], state.phi)
            if goal.distance < .75 and goal.s >= origin-.5:
                candidate = max(0., goal.s-origin)
                stop = candidate if stop is None else min(stop, candidate)
        v0 = state.speed
        amax, decel = min(c.max_accel, vehicle.max_accel), min(c.max_decel, vehicle.max_decel)
        a0 = float(np.clip(state.accel, -vehicle.max_decel, vehicle.max_accel))
        if v0 < .05:
            a0 = max(0., a0)
        if stop is not None and stop < .05 and v0 < .15:
            t = np.arange(0., c.horizon_s+c.time_step_s/2, c.time_step_s)
            z = np.zeros_like(t)
            return SpeedPlan(t, z.copy(), z.copy(), z.copy(), z[:-1].copy(), origin, stop, 'solved')
        # A truncated local path is not a stop line. Shorten the prediction
        # horizon before reaching unknown geometry instead of adding v_end=0.
        horizon = (c.horizon_s if stop is not None else
                   min(c.horizon_s, remaining/max(v0, target_speed_kmh/3.6, 1.)))
        n = max(3, int(horizon/c.time_step_s))
        dt = min(c.time_step_s, horizon/n)
        time = np.arange(n+1)*dt
        # A safety brake can exceed comfort deceleration. Preserve the measured
        # initial acceleration and return to comfort bounds at bounded jerk.
        acceleration_recovery = max(dt, math.ceil(max(a0-amax, -decel-a0, 0.)/c.max_jerk/dt)*dt)
        def accel_bounds(t):
            fraction = min(1., t/acceleration_recovery)
            return (min(-decel, a0+(-decel-a0)*fraction),
                    max(amax, a0+(amax-a0)*fraction))
        upper_s = remaining-.01
        if stop is not None:
            upper_s = min(upper_s, stop)
            terminal_stop = stop < max(v0, target_speed_kmh/3.6)*time[-1]*.8
            caps = np.minimum(caps, np.sqrt(2*decel*np.maximum(stop-stations, 0.)))
        # Back-propagate future restrictions before DP pruning. A conservative
        # braking envelope reserves acceleration/jerk transitions, so a beam
        # pursuing the current high cap does not lose all braking candidates.
        for i in range(len(caps)-2, -1, -1):
            caps[i] = min(caps[i], math.sqrt(caps[i+1]**2+decel*(stations[i+1]-stations[i])))
        # If already above a newly lowered speed cap, enforce the fastest
        # jerk-bounded recovery envelope rather than demanding an impossible
        # instantaneous speed jump. Static stop bounds remain hard.
        ramp = abs(a0+decel)/c.max_jerk
        recovery_jerk = -c.max_jerk if a0 >= -decel else c.max_jerk
        def recovery(t):
            tau = np.minimum(t, ramp)
            return np.maximum(0., v0+a0*tau+.5*recovery_jerk*tau**2
                              -decel*np.maximum(t-ramp, 0.))
        def limit(s, t):
            return np.maximum(np.interp(s, stations, caps), recovery(t))
        def qp_limit(s, t):
            # DP's anticipatory braking envelope guides the search. The QP
            # instead enforces physical stop position/terminal dynamics and
            # actual road caps, allowing its arrival time to move.
            return np.maximum(np.interp(s, stations, road_caps), recovery(t))
        # State: [s, v, a]. DP edges use the same integrator as the QP.
        states = np.array([[0., v0, a0]])
        costs = np.zeros(1)
        layers, parents = [states], []
        actions = np.linspace(-c.max_jerk, c.max_jerk, 7)
        action_count = len(actions)+4
        for step in range(n):
            # Boundary-reaching jerk actions avoid quantization dead ends when
            # measured a0 is not an exact multiple of the lattice spacing.
            lo_a, hi_a = accel_bounds(time[step+1])
            adaptive = np.clip(np.column_stack(((lo_a-states[:, 2])/dt,
                                                (hi_a-states[:, 2])/dt,
                                                -states[:, 2]/dt,
                                                -states[:, 1]/dt**2-1.5*states[:, 2]/dt)),
                               -c.max_jerk, c.max_jerk)
            j = np.column_stack((np.tile(actions, (len(states), 1)), adaptive)).ravel()
            previous = np.repeat(states, action_count, axis=0)
            s, v, a = previous.T
            next_states = np.column_stack((s+v*dt+.5*a*dt**2+j*dt**3/6,
                                          v+a*dt+.5*j*dt**2, a+j*dt))
            ns, nv, na = next_states.T
            mid_v = v+a*dt/2+j*dt**2/8
            valid = ((ns <= upper_s+1e-6) & (ns >= s-1e-6) & (nv >= -1e-6)
                     & (mid_v >= -1e-6) & (na <= accel_bounds(time[step+1])[1]+1e-6)
                     & (na >= accel_bounds(time[step+1])[0]-1e-6)
                     & (nv >= np.minimum(na, 0.)**2/(2*c.max_jerk)-1e-6)
                     & (nv <= limit(ns, time[step+1])+.15))
            new_cost = np.repeat(costs, action_count)+dt*(4*(nv-np.interp(ns, stations, caps))**2
                                                         +.3*na**2+.1*j**2)
            if step == n-1 and terminal_stop:
                new_cost += 100*nv**2 + 5*(ns-upper_s)**2
            indices = np.flatnonzero(valid)
            if not len(indices):
                return empty('dp_infeasible_'+str(step))
            order = indices[np.argsort(new_cost[indices])]
            # Keep alternative positions/velocities/accelerations, not merely
            # many identical histories with the cheapest short-term speed.
            # Quantization must never discard the fastest braking state: it
            # can be the only state meeting an overspeed recovery envelope.
            quantized = np.round(next_states/[.5, .25, .5]).astype(int)
            low_speed = nv < .5
            quantized[low_speed] = np.round(next_states[low_speed]/[.5, .02, .05]).astype(int)
            def key_for(i):
                return tuple(quantized[i])
            fastest = int(indices[np.argmin(nv[indices])])
            selected, seen = [fastest], {key_for(fastest)}
            for i in order:
                key = key_for(i)
                if key not in seen:
                    seen.add(key)
                    selected.append(i)
                    if len(selected) >= c.beam_width:
                        break
            indices = np.array(selected)
            parents.append(indices//action_count)
            states, costs = next_states[indices], new_cost[indices]
            layers.append(states)
        best = int(np.argmin(costs))
        dp_cost = float(costs[best])
        indices = [best]
        for parent in reversed(parents):
            best = int(parent[best])
            indices.append(best)
        seed = np.array([layer[index] for layer, index in zip(layers, reversed(indices))])
        # Optimize x=[s_0..s_N,v_0..v_N,a_0..a_N,j_0..j_(N-1)].
        m, size = n+1, 4*n+3
        weights = np.r_[np.full(m, .4), np.full(m, 8.), np.full(m, .4), np.full(n, .2)]
        desired_v = np.interp(seed[:, 0], stations, caps)
        reference = np.r_[seed[:, 0], desired_v, np.zeros(m+n)]
        if terminal_stop:
            weights[n] = 200.
            reference[n] = upper_s
        rows, lower, upper = [], [], []
        def constraint(entries, lo, hi):
            rows.append(entries)
            lower.append(lo)
            upper.append(hi)
        for offset, value in [(0, 0.), (m, v0), (2*m, a0)]:
            constraint({offset: 1.}, value, value)
        for i in range(n):
            j = 3*m+i
            constraint({i+1: 1., i: -1., m+i: -dt, 2*m+i: -.5*dt**2, j: -dt**3/6}, 0., 0.)
            constraint({m+i+1: 1., m+i: -1., 2*m+i: -dt, j: -.5*dt**2}, 0., 0.)
            constraint({2*m+i+1: 1., 2*m+i: -1., j: -dt}, 0., 0.)
        speed_rows = []
        # Static ST boundaries are one convex free interval. Do not impose a
        # narrow DP timing corridor on a terminal stop: DP guides the topology,
        # while the QP must be free to move its deceleration/arrival time.
        corridor_low = np.zeros(m)
        corridor_high = np.minimum(upper_s, seed[:, 0]+(upper_s if stop is not None else 2.))
        corridor_caps = []
        for i in range(m):
            # Bound both adjacent intervals conservatively, so a QP station
            # shift cannot move a high-speed knot into a lower spatial cap.
            lo = corridor_low[max(0, i-1)]
            hi = corridor_high[min(n, i+1)]
            values = road_caps[(stations >= lo) & (stations <= hi)]
            corridor_caps.append(min(float(np.interp(lo, stations, road_caps)),
                                     float(np.interp(hi, stations, road_caps)),
                                     float(np.min(values)) if len(values) else float('inf')))
        for i in range(m):
            constraint({i: 1.}, corridor_low[i], corridor_high[i])
            speed_rows.append(len(rows))
            constraint({m+i: 1.}, 0., max(v0, corridor_caps[0]) if i == 0 else
                       max(corridor_caps[i], float(recovery(time[i]))))
            constraint({2*m+i: 1.}, *accel_bounds(time[i]))
        for i in range(n):
            constraint({3*m+i: 1.}, -c.max_jerk, c.max_jerk)
        if terminal_stop:
            constraint({m+n: 1.}, 0., 0.)
            constraint({2*m+n: 1.}, 0., 0.)
        ri, ci, values = [], [], []
        for r, entries in enumerate(rows):
            for col, value in entries.items():
                ri.append(r); ci.append(col); values.append(value)
        matrix = sp.csc_matrix((values, (ri, ci)), shape=(len(rows), size))
        solver = osqp.OSQP()
        solver.setup(P=sp.diags(2*weights, format='csc'), q=-2*weights*reference,
                     A=matrix, l=np.array(lower), u=np.array(upper), verbose=False,
                     eps_abs=1e-5, eps_rel=1e-5, max_iter=10000,
                     time_limit=c.qp_time_limit_s, polishing=True)
        # Station changes alter curvature/route caps. Refine the frozen QP
        # limits at the optimized stations; never publish unchecked knots.
        for iteration in range(4):
            solution = solver.solve(raise_error=False)
            if solution.x is None or not solution.info.status.lower().startswith('solved'):
                return empty('qp_'+solution.info.status.replace(' ', '_'))
            z = solution.x
            residual = matrix@z
            if np.max(np.maximum(np.array(lower)-residual, residual-np.array(upper))) > 2e-3:
                return empty('qp_constraint_violation')
            result = SpeedPlan(time, z[:m], z[m:2*m], z[2*m:3*m], z[3*m:], origin, stop,
                               'solved', dp_cost)
            valid = True
            for i in range(n):
                samples = list(np.linspace(0., dt, 9))
                if abs(result.jerk[i]) > 1e-9:
                    critical = -result.accel[i]/result.jerk[i]
                    if 0 < critical < dt:
                        samples.append(critical)
                for tau in samples:
                    t = time[i]+tau
                    s, v, a = result.sample(t)
                    raw_v = result.speed[i]+result.accel[i]*tau+.5*result.jerk[i]*tau**2
                    if (raw_v < -.003 or s > upper_s+.003
                            or not accel_bounds(t)[0]-.003 <= a <= accel_bounds(t)[1]+.003):
                        return empty('dense_validation_failed')
                    if v > qp_limit(s, t)+.12:
                        valid = False
            if valid:
                return result
            for i in range(1, m):
                upper[speed_rows[i]] = min(upper[speed_rows[i]],
                    float(qp_limit(result.s[i], time[i])))
            solver.update(u=np.array(upper))
        return empty('spatial_limit_refinement_failed')
