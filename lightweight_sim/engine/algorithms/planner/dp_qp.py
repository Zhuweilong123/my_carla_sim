"""Corridor lattice DP followed by a constrained, piecewise-jerk path QP.

Derivatives and jerk are with respect to arc length, not time. Obstacles
are treated as static footprints; longitudinal prediction is a separate task.
"""
from __future__ import annotations

import numpy as np


def quintic_edge(length, start, end, distances):
    """Evaluate l, dl, ddl on a numerically scaled Hermite segment."""
    l, dl, ddl = start
    end_l, end_dl, end_ddl = (end, 0., 0.) if np.isscalar(end) else end
    c = np.array([l, dl * length, ddl * length**2 / 2, 0., 0., 0.])
    c[3:] = np.linalg.solve(
        [[1, 1, 1], [3, 4, 5], [6, 12, 20]],
        [end_l - sum(c[:3]), end_dl*length-c[1] - 2*c[2], end_ddl*length**2-2*c[2]],
    )
    u = np.asarray(distances) / length
    return tuple(np.polynomial.polynomial.polyval(u, v) / length**order
                 for order, v in enumerate((c, np.arange(1, 6)*c[1:],
                                            np.arange(2, 6)*np.arange(1, 5)*c[2:])))


def DP_algorithm(stations, lateral_samples, start, preferred, edge_is_safe,
                 resolution=0.5, edge_cost=None):
    """Search the full corridor lattice; unreachable terminal states fail closed."""
    states = [np.asarray([start], dtype=float)]
    costs = np.zeros(1)
    parents_by_layer = []
    for column in range(1, len(stations)):
        ds = stations[column]-stations[column-1]
        offsets = np.linspace(0., ds, max(2, int(np.ceil(ds/resolution))+1))
        slopes = (0.,) if column == len(stations)-1 else (-.2, 0., .2)
        targets = np.asarray([(l, dl, 0.) for l in lateral_samples[column] for dl in slopes])
        endpoint_safe = edge_is_safe(np.asarray([stations[column]]),
                                     targets[:, 0, None], targets[:, 1, None], targets[:, 2, None])
        targets = targets[endpoint_safe]
        if not len(targets): return None
        previous = states[-1]
        reachable_pair = abs(previous[:, 0, None]-targets[None, :, 0]) <= np.tan(.35)*ds+1e-6
        parent_index, target_index = np.nonzero(reachable_pair)
        if not len(parent_index): return None
        p_l, p_dl, p_ddl = (previous[parent_index, i, None] for i in range(3))
        e_l, e_dl, e_ddl = (targets[target_index, i, None] for i in range(3))
        c1, c2 = p_dl*ds, p_ddl*ds*ds/2
        delta, end_d, end_dd = e_l-p_l, e_dl*ds, e_ddl*ds*ds
        c3 = 10*delta-6*c1-3*c2-4*end_d+end_dd/2
        c4 = -15*delta+8*c1+3*c2+7*end_d-end_dd
        c5 = 6*delta-3*c1-c2-3*end_d+end_dd/2
        u = offsets[None, :]/ds
        l = p_l+c1*u+c2*u*u+c3*u**3+c4*u**4+c5*u**5
        dl = (c1+2*c2*u+3*c3*u*u+4*c4*u**3+5*c5*u**4)/ds
        ddl = (2*c2+6*c3*u+12*c4*u*u+20*c5*u**3)/ds**2
        valid = edge_is_safe(stations[column-1]+offsets, l, dl, ddl)
        pair_cost = costs[parent_index]+ds*np.mean((l-preferred)**2+20*dl**2+100*ddl**2, axis=-1)
        if edge_cost is not None:
            pair_cost += ds*np.mean(edge_cost(stations[column-1]+offsets, l), axis=-1)
        trial = np.full((len(previous), len(targets)), np.inf)
        trial[parent_index, target_index] = np.where(valid, pair_cost, np.inf)
        parents = np.argmin(trial, axis=0)
        costs = trial[parents, np.arange(len(targets))]
        reachable = np.isfinite(costs)
        if not np.any(reachable):
            return None
        states.append(targets[reachable])
        parents_by_layer.append(parents[reachable])
        costs = costs[reachable]
    parent = int(np.argmin(costs))
    result = [states[-1][parent]]
    for i in range(len(parents_by_layer)-1, -1, -1):
        parent = parents_by_layer[i][parent]
        result.append(states[i][parent])
    return np.asarray(result[::-1])


def jerk_matrices(knots, samples, start):
    """Exact triple integration of constant jerk per interval (C2 path)."""
    t = np.asarray(samples)
    left, right = np.asarray(knots[:-1]), np.asarray(knots[1:])
    a = np.maximum(t[:, None]-left, 0.)
    b = np.maximum(t[:, None]-right, 0.)
    matrices = ((a**3-b**3)/6, (a**2-b**2)/2, a-b)
    l, dl, ddl = start
    constants = (l+dl*t+ddl*t*t/2, dl+ddl*t, np.full(len(t), ddl))
    return matrices, constants


def adaptive_dp_knots(length, coarse_step, obstacle_stations=()):
    values = [0.]
    while values[-1] < length-1e-8:
        t = values[-1]
        step = min(coarse_step, 4. if t < 12. else
                   4. if any(abs(t-obstacle)<12. for obstacle in obstacle_stations)
                   else coarse_step)
        values.append(min(length, t+step))
    return np.asarray(values)


def adaptive_qp_knots(length, coarse_step, obstacle_stations=()):
    """Fine start mesh preserves initial derivatives; refine obstacle regions."""
    values = [0.]
    while values[-1] < length-1e-8:
        station = values[-1]
        step = min(coarse_step, .5 if station < 12. else
                   1. if any(abs(station-obstacle) < 12. for obstacle in obstacle_stations)
                   else coarse_step)
        values.append(min(length, station+step))
    return np.asarray(values)


def Quadratic_planning(knots, samples, start, dp_l, lower, upper, preferred,
                       road_limits=None, half_length=0., max_slope=None,
                       time_limit_s=.08, diagnostics=None, footprint_constraints=(), max_second_derivative=.08):
    """Sparse state QP, exact C2 dynamics, bounded OSQP iterations/runtime.

    Keeping l/dl/ddl explicitly avoids the ill-conditioned cubic integration
    matrix over the entire horizon. Infeasibility detection is part of OSQP;
    an unsuccessful or constraint-violating solve never produces a path.
    """
    import osqp
    from scipy import sparse
    n = len(knots)
    size = 3*n+n-1
    h = np.diff(knots)
    interval = np.clip(np.searchsorted(knots, samples, side='right')-1, 0, n-2)
    t = samples-np.asarray(knots)[interval]
    rows = np.arange(len(samples))
    def evaluation(order):
        blocks = ((np.ones(len(t)), t, t*t/2, t**3/6),
                  (np.zeros(len(t)), np.ones(len(t)), t, t*t/2),
                  (np.zeros(len(t)), np.zeros(len(t)), np.ones(len(t)), t))[order]
        columns = (3*interval, 3*interval+1, 3*interval+2, 3*n+interval)
        return sparse.coo_matrix((np.concatenate(blocks),
            (np.tile(rows, 4), np.concatenate(columns))), shape=(len(t), size)).tocsc()
    L, D, A = (evaluation(i) for i in range(3))
    J = sparse.coo_matrix((np.ones(n-1), (np.arange(n-1), 3*n+np.arange(n-1))),
                         shape=(n-1, size)).tocsc()
    P = (10.1*L.T@L + 20*D.T@D + 100*A.T@A + 10*J.T@sparse.diags(h)@J)
    q = np.asarray(-L.T@(10*dp_l+.1*preferred)).ravel()
    # Exact state integration with a constant jerk in each interval.
    erows, ecols, data = [], [], []
    for i, distance in enumerate(h):
        entries = (([(3*(i+1), 1), (3*i, -1), (3*i+1, -distance),
                     (3*i+2, -distance**2/2), (3*n+i, -distance**3/6)]),
                   ([(3*(i+1)+1, 1), (3*i+1, -1), (3*i+2, -distance), (3*n+i, -distance**2/2)]),
                   ([(3*(i+1)+2, 1), (3*i+2, -1), (3*n+i, -distance)]))
        for derivative, entries_for_row in enumerate(entries):
            for column, value in entries_for_row:
                erows.append(3*i+derivative); ecols.append(column); data.append(value)
    E = sparse.coo_matrix((data, (erows, ecols)), shape=(3*(n-1), size)).tocsc()
    initial = sparse.coo_matrix((np.ones(3), (np.arange(3), np.arange(3))), shape=(3, size)).tocsc()
    matrices = [E, initial, L, D]
    slope = np.tan(.35) if max_slope is None else max_slope
    lows = [np.zeros(E.shape[0]), np.asarray(start), lower, np.broadcast_to(-slope, len(samples))]
    highs = [np.zeros(E.shape[0]), np.asarray(start), upper, np.broadcast_to(slope, len(samples))]
    if road_limits is not None:
        for sign in (-1., 1.):
            matrices.append(L+sign*half_length*D)
            lows.append(road_limits[0]); highs.append(road_limits[1])
    for offset, footprint_lower, footprint_upper in footprint_constraints:
        matrices.append(L+offset*D)
        lows.append(footprint_lower); highs.append(footprint_upper)
    matrices.append(A)
    lows.append(np.full(len(samples), -max_second_derivative))
    highs.append(np.full(len(samples), max_second_derivative))
    C = sparse.vstack(matrices, format='csc')
    lb, ub = np.concatenate(lows), np.concatenate(highs)
    if np.any(lb > ub) or not np.all(np.isfinite(q)):
        if diagnostics is not None: diagnostics['status'] = 'invalid_bounds'
        return None
    solver = osqp.OSQP(algebra='builtin')
    solver.setup(P=sparse.triu(P*.01, format='csc'), q=q*.01, A=C, l=lb, u=ub,
                 verbose=False, eps_abs=1e-5, eps_rel=1e-5, max_iter=4000,
                 time_limit=time_limit_s, polishing=True, check_termination=10)
    seed_l = np.interp(knots, samples, dp_l)
    seed = np.zeros(size)
    seed[:3*n:3] = seed_l
    seed[1:3*n:3] = np.gradient(seed_l, knots)
    seed[2:3*n:3] = np.gradient(seed[1:3*n:3], knots)
    seed[:3] = start
    solver.warm_start(x=seed)
    result = solver.solve(raise_error=False)
    if diagnostics is not None:
        diagnostics.update(status=result.info.status, iterations=result.info.iter,
                           solve_time_s=result.info.run_time, variables=size)
    if result.info.status_val not in (1, 2) or result.x is None or not np.all(np.isfinite(result.x)):
        return None
    actual = C@result.x
    if np.any(actual < lb-1e-5) or np.any(actual > ub+1e-5):
        if diagnostics is not None: diagnostics['status'] = 'constraint_residual'
        return None
    return L@result.x, D@result.x, A@result.x
