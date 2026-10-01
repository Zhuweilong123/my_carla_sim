"""Box-constrained positive-definite QP solvers for the MPC steering sequence."""

import numpy as np


def solve_box_qp(H, f, bound, *, backend='auto'):
    """Minimize 0.5*u.T@H@u + f.T@u subject to |u| <= bound.

    The NumPy primal active-set backend solves the same QP as cvxopt; it is
    not a controller fallback. Only a missing optional cvxopt dependency
    triggers automatic backend selection. Solver failures are propagated.
    """
    H = np.asarray(H, dtype=float)
    f = np.asarray(f, dtype=float).reshape(-1)
    if (H.shape != (f.size, f.size) or f.size == 0
            or not np.all(np.isfinite(H)) or not np.all(np.isfinite(f))
            or not np.allclose(H, H.T) or not np.isfinite(bound) or bound <= 0):
        raise ValueError('invalid box QP data')
    if backend not in ('auto', 'numpy', 'cvxopt'):
        raise ValueError('unknown MPC QP backend')
    np.linalg.cholesky(H)
    if backend in ('auto', 'cvxopt'):
        try:
            from cvxopt import matrix, solvers
        except ImportError:
            if backend == 'cvxopt':
                raise RuntimeError('cvxopt backend requested but cvxopt is not installed')
        else:
            count = f.size
            result = solvers.qp(matrix(H), matrix(f),
                                matrix(np.vstack((np.eye(count), -np.eye(count)))),
                                matrix(np.full(2*count, bound)),
                                options={'show_progress': False, 'abstol': 1e-9,
                                         'reltol': 1e-9, 'feastol': 1e-9})
            if result['status'] != 'optimal':
                raise RuntimeError('MPC QP failed: ' + result['status'])
            solution = np.asarray(result['x']).reshape(-1)
            if not np.all(np.isfinite(solution)) or np.any(abs(solution) > bound+1e-7):
                raise RuntimeError('MPC QP returned invalid steering commands')
            return np.clip(solution, -bound, bound), 'cvxopt'

    # Start feasible, then optimize the free variables and release bounds
    # whose multipliers violate KKT. Each step stops at the first new bound.
    u = np.clip(np.linalg.solve(H, -f), -bound, bound)
    active = np.zeros(f.size, dtype=int)
    active[u <= -bound+1e-12] = -1
    active[u >= bound-1e-12] = 1
    tolerance = 1e-9 * max(1.0, np.max(abs(f)), np.linalg.norm(H, np.inf)*bound)
    for _ in range(max(100, 20*f.size)):
        gradient = H @ u + f
        free = active == 0
        direction = np.zeros_like(u)
        if np.any(free):
            direction[free] = np.linalg.solve(H[np.ix_(free, free)], -gradient[free])
        if np.max(abs(direction)) <= 1e-12:
            violation = np.where(active == -1, -gradient,
                                 np.where(active == 1, gradient, 0.0))
            if np.max(violation) <= tolerance and np.max(abs(gradient[free]), initial=0) <= tolerance:
                return u, 'numpy'
            active[np.argmax(violation)] = 0
            continue
        alpha, blocker = 1.0, None
        for j in np.flatnonzero(free):
            if abs(direction[j]) <= 1e-14:
                continue
            edge = bound if direction[j] > 0 else -bound
            distance = (edge-u[j])/direction[j]
            if distance < alpha:
                alpha, blocker = max(0.0, distance), j
        u = np.clip(u+alpha*direction, -bound, bound)
        if blocker is not None:
            active[blocker] = 1 if direction[blocker] > 0 else -1
    raise RuntimeError('MPC box QP did not converge')
