"""五次多项式插值 (从 planner/planner_utiles.py 迁移)"""

import numpy as np
from typing import List


def cal_quintic_coefficient(start_l: float, start_dl: float, start_ddl: float,
                             end_l: float, end_dl: float, end_ddl: float,
                             start_s: float, end_s: float) -> List[float]:
    """
    给定6个边界条件, 求解五次多项式系数.

    l(s) = a0 + a1·s + a2·s² + a3·s³ + a4·s⁴ + a5·s⁵

    求解线性方程组 B = A @ coeffi, A(6×6), B(6×1)

    Args:
        start_l, start_dl, start_ddl: 起点l, dl/ds, d²l/ds²
        end_l, end_dl, end_ddl:       终点l, dl/ds, d²l/ds²
        start_s, end_s:               起点/终点弧长
    Returns:
        [a0, a1, a2, a3, a4, a5]
    """
    A = np.array([
        [1, start_s, pow(start_s, 2), pow(start_s, 3), pow(start_s, 4), pow(start_s, 5)],
        [0, 1, 2 * start_s, 3 * pow(start_s, 2), 4 * pow(start_s, 3), 5 * pow(start_s, 4)],
        [0, 0, 2, 6 * start_s, 12 * pow(start_s, 2), 20 * pow(start_s, 3)],
        [1, end_s, pow(end_s, 2), pow(end_s, 3), pow(end_s, 4), pow(end_s, 5)],
        [0, 1, 2 * end_s, 3 * pow(end_s, 2), 4 * pow(end_s, 3), 5 * pow(end_s, 4)],
        [0, 0, 2, 6 * end_s, 12 * pow(end_s, 2), 20 * pow(end_s, 3)]
    ])
    B = np.array([start_l, start_dl, start_ddl, end_l, end_dl, end_ddl]).reshape((6, 1))
    coeffi = np.linalg.inv(A) @ B
    return list(coeffi.squeeze())


def evaluate_quintic(coeffi: List[float], s: np.ndarray):
    """
    计算五次多项式在给定s处的 l, dl, ddl, dddl

    Args:
        coeffi: 五次多项式系数 [a0,...,a5]
        s: 弧长数组 (np.ndarray)
    Returns:
        l, dl, ddl, dddl
    """
    l = (coeffi[0] + coeffi[1] * s + coeffi[2] * s**2 +
         coeffi[3] * s**3 + coeffi[4] * s**4 + coeffi[5] * s**5)
    dl = (coeffi[1] + 2 * coeffi[2] * s + 3 * coeffi[3] * s**2 +
          4 * coeffi[4] * s**3 + 5 * coeffi[5] * s**4)
    ddl = (2 * coeffi[2] + 6 * coeffi[3] * s + 12 * coeffi[4] * s**2 +
           20 * coeffi[5] * s**3)
    dddl = 6 * coeffi[3] + 24 * coeffi[4] * s + 60 * coeffi[5] * s**2
    return l, dl, ddl, dddl


def quintic_transition(ratio):
    """Zero-slope/zero-second-derivative blend on [0, 1].

    This is the normalized quintic solution for a spatial lateral transition;
    it does not claim minimum temporal jerk for varying vehicle speed.
    """
    u = np.clip(ratio, 0.0, 1.0)
    return u**3 * (10.0 + u * (-15.0 + 6.0*u))


def sample_quintic_path(reference, start_l, target_l, transition_distance_m,
                        sampling_resolution_m=0.5, *, lateral_bounds=None):
    """Densify a reference-relative quintic lateral path in local arc length.

    Keep reference vertices and the exact transition endpoint. Subdivide each
    reference segment to the requested maximum longitudinal spacing, evaluate
    the quintic at every sample and interpolate the corridor bounds there.
    Headings/curvatures are recomputed from the resulting Cartesian points.
    """
    import math
    from .geometry import cal_heading_kappa

    if (not math.isfinite(transition_distance_m) or transition_distance_m <= 0
            or not math.isfinite(sampling_resolution_m) or sampling_resolution_m <= 0):
        raise ValueError('transition distance and sampling resolution must be positive and finite')
    if lateral_bounds is not None and len(lateral_bounds) != len(reference):
        raise ValueError('corridor bounds must match reference points')
    if not reference:
        return []
    xy = []

    def append_sample(first, second, fraction, distance, left_index, right_index):
        heading_delta = math.atan2(math.sin(second[2]-first[2]), math.cos(second[2]-first[2]))
        heading = first[2]+fraction*heading_delta
        lateral = start_l+(target_l-start_l)*float(quintic_transition(distance/transition_distance_m))
        if lateral_bounds is not None:
            lower = (1-fraction)*lateral_bounds[left_index][0]+fraction*lateral_bounds[right_index][0]
            upper = (1-fraction)*lateral_bounds[left_index][1]+fraction*lateral_bounds[right_index][1]
            if lower > upper:
                raise ValueError('invalid drivable corridor bounds')
            lateral = max(lower, min(upper, lateral))
        x = first[0]+fraction*(second[0]-first[0])-lateral*math.sin(heading)
        y = first[1]+fraction*(second[1]-first[1])+lateral*math.cos(heading)
        if not xy or math.hypot(x-xy[-1][0], y-xy[-1][1]) > 1e-9:
            xy.append((x, y))

    append_sample(reference[0], reference[0], 0.0, 0.0, 0, 0)
    travelled = 0.0
    for index, (first, second) in enumerate(zip(reference[:-1], reference[1:])):
        length = math.hypot(second[0]-first[0], second[1]-first[1])
        if length <= 1e-9:
            continue
        steps = max(1, math.ceil(length/sampling_resolution_m))
        fractions = [step/steps for step in range(1, steps+1)]
        if travelled < transition_distance_m < travelled+length:
            endpoint = (transition_distance_m-travelled)/length
            if all(abs(fraction-endpoint) > 1e-12 for fraction in fractions):
                fractions.append(endpoint)
                fractions.sort()
        for fraction in fractions:
            append_sample(first, second, fraction, travelled+fraction*length, index, index+1)
        travelled += length
    headings, curvatures = cal_heading_kappa(xy)
    return [(x, y, heading, curvature)
            for (x, y), heading, curvature in zip(xy, headings, curvatures)]
