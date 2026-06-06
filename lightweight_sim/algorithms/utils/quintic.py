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
