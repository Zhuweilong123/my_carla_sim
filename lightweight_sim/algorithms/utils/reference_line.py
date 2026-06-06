"""参考线平滑 — QP二次规划方法 (从 planner/planner_utiles.py 迁移)"""

import numpy as np
import cvxopt
from typing import List, Tuple
from .geometry import cal_heading_kappa


def smooth_reference_line(local_frenet_path_xy: List[Tuple],
                          w_cost_smooth: float = 0.4,
                          w_cost_length: float = 0.3,
                          w_cost_ref: float = 0.3,
                          x_thre: float = 0.2,
                          y_thre: float = 0.2) -> List[Tuple[float, float, float, float]]:
    """
    对原始参考线进行QP平滑处理.

    代价函数 (三项加权):
      J_smooth = Σ[(x_i - 2x_{i+1} + x_{i+2})² + (y_i - 2y_{i+1} + y_{i+2})²]  → 二阶平滑
      J_length = Σ[(x_i - x_{i+1})² + (y_i - y_{i+1})²]                         → 紧凑性
      J_ref    = Σ[(x_i - x_i_ref)² + (y_i - y_i_ref)²]                         → 几何相似

    约束: 每个点偏离原始位置不超过 (x_thre, y_thre).

    Args:
        local_frenet_path_xy: [(x0,y0), (x1,y1), ...] 或 [(x0,y0,theta0,kappa0), ...]
        w_cost_smooth, w_cost_length, w_cost_ref: 三项代价权重
        x_thre, y_thre: 偏差阈值 (m)
    Returns:
        [(x_opt0, y_opt0, theta_0, kappa_0), ...]
    """
    n = len(local_frenet_path_xy)
    if n < 3:
        # 太短无法平滑, 直接返回
        result = []
        for p in local_frenet_path_xy:
            if len(p) == 2:
                result.append((p[0], p[1], 0.0, 0.0))
            else:
                result.append(tuple(p[:4]))
        return result

    # 构建参考向量 x_ref = [x0, y0, x1, y1, ...]
    x_ref = np.zeros((2 * n, 1))
    lb = np.zeros((2 * n, 1))
    ub = np.zeros((2 * n, 1))
    for i in range(n):
        x_ref[2 * i] = local_frenet_path_xy[i][0]
        x_ref[2 * i + 1] = local_frenet_path_xy[i][1]
        lb[2 * i] = local_frenet_path_xy[i][0] - x_thre
        lb[2 * i + 1] = local_frenet_path_xy[i][1] - y_thre
        ub[2 * i] = local_frenet_path_xy[i][0] + x_thre
        ub[2 * i + 1] = local_frenet_path_xy[i][1] + y_thre

    # A1: 二阶平滑 (2n-4, 2n)
    A1 = np.zeros((2 * n - 4, 2 * n))
    for i in range(n - 2):
        A1[2 * i, 2 * i + 0] = 1
        A1[2 * i, 2 * i + 2] = -2
        A1[2 * i, 2 * i + 4] = 1
        A1[2 * i + 1, 2 * i + 1] = 1
        A1[2 * i + 1, 2 * i + 3] = -2
        A1[2 * i + 1, 2 * i + 5] = 1

    # A2: 紧凑性 (2n-2, 2n)
    A2 = np.zeros((2 * n - 2, 2 * n))
    for i in range(n - 1):
        A2[2 * i, 2 * i + 0] = 1
        A2[2 * i, 2 * i + 2] = -1
        A2[2 * i + 1, 2 * i + 1] = 1
        A2[2 * i + 1, 2 * i + 3] = -1

    A3 = np.identity(2 * n)

    H = 2 * (w_cost_smooth * A1.T @ A1 +
             w_cost_length * A2.T @ A2 +
             w_cost_ref * A3)
    f = -2 * w_cost_ref * x_ref

    G = np.concatenate((np.identity(2 * n), -np.identity(2 * n)))
    h = np.concatenate((ub, -lb))

    cvxopt.solvers.options['show_progress'] = False
    res = cvxopt.solvers.qp(cvxopt.matrix(H), cvxopt.matrix(f),
                            G=cvxopt.matrix(G), h=cvxopt.matrix(h))

    # 提取结果
    local_path_xy_opt = []
    for i in range(0, len(res['x']), 2):
        local_path_xy_opt.append((res['x'][i], res['x'][i + 1]))

    # 计算theta和kappa
    theta_list, kappa_list = cal_heading_kappa(local_path_xy_opt)
    result = []
    for i in range(len(local_path_xy_opt)):
        result.append((local_path_xy_opt[i][0], local_path_xy_opt[i][1],
                       theta_list[i], kappa_list[i]))
    return result
