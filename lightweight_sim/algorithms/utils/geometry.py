"""航向角与曲率的数值计算 (从 planner/planner_utiles.py 迁移, 去除CARLA依赖)"""

import math
import numpy as np
from typing import List, Tuple


def cal_heading_kappa(frenet_path_xy_list: List[Tuple[float, float]]):
    """
    计算frenet曲线上每个点的切向角theta和曲率kappa

    原理:
      theta = arctan(dy/dx)
      kappa = d_theta / d_s,  d_s = sqrt(dx² + dy²)
    采用中点欧拉法计算每个点处的导数, O(h²)精度.

    Args:
        frenet_path_xy_list: [(x0,y0), (x1,y1), ...]
    Returns:
        theta_list, kappa_list (均为弧度制)
    """
    if len(frenet_path_xy_list) < 2:
        return [0.0], [0.0]

    dx_ = []
    dy_ = []
    for i in range(len(frenet_path_xy_list) - 1):
        dx_.append(frenet_path_xy_list[i + 1][0] - frenet_path_xy_list[i][0])
        dy_.append(frenet_path_xy_list[i + 1][1] - frenet_path_xy_list[i][1])

    # n个点的差分得到n-1个结果, 对首尾做延拓, 取前后差分的均值
    dx_pre = [dx_[0]] + dx_
    dx_aft = dx_ + [dx_[-1]]
    dx = (np.array(dx_pre) + np.array(dx_aft)) / 2

    dy_pre = [dy_[0]] + dy_
    dy_aft = dy_ + [dy_[-1]]
    dy = (np.array(dy_pre) + np.array(dy_aft)) / 2

    theta = np.arctan2(dy, dx)  # 限制在 (-pi, pi)

    # 曲率计算: kappa = d_theta/ds
    d_theta_ = np.diff(theta)
    d_theta_pre = np.insert(d_theta_, 0, d_theta_[0])
    d_theta_aft = np.insert(d_theta_, -1, d_theta_[-1])
    # 用sin(d_theta)近似d_theta, 避免角度多值性
    d_theta = np.sin((d_theta_pre + d_theta_aft) / 2)
    ds = np.sqrt(dx**2 + dy**2)
    kappa = d_theta / (ds + np.finfo(float).eps)

    return list(theta), list(kappa)


def cal_heading_kappa_from_xy_theta(xy_theta_list: List[Tuple[float, float, float]]):
    """
    从带朝向的点列表计算曲率 (输入已含theta)
    """
    theta_list = [p[2] for p in xy_theta_list]
    xy_list = [(p[0], p[1]) for p in xy_theta_list]
    _, kappa_list = cal_heading_kappa(xy_list)
    # 用原有theta替换
    kappa_from_theta = []
    for i in range(1, len(theta_list)):
        ds = math.hypot(xy_list[i][0] - xy_list[i-1][0],
                        xy_list[i][1] - xy_list[i-1][1]) + np.finfo(float).eps
        dtheta = math.sin(theta_list[i] - theta_list[i-1])
        kappa_from_theta.append(dtheta / ds)
    kappa_from_theta.append(kappa_from_theta[-1] if kappa_from_theta else 0.0)
    return theta_list, kappa_from_theta
