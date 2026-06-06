"""二次规划路径优化 + SL→XY坐标逆变换 (从 planner/motion_plan_path_planning.py 迁移)"""

import math
import time
import numpy as np
import cvxopt
from typing import List, Tuple
from ..utils.quintic import cal_quintic_coefficient
from ..utils.geometry import cal_heading_kappa
from ..utils.reference_line import smooth_reference_line


def Quadratic_planning(l_min: List[float], l_max: List[float],
                       plan_start_l: float, plan_start_dl: float, plan_start_ddl: float,
                       dp_sampling_res: float = 2.0,
                       w_cost_l: float = 1000, w_cost_dl: float = 10000,
                       w_cost_ddl: float = 3000, w_cost_dddl: float = 150,
                       w_cost_centre: float = 250,
                       w_cost_end_l: float = 40, w_cost_end_dl: float = 40,
                       w_cost_end_ddl: float = 40,
                       host_d1: float = 3, host_d2: float = 3, host_w: float = 3,
                       target_end_l: float = None):
    """
    二次规划实现平滑避障路径.

    Args:
        l_min, l_max: 每个s点的横向边界
        plan_start_l, dl, ddl: 规划起点状态
        dp_sampling_res: DP采样分辨率
        w_cost_*: 各项代价权重
        host_d1/d2: 质心到车头/尾距离 (m)
        host_w: 车宽 (m)

    Returns:
        qp_path_l, qp_path_dl, qp_path_ddl
    """
    n = len(l_min)
    if n < 3:
        return [plan_start_l] * n, [0.0] * n, [0.0] * n

    ds = dp_sampling_res

    # ---- 等式约束: 三阶连续性 ----
    Aeq = np.zeros((2 * n - 2, 3 * n))
    Aeq_sub = np.array([[1, ds, ds**2 / 2, -1, 0, ds**2 / 6],
                        [0, 1, ds / 2, 0, -1, ds / 2]])
    for i in range(n - 1):
        Aeq[i * 2: i * 2 + 2, i * 3:i * 3 + 6] = Aeq_sub
    beq = np.zeros((2 * n - 2, 1))

    # ---- 不等式约束: 车辆矩形形状 ----
    A_ineq = np.zeros((8 * n, 3 * n))
    b_ineq = np.zeros((8 * n, 1))
    A_sub = np.array([[1, host_d1, 0], [1, host_d1, 0],
                      [1, -host_d2, 0], [1, -host_d2, 0],
                      [-1, -host_d1, 0], [-1, -host_d1, 0],
                      [-1, host_d2, 0], [-1, host_d2, 0]])

    front_index = math.ceil(host_d1 / ds)
    back_index = math.ceil(host_d2 / ds)
    for i in range(n):
        A_ineq[8 * i:8 * i + 8, 3 * i:3 * i + 3] = A_sub
        idx1 = min(i + front_index, n - 1)
        idx2 = max(i - back_index, 0)
        b_sub = np.array([[l_max[idx1] - host_w / 2], [l_max[idx1] + host_w / 2],
                          [l_max[idx1] - host_w / 2], [l_max[idx1] + host_w / 2],
                          [-l_min[idx2] + host_w / 2], [-l_min[idx2] - host_w / 2],
                          [-l_min[idx2] + host_w / 2], [-l_min[idx2] - host_w / 2]])
        b_ineq[8 * i:8 * (i + 1), 0] = b_sub.squeeze()

    # ---- 起点/终点约束 ----
    lb = np.ones((3 * n, 1)) * (-100000)
    ub = np.ones((3 * n, 1)) * 100000
    lb[0], lb[1], lb[2] = plan_start_l, plan_start_dl, plan_start_ddl
    ub[0], ub[1], ub[2] = plan_start_l, plan_start_dl, plan_start_ddl
    # 终点: 回到原始车道偏移 (默认0=道路中心线, 由调用方传入车道偏移量)
    end_l = target_end_l if target_end_l is not None else 0.0
    lb[3 * n - 3] = ub[3 * n - 3] = end_l  # 终点l
    lb[3 * n - 2] = ub[3 * n - 2] = 0       # 终点dl=0
    lb[3 * n - 1] = ub[3 * n - 1] = 0       # 终点ddl=0

    A_bound = np.concatenate((np.identity(3 * n), -np.identity(3 * n)))
    b_bound = np.concatenate((ub, -lb))

    G = np.concatenate((A_ineq, A_bound))
    h = np.concatenate((b_ineq, b_bound))

    # ---- H矩阵 ----
    H_L = np.zeros((3 * n, 3 * n))
    H_DL = np.zeros((3 * n, 3 * n))
    H_DDL = np.zeros((3 * n, 3 * n))
    for i in range(n):
        H_L[3 * i, 3 * i] = 1
        H_DL[3 * i + 1, 3 * i + 1] = 1
        H_DDL[3 * i + 2, 3 * i + 2] = 1
    H_CENTRE = H_L

    # 三阶平滑矩阵
    H_DDDL = np.zeros((n - 1, 3 * n))
    H_dddl_sub = np.array([[0, 0, -1, 0, 0, 1]])
    for i in range(n - 1):
        H_DDDL[i, 3 * i:3 * i + 6] = H_dddl_sub

    # 终点正则化
    H_L_END = np.zeros((3 * n, 3 * n))
    H_DL_END = np.zeros((3 * n, 3 * n))
    H_DDL_END = np.zeros((3 * n, 3 * n))
    H_L_END[3 * n - 3, 3 * n - 3] = 1
    H_DL_END[3 * n - 2, 3 * n - 2] = 1
    H_DDL_END[3 * n - 1, 3 * n - 1] = 1

    H = (w_cost_l * (H_L.T @ H_L) + w_cost_dl * (H_DL.T @ H_DL) +
         w_cost_ddl * (H_DDL.T @ H_DDL) + w_cost_dddl * (H_DDDL.T @ H_DDDL) +
         w_cost_centre * (H_CENTRE.T @ H_CENTRE) +
         w_cost_end_l * (H_L_END.T @ H_L_END) +
         w_cost_end_dl * (H_DL_END.T @ H_DL_END) +
         w_cost_end_ddl * (H_DDL_END.T @ H_DDL_END))
    H = 2 * H

    # ---- f向量 (凸空间中心) ----
    f = np.zeros((3 * n, 1))
    centre_line = (np.array(l_min) + np.array(l_max)) / 2
    for i in range(n):
        f[3 * i] = -2 * centre_line[i]
    f = w_cost_centre * f

    # ---- 求解 ----
    cvxopt.solvers.options['show_progress'] = False
    try:
        res = cvxopt.solvers.qp(
            cvxopt.matrix(H), cvxopt.matrix(f),
            G=cvxopt.matrix(G), h=cvxopt.matrix(h),
            A=cvxopt.matrix(Aeq), b=cvxopt.matrix(beq),
        )
        qp_path_l = list(res['x'][0::3])
        qp_path_dl = list(res['x'][1::3])
        qp_path_ddl = list(res['x'][2::3])
    except Exception as e:
        print(f"[QP] solver failed: {e}, falling back to DP path")
        qp_path_l = [plan_start_l] * n
        qp_path_dl = [0.0] * n
        qp_path_ddl = [0.0] * n

    return qp_path_l, qp_path_dl, qp_path_ddl


def cal_lmin_lmax(dp_path_s: List[float], dp_path_l: List[float],
                  obs_s_list: List[float], obs_l_list: List[float],
                  obs_length: float = 5, obs_width: float = 5):
    """根据DP路径和障碍物位置计算QP的边界约束"""
    lmin = -6 * np.ones(len(dp_path_s))
    lmax = 6 * np.ones(len(dp_path_s))

    for i in range(len(obs_s_list)):
        obs_s_min = obs_s_list[i] - obs_length / 2
        obs_s_max = obs_s_list[i] + obs_length / 2
        obs_s_min_idx = np.argmin(np.abs(np.array(dp_path_s) - obs_s_min)) + 1
        obs_s_max_idx = np.argmin(np.abs(np.array(dp_path_s) - obs_s_max)) + 1

        centre_idx = np.argmin(np.abs(np.array(dp_path_s) - obs_s_list[i]))
        path_l = dp_path_l[centre_idx]

        if path_l < obs_l_list[i]:
            # 障碍物在规划路径右侧 → 限制lmax
            for j in range(obs_s_min_idx, min(obs_s_max_idx + 1, len(lmax))):
                lmax[j] = min(lmax[j], obs_l_list[i] - obs_width / 2)
        else:
            # 障碍物在规划路径左侧 → 限制lmin
            for j in range(obs_s_min_idx, min(obs_s_max_idx + 1, len(lmin))):
                lmin[j] = max(lmin[j], obs_l_list[i] + obs_width / 2)

    return list(lmin), list(lmax)


def frenet_2_x_y_theta_kappa(plan_start_s: float, plan_start_l: float,
                              enriched_s_list: List[float],
                              enriched_l_list: List[float],
                              frenet_path_opt: List[Tuple[float, float, float, float]],
                              s_map: List[float]) -> List[Tuple[float, float, float, float]]:
    """
    将S-L路径转换为笛卡尔坐标系下的 (x, y, theta, kappa).
    """
    target_xy = []

    # 规划起点
    proj_x, proj_y, proj_theta, proj_kappa, pre_idx = \
        _cal_proj_point(plan_start_s, 0, frenet_path_opt, s_map)
    nor_v = np.array([-math.sin(proj_theta), math.cos(proj_theta)])
    cur_x, cur_y = np.array([proj_x, proj_y]) + plan_start_l * nor_v
    target_xy.append((cur_x, cur_y))

    for i in range(len(enriched_l_list)):
        cur_s = enriched_s_list[i]
        cur_l = enriched_l_list[i]
        if cur_s > s_map[-1]:
            break
        proj_x, proj_y, proj_theta, proj_kappa, pre_idx = \
            _cal_proj_point(cur_s, pre_idx, frenet_path_opt, s_map)
        nor_v = np.array([-math.sin(proj_theta), math.cos(proj_theta)])
        cur_x, cur_y = np.array([proj_x, proj_y]) + cur_l * nor_v
        target_xy.append((cur_x, cur_y))

    # 平滑并计算theta, kappa
    if len(target_xy) >= 3:
        target_path = smooth_reference_line(target_xy,
                                            w_cost_smooth=0.5,
                                            w_cost_length=0.2,
                                            w_cost_ref=0.3)
    else:
        target_path = [(p[0], p[1], 0.0, 0.0) for p in target_xy]

    return target_path


def _cal_proj_point(s: float, pre_match_index: int,
                    frenet_path_opt: List[Tuple[float, float, float, float]],
                    s_map: List[float]):
    """确定给定s在参考线上的投影点"""
    idx = pre_match_index
    s_map_len = len(s_map)
    while idx + 1 < s_map_len and s_map[idx + 1] < s:
        idx += 1
    idx = min(idx, s_map_len - 1)

    mp_x, mp_y, mp_theta, mp_kappa = frenet_path_opt[idx]
    ds = s - s_map[idx]
    mp_tou_v = np.array([math.cos(mp_theta), math.sin(mp_theta)])
    r_m = np.array([mp_x, mp_y])
    proj_x, proj_y = r_m + ds * mp_tou_v
    proj_theta = mp_theta + mp_kappa * ds
    proj_kappa = mp_kappa
    return proj_x, proj_y, proj_theta, proj_kappa, idx
