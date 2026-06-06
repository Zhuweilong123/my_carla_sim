"""S-L图上的动态规划路径规划 (从 planner/motion_plan_path_planning.py 迁移)

去除了CARLA依赖, 改为纯numpy实现.
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from ..utils.quintic import cal_quintic_coefficient, evaluate_quintic


# =============================================================================
# 主入口
# =============================================================================

def DP_algorithm(obs_s_list: List[float], obs_l_list: List[float],
                 plan_start_s: float, plan_start_l: float,
                 plan_start_dl: float, plan_start_ddl: float,
                 sampling_res: float = 2.0,
                 w_collision_cost: float = 1e12,
                 w_smooth_cost: List[float] = None,
                 w_reference_cost: float = 20.0,
                 row: int = 12, col: int = 10,
                 sample_s: float = 8, sample_l: float = 1.0) -> Tuple[List[float], List[float]]:
    """
    动态规划在S-L图上搜索最优避障路径.

    Args:
        obs_s_list, obs_l_list: 障碍物的(s,l)坐标
        plan_start_s, plan_start_l: 规划起点
        plan_start_dl, plan_start_ddl: 规划起点的一二阶导数
        sampling_res: 增密分辨率 (m)
        w_collision_cost: 碰撞代价权重
        w_smooth_cost: 平滑代价 [w_dl, w_ddl, w_dddl]
        w_reference_cost: 参考线代价权重
        row, col: 采样网格行列数
        sample_s, sample_l: s/l方向采样间隔

    Returns:
        enriched_s_list, enriched_l_list: 增密后的路径 (分辨率=sampling_res)
    """
    if w_smooth_cost is None:
        w_smooth_cost = [300, 1000, 5000]

    has_obs = len(obs_s_list) > 0

    if has_obs:
        # DP 递推
        cost = np.ones((row, col)) * np.inf
        pre_node_index = (row >> 1) * np.ones((row, col), dtype="int32")

        # 起点到第一列的cost
        for i in range(row):
            cost[i, 0] = cal_start_cost(
                obs_s_list, obs_l_list,
                begin_s=plan_start_s, begin_l=plan_start_l,
                begin_dl=plan_start_dl, begin_ddl=plan_start_ddl,
                cur_node_row=i, row=row,
                sample_s=sample_s, sample_l=sample_l,
                w_cost_collision=w_collision_cost,
                w_cost_smooth=w_smooth_cost,
                w_cost_ref=w_reference_cost,
            )
            if i < (row >> 1):
                cost[i, 0] += 10000  # 左侧车道偏右行

        # 后续列
        for j in range(1, col):
            for i in range(row):
                cur_node_s = plan_start_s + (j + 1) * sample_s
                cur_node_l = ((row + 1) / 2 - 1 - i) * sample_l

                for k in range(row):
                    pre_node_s = plan_start_s + j * sample_s
                    pre_node_l = ((row + 1) / 2 - 1 - k) * sample_l

                    cost_neighbor = cal_neighbor_cost(
                        obs_s_list, obs_l_list,
                        pre_node_s, pre_node_l,
                        cur_node_s=cur_node_s, cur_node_l=cur_node_l,
                        sample_s=sample_s,
                        w_cost_collision=w_collision_cost,
                        w_cost_smooth=w_smooth_cost,
                        w_cost_ref=w_reference_cost,
                    )
                    cost_temp = cost[k, j - 1] + cost_neighbor
                    if i < (row >> 1):
                        cost_temp += 10000

                    if cost_temp < cost[i, j]:
                        cost[i, j] = cost_temp
                        pre_node_index[i, j] = k

        # 回溯
        DP_row_index_list = []
        if cost[:, -1].min() > w_collision_cost:
            print("[DP] WARNING: no collision-free path found")
        min_index = int(cost[:, -1].argmin())
        DP_row_index_list.append(min_index)
        for j in range(col - 1, 0, -1):
            min_index = int(pre_node_index[min_index, j])
            DP_row_index_list.append(min_index)
        DP_row_index_list.reverse()
    else:
        # 无障碍: 走直线 l=0
        DP_row_index_list = list(int((row + 1) / 2 - 1) * np.ones(col))

    # 索引 -> s,l
    DP_s_list, DP_l_list = [], []
    for i in range(len(DP_row_index_list)):
        DP_s_list.append(plan_start_s + (i + 1) * sample_s)
        DP_l_list.append(((row + 1) / 2 - 1 - DP_row_index_list[i]) * sample_l)

    # 增密
    enriched_s, enriched_l = enrich_DP_s_l(
        DP_s_list, DP_l_list,
        plan_start_s, plan_start_l, plan_start_dl, plan_start_ddl,
        resolution=sampling_res,
    )
    return enriched_s, enriched_l


# =============================================================================
# 路径增密
# =============================================================================

def enrich_DP_s_l(DP_s_list: List[float], DP_l_list: List[float],
                  plan_start_s: float, plan_start_l: float,
                  plan_start_dl: float, plan_start_ddl: float,
                  resolution: float = 1.0):
    """在DP采样点之间用五次多项式插值增密"""
    enriched_s_list, enriched_l_list = [], []

    # 规划起点 → 第一个DP点
    start_s, start_l, start_dl, start_ddl = plan_start_s, plan_start_l, plan_start_dl, plan_start_ddl
    end_s = DP_s_list[0]
    end_l = DP_l_list[0]
    coeffi = cal_quintic_coefficient(start_l, start_dl, start_ddl, end_l, 0, 0, start_s, end_s)

    s_vals = start_s + np.arange(0, int(end_s - start_s), resolution)
    l0, _, _, _ = evaluate_quintic(coeffi, s_vals)
    enriched_s_list.extend(list(s_vals))
    enriched_l_list.extend(list(l0))

    # 中间DP点之间
    for i in range(1, len(DP_s_list)):
        start_s = DP_s_list[i - 1]
        start_l = DP_l_list[i - 1]
        end_s = DP_s_list[i]
        end_l = DP_l_list[i]
        coeffi = cal_quintic_coefficient(start_l, 0, 0, end_l, 0, 0, start_s, end_s)

        s_vals = start_s + np.arange(0, int(end_s - start_s), resolution)
        li, _, _, _ = evaluate_quintic(coeffi, s_vals)
        enriched_s_list.extend(list(s_vals))
        enriched_l_list.extend(list(li))

    enriched_s_list.append(end_s)
    enriched_l_list.append(end_l)

    return enriched_s_list, enriched_l_list


# =============================================================================
# 代价函数
# =============================================================================

def cal_start_cost(obs_s_list, obs_l_list,
                   begin_s, begin_l, begin_dl, begin_ddl,
                   cur_node_row, row, sample_s, sample_l,
                   w_cost_collision, w_cost_smooth, w_cost_ref):
    """计算起点到第一列的代价"""
    start_l, start_dl, start_ddl = begin_l, begin_dl, begin_ddl
    start_s = begin_s
    end_l = ((row + 1) / 2 - 1 - cur_node_row) * sample_l
    end_s = begin_s + sample_s

    coeffi = cal_quintic_coefficient(start_l, start_dl, start_ddl, end_l, 0, 0, start_s, end_s)

    s = np.zeros((10, 1))
    for i in range(10):
        s[i, 0] = start_s + i * sample_s / 10

    l, dl, ddl, dddl = evaluate_quintic(coeffi, s)
    cost_smooth = float(w_cost_smooth[0] * (dl.T @ dl).item() +
                        w_cost_smooth[1] * (ddl.T @ ddl).item() +
                        w_cost_smooth[2] * (dddl.T @ dddl).item())
    cost_ref = float(w_cost_ref * (l.T @ l).item())
    cost_collision = _calc_collision(obs_s_list, obs_l_list, s, l, w_cost_collision)

    return float(cost_smooth + cost_collision + cost_ref)


def cal_neighbor_cost(obs_s_list, obs_l_list,
                      pre_node_s, pre_node_l,
                      cur_node_s, cur_node_l, sample_s,
                      w_cost_collision, w_cost_smooth, w_cost_ref):
    """计算相邻列节点之间的代价"""
    start_l, start_s = pre_node_l, pre_node_s
    end_l, end_s = cur_node_l, cur_node_s

    coeffi = cal_quintic_coefficient(start_l, 0, 0, end_l, 0, 0, start_s, end_s)

    s = np.zeros((10, 1))
    for i in range(10):
        s[i, 0] = start_s + i * sample_s / 10

    l, dl, ddl, dddl = evaluate_quintic(coeffi, s)
    cost_smooth = float(w_cost_smooth[0] * (dl.T @ dl).item() +
                        w_cost_smooth[1] * (ddl.T @ ddl).item() +
                        w_cost_smooth[2] * (dddl.T @ dddl).item())
    cost_ref = float(w_cost_ref * (l.T @ l).item())
    cost_collision = _calc_collision(obs_s_list, obs_l_list, s, l, w_cost_collision)

    return float(cost_smooth + cost_collision + cost_ref)


def _calc_collision(obs_s_list, obs_l_list, s, l, w_cost_collision,
                    danger_dis=4.0, safe_dis=6.0):
    """计算障碍物碰撞代价"""
    cost = 0.0
    if len(obs_s_list) == 0:
        return cost
    for i in range(len(obs_s_list)):
        d_lon = obs_s_list[i] - s
        d_lat = obs_l_list[i] - l
        square_d = d_lon**2 + d_lat**2
        for sd in square_d.squeeze():
            sd = float(sd)
            if sd <= danger_dis**2:
                cost += w_cost_collision
                return cost
            elif danger_dis**2 < sd < safe_dis**2:
                cost += 5000.0 / (sd + 1e-6)
    return cost
