"""Frenet坐标变换 (从 planner/planner_utiles.py 迁移, 去除CARLA依赖)"""

import math
import numpy as np
from typing import List, Tuple


def find_match_points(xy_list: List[Tuple[float, float]],
                      frenet_path_node_list: List[Tuple[float, float, float, float]],
                      is_first_run: bool = True,
                      pre_match_index: int = 0):
    """
    计算笛卡尔坐标位置在frenet参考线上的匹配点索引和投影点信息.

    Args:
        xy_list: [(x0,y0), (x1,y1), ...] 待投影点
        frenet_path_node_list: [(x,y,theta,kappa), ...] 参考线
        is_first_run: 是否首次匹配 (决定搜索方向)
        pre_match_index: 上次匹配点索引
    Returns:
        match_point_index_list: 匹配点索引列表
        project_node_list: [(x_p, y_p, theta_p, kappa_p), ...] 投影点信息
    """
    input_xy_length = len(xy_list)
    frenet_path_length = len(frenet_path_node_list)
    match_point_index_list = np.zeros(input_xy_length, dtype="int32")
    project_node_list = []

    for index_xy in range(input_xy_length):
        x, y = xy_list[index_xy]

        if is_first_run:
            start_index = 0
            increase_count = 0
            min_distance = float("inf")
            for i in range(start_index, frenet_path_length):
                fx, fy, _, _ = frenet_path_node_list[i]
                distance = math.hypot(fx - x, fy - y)
                if distance < min_distance:
                    min_distance = distance
                    match_point_index_list[index_xy] = i
                    increase_count = 0
                else:
                    increase_count += 1
                    if increase_count >= 50:
                        break
        else:
            start_index = pre_match_index
            increase_count = 0
            pre_x, pre_y, pre_theta, _ = frenet_path_node_list[start_index]
            pre_direction = np.array([np.cos(pre_theta), np.sin(pre_theta)])
            pre_to_xy = np.array([x - pre_x, y - pre_y])
            flag = np.dot(pre_to_xy, pre_direction)

            min_distance = float("inf")
            if flag > 0:
                for i in range(start_index, frenet_path_length):
                    fx, fy, _, _ = frenet_path_node_list[i]
                    distance = math.hypot(fx - x, fy - y)
                    if distance < min_distance:
                        min_distance = distance
                        match_point_index_list[index_xy] = i
                        increase_count = 0
                    else:
                        increase_count += 1
                        if increase_count >= 5:
                            break
            else:
                for i in range(start_index, -1, -1):
                    fx, fy, _, _ = frenet_path_node_list[i]
                    distance = math.hypot(fx - x, fy - y)
                    if distance < min_distance:
                        min_distance = distance
                        match_point_index_list[index_xy] = i
                        increase_count = 0
                    else:
                        increase_count += 1
                        if increase_count >= 5:
                            break

        # 计算投影点
        match_idx = match_point_index_list[index_xy]
        x_m, y_m, theta_m, k_m = frenet_path_node_list[match_idx]
        d_v = np.array([x - x_m, y - y_m])
        tou_v = np.array([np.cos(theta_m), np.sin(theta_m)])
        ds = np.dot(d_v, tou_v)
        r_m_v = np.array([x_m, y_m])

        # 投影点位置: r_p = r_m + ds * t_m
        x_r, y_r = r_m_v + ds * tou_v
        # 投影点航向: theta_p = theta_m + k_m * ds
        theta_r = theta_m + k_m * ds
        # 投影点曲率 = 匹配点曲率 (一阶近似)
        k_r = k_m
        project_node_list.append((x_r, y_r, theta_r, k_r))

    return list(match_point_index_list), project_node_list


def cal_s_map_fun(local_path_opt: List[Tuple[float, float, float, float]],
                  origin_xy: Tuple[float, float]) -> List[float]:
    """
    计算参考线上每个节点相对于车辆当前位置投影点的弧长s.

    Args:
        local_path_opt: 参考线 [(x,y,theta,kappa), ...]
        origin_xy: 车辆当前坐标 (x, y)
    Returns:
        s_map: 每个节点对应的弧长 (以车辆投影点为原点)
    """
    # 计算车辆投影点索引
    origin_match_index, _ = find_match_points([origin_xy], local_path_opt, True, 0)
    origin_match_index = origin_match_index[0]

    # 以参考线起点为原点的弧长
    ref_s_map = [0.0]
    for i in range(1, len(local_path_opt)):
        s = math.hypot(local_path_opt[i][0] - local_path_opt[i-1][0],
                       local_path_opt[i][1] - local_path_opt[i-1][1]) + ref_s_map[-1]
        ref_s_map.append(s)

    # 车辆投影点的s
    s0 = cal_projection_s_fun(local_path_opt, [origin_match_index], [origin_xy], ref_s_map)
    s_map = np.array(ref_s_map) - s0[0]
    return list(s_map)


def cal_projection_s_fun(local_path_opt: List[Tuple[float, float, float, float]],
                          match_index_list: List[int],
                          xy_list: List[Tuple[float, float]],
                          s_map: List[float]) -> List[float]:
    """
    计算给定点在参考线上的投影弧长.

    Args:
        local_path_opt: 参考线
        match_index_list: 匹配点索引
        xy_list: 待投点坐标 [(x,y), ...]
        s_map: 参考线弧长映射
    Returns:
        投影点弧长列表
    """
    projection_s_list = []
    for i in range(len(match_index_list)):
        x, y, theta, kappa = local_path_opt[match_index_list[i]]
        d_v = np.array([xy_list[i][0] - x, xy_list[i][1] - y])
        tou_v = np.array([math.cos(theta), math.sin(theta)])
        projection_s_list.append(s_map[match_index_list[i]] + np.dot(d_v, tou_v))
    return projection_s_list


def cal_s_l_fun(obs_xy_list: List[Tuple[float, float]],
                local_path_opt: List[Tuple[float, float, float, float]],
                s_map: List[float]):
    """
    将笛卡尔坐标转换为Frenet坐标 (s, l).

    Args:
        obs_xy_list: 待转换的坐标 [(x,y), ...]
        local_path_opt: 参考线
        s_map: 弧长映射
    Returns:
        s_list, l_list
    """
    match_index_list, projection_list = find_match_points(obs_xy_list, local_path_opt, True, 0)
    s_list = cal_projection_s_fun(local_path_opt, match_index_list, obs_xy_list, s_map)

    l_list = []
    for i in range(len(obs_xy_list)):
        pro_x, pro_y, theta, kappa = projection_list[i]
        n_r = np.array([-math.sin(theta), math.cos(theta)])  # 法向量
        x, y = obs_xy_list[i]
        r_h = np.array([x, y])
        r_r = np.array([pro_x, pro_y])
        l_list.append(float(np.dot(r_h - r_r, n_r)))

    return s_list, l_list


def cal_s_l_deri_fun(xy_list: List[Tuple[float, float]],
                      V_xy_list: List[Tuple[float, float]],
                      a_xy_list: List[Tuple[float, float]],
                      local_path_xy_opt: List[Tuple[float, float, float, float]],
                      origin_xy: Tuple[float, float]):
    """
    坐标变换: 计算l, dl/dt, ds/dt, ddl/dt², dl/ds, dds/dt², d²l/ds².

    Returns:
        l_list, dl_list, ds_list, ddl_list, l_ds_list, dds_list, l_dds_list
    """
    match_index_list, projection_list = find_match_points(xy_list, local_path_xy_opt, True, 0)

    l_list, dl_list, ds_list = [], [], []
    ddl_list, l_ds_list, dds_list, l_dds_list = [], [], [], []

    for i in range(len(xy_list)):
        x, y, theta, kappa = projection_list[i]
        nor_r = np.array([-math.sin(theta), math.cos(theta)])
        tou_r = np.array([math.cos(theta), math.sin(theta)])
        r_h = np.array([origin_xy[0], origin_xy[1]])
        r_r = np.array([x, y])

        # 1. l
        l = float(np.dot(r_h - r_r, nor_r))
        l_list.append(l)

        # 2. dl
        Vx, Vy = V_xy_list[i]
        V_h = np.array([Vx, Vy])
        dl = float(np.dot(V_h, nor_r))
        dl_list.append(dl)

        # 3. ds
        denom = 1 - kappa * l_list[i]
        if abs(denom) < 1e-10:
            denom = 1e-10
        ds = float(np.dot(V_h, tou_r) / denom)
        ds_list.append(ds)

        # 4. ddl
        ax, ay = a_xy_list[i]
        a_h = np.array([ax, ay])
        ddl = float(np.dot(a_h, nor_r) - kappa * (1 - kappa * l) * ds**2)
        ddl_list.append(ddl)

        # 5. l_ds = dl/ds
        if abs(ds) < 1e-6:
            l_ds = 0.0
        else:
            l_ds = dl / ds
        l_ds_list.append(l_ds)

        # 6. dds
        dds = float((np.dot(a_h, tou_r) + 2 * ds**2 * kappa * l_ds) / denom)
        dds_list.append(dds)

        # 7. l_dds = (ddl - l_ds * dds) / ds²
        if abs(ds) < 1e-6:
            l_dds = 0.0
        else:
            l_dds = (ddl - l_ds * dds) / ds**2
        l_dds_list.append(l_dds)

    return l_list, dl_list, ds_list, ddl_list, l_ds_list, dds_list, l_dds_list
