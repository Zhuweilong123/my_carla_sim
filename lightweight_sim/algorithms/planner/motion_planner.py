"""运动规划器 — DP+QP 管道 + 多进程架构"""

import time
import math
import numpy as np
from multiprocessing import Process, Pipe
from typing import List, Tuple, Optional
from .dp_path_plan import DP_algorithm
from .qp_path_plan import Quadratic_planning, cal_lmin_lmax, frenet_2_x_y_theta_kappa
from ..utils.frenet import (find_match_points, cal_s_map_fun, cal_s_l_fun, cal_s_l_deri_fun)
from ..utils.reference_line import smooth_reference_line


# =============================================================================
# 规划子进程
# =============================================================================

def _planning_process(conn):
    """
    独立的规划子进程. 通过Pipe与主进程通信.
    """
    def do_planning(data):
        """实际规划逻辑"""
        (obs_xy_list, vehicle_loc, pred_loc,
         vehicle_v, vehicle_a,
         global_frenet_path, match_point_list) = data

        start_time = time.time()

        # 1. 确定预测点在全局路径上的投影
        match_point_list, _ = find_match_points(
            xy_list=[pred_loc],
            frenet_path_node_list=global_frenet_path,
            is_first_run=False,
            pre_match_index=match_point_list[0],
        )

        # 2. 采样局部参考线
        local_frenet_path = _sample_path(match_point_list[0], global_frenet_path,
                                         back=10, forward=50)

        # 3. QP平滑参考线
        local_frenet_path_opt = smooth_reference_line(local_frenet_path)

        # 4. s_map (以车辆当前位置为原点)
        s_map = cal_s_map_fun(local_frenet_path_opt, origin_xy=vehicle_loc)

        # 5. 障碍物的s,l
        if obs_xy_list:
            obs_s_list, obs_l_list = cal_s_l_fun(obs_xy_list, local_frenet_path_opt, s_map)
        else:
            obs_s_list, obs_l_list = [], []

        # 6. 规划起点的s,l
        begin_s_list, begin_l_list = cal_s_l_fun([pred_loc], local_frenet_path_opt, s_map)

        # 7. 规划起点的导数
        l_list, _, _, _, l_ds_list, _, l_dds_list = cal_s_l_deri_fun(
            xy_list=[pred_loc],
            V_xy_list=[vehicle_v],
            a_xy_list=[vehicle_a],
            local_path_xy_opt=local_frenet_path_opt,
            origin_xy=pred_loc,
        )

        # 8. DP 动态规划
        try:
            dp_path_s, dp_path_l = DP_algorithm(
                obs_s_list, obs_l_list,
                plan_start_s=begin_s_list[0],
                plan_start_l=l_list[0],
                plan_start_dl=l_ds_list[0],
                plan_start_ddl=l_dds_list[0],
            )
        except Exception as e:
            print(f"[Planner] DP failed: {e}")
            # 回退: 直接使用参考线
            dp_path_s = list(s_map[1:min(len(s_map), 50)])
            dp_path_l = [0.0] * len(dp_path_s)

        # 9. QP 二次规划
        try:
            # 降采样
            dp_l_down = dp_path_l[::2]
            dp_s_down = dp_path_s[::2]

            l_min, l_max = cal_lmin_lmax(
                dp_path_s=dp_s_down, dp_path_l=dp_l_down,
                obs_s_list=obs_s_list, obs_l_list=obs_l_list,
                obs_length=5, obs_width=4,
            )

            qp_path_l, _, _ = Quadratic_planning(
                list(l_min), list(l_max),
                plan_start_l=l_list[0],
                plan_start_dl=l_ds_list[0],
                plan_start_ddl=l_dds_list[0],
            )

            # 合并DP和QP结果 (插值回原始分辨率)
            path_s_out = [dp_path_s[0]]
            path_l_out = [qp_path_l[0]]
            for i in range(1, len(qp_path_l)):
                path_s_out.append((dp_path_s[i] + dp_path_s[i - 1]) / 2)
                path_l_out.append((qp_path_l[i] + qp_path_l[i - 1]) / 2)
            path_s_out.append(dp_path_s[-1])
            path_l_out.append(qp_path_l[-1])

            plan_used = "DP+QP"
        except Exception as e:
            print(f"[Planner] QP failed: {e}, using DP only")
            path_s_out = dp_path_s
            path_l_out = dp_path_l
            plan_used = "DP"

        # 10. Frenet → Cartesian
        planned_path = frenet_2_x_y_theta_kappa(
            plan_start_s=begin_s_list[0],
            plan_start_l=begin_l_list[0],
            enriched_s_list=path_s_out,
            enriched_l_list=path_l_out,
            frenet_path_opt=local_frenet_path_opt,
            s_map=s_map,
        )

        elapsed = time.time() - start_time
        print(f"[Planner] {plan_used} done in {elapsed:.3f}s, "
              f"path: {len(planned_path)} points")

        conn.send((planned_path, match_point_list, path_s_out, path_l_out))

    # ---- 主循环: 接收请求 → 规划 → 发送结果 ----
    while True:
        try:
            data = conn.recv()
        except EOFError:
            break

        try:
            do_planning(data)
        except Exception as e:
            print(f"[Planner] error: {e}")
            import traceback; traceback.print_exc()
            # 发送空结果防止主进程阻塞
            try:
                conn.send(([], [0], [], []))
            except Exception:
                pass


def _sample_path(match_idx: int, global_path: List, back: int, forward: int):
    """从全局路径中采样局部参考线"""
    n = len(global_path)
    b = min(back, match_idx)
    f = min(forward, n - match_idx - 1)
    # 保持总长度
    if b < back:
        f = min(back + forward - b, n - match_idx - 1)
    if f < forward:
        b = min(back + forward - f, match_idx)

    return global_path[match_idx - b: match_idx + f + 1]


# =============================================================================
# 主进程端: MotionPlanner
# =============================================================================

class MotionPlanner:
    """
    运动规划器 (主进程端).

    用法:
        planner = MotionPlanner(global_path)
        planner.start()                              # 启动子进程
        planned = planner.plan(ego_state, obstacles)  # 触发规划
        planner.stop()                               # 关闭子进程
    """

    def __init__(self, global_frenet_path: List[Tuple[float, float, float, float]]):
        self.global_path = global_frenet_path
        self._parent_conn = None
        self._child_conn = None
        self._process = None
        self._is_running = False
        self._match_point_list = None

        # 初始化匹配点 (在全局路径起点)
        if global_frenet_path:
            self._match_point_list = [0]

    def start(self):
        """启动规划子进程"""
        if self._is_running:
            return
        self._parent_conn, self._child_conn = Pipe()
        self._process = Process(target=_planning_process, args=(self._child_conn,))
        self._process.start()
        self._is_running = True
        print("[Planner] subprocess started")

    def stop(self):
        """停止规划子进程"""
        if not self._is_running:
            return
        self._parent_conn.close()
        self._process.terminate()
        self._process.join(timeout=2)
        self._is_running = False
        print("[Planner] subprocess stopped")

    def plan(self, ego_state, obstacles,
             vehicle_v: Tuple[float, float],
             vehicle_a: Tuple[float, float],
             pred_loc: Tuple[float, float],
             vehicle_loc: Tuple[float, float]) -> Optional[List]:
        """
        触发一次规划 (非阻塞: 发送数据给子进程).

        需要调用 get_result() 获取结果 (阻塞直到子进程完成).
        """
        if not self._is_running:
            return None

        # 提取障碍物坐标
        obs_xy = []
        for obs in obstacles:
            obs_xy.append((obs.x, obs.y))

        data = (obs_xy, vehicle_loc, pred_loc,
                vehicle_v, vehicle_a,
                self.global_path, self._match_point_list)

        self._parent_conn.send(data)
        return True  # 表示已发送

    def poll_result(self) -> bool:
        """检查子进程是否有结果就绪 (非阻塞)."""
        if not self._is_running:
            return False
        return self._parent_conn.poll()

    def get_result(self) -> Optional[List]:
        """
        获取规划结果 (阻塞直到子进程返回).
        建议先调用 poll_result() 确保有数据.
        """
        if not self._is_running:
            return None
        try:
            result = self._parent_conn.recv()
            planned_path, self._match_point_list, path_s, path_l = result
            return planned_path
        except Exception as e:
            print(f"[Planner] recv failed: {e}")
            return None
