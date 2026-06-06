"""运动规划器 — DP+QP 管道 + 多进程架构"""

import time
import math
import numpy as np
import threading
import queue
from typing import List, Tuple, Optional
from .dp_path_plan import DP_algorithm
from .qp_path_plan import Quadratic_planning, cal_lmin_lmax, frenet_2_x_y_theta_kappa
from ..utils.frenet import (find_match_points, cal_s_map_fun, cal_s_l_fun, cal_s_l_deri_fun)
from ..utils.reference_line import smooth_reference_line


# =============================================================================
# 规划子进程
# =============================================================================

def _planning_thread(request_queue: queue.Queue, response_queue: queue.Queue):
    """
    独立的规划线程. 通过Queue与主线程通信.
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

        # 2. 采样局部参考线 (后10点+前70点 ≈ 160m范围)
        local_frenet_path = _sample_path(match_point_list[0], global_frenet_path,
                                         back=10, forward=70)

        # 3. QP平滑参考线
        local_frenet_path_opt = smooth_reference_line(local_frenet_path)

        # 4. s_map (以车辆当前位置为原点)
        s_map = cal_s_map_fun(local_frenet_path_opt, origin_xy=vehicle_loc)

        # 5. 障碍物的s,l (过滤掉车辆后方的障碍物: obs_s < -5m)
        if obs_xy_list:
            raw_s, raw_l = cal_s_l_fun(obs_xy_list, local_frenet_path_opt, s_map)
            obs_s_list, obs_l_list = [], []
            for s, l, xy in zip(raw_s, raw_l, obs_xy_list):
                if s > -5:  # 只考虑前方和近后方障碍物
                    obs_s_list.append(s)
                    obs_l_list.append(l)
            if not obs_s_list:
                obs_s_list, obs_l_list = [], []
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
                sampling_res=2.0,    # 增密分辨率 2m
                row=12, col=10,      # 更密的列采样
                sample_s=8,          # 列间距8m (原15m)
                sample_l=1.0,        # 行间距1m (原1.5m)
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

        # 10. Frenet → Cartesian (并补上车辆当前位置作为路径起点)
        # 先加入车辆当前位置(投影点, s=0)
        full_s = [0.0] + path_s_out
        full_l = [0.0] + path_l_out  # 车辆在参考线上, l≈0
        planned_path = frenet_2_x_y_theta_kappa(
            plan_start_s=full_s[0],
            plan_start_l=full_l[0],
            enriched_s_list=full_s[1:],
            enriched_l_list=full_l[1:],
            frenet_path_opt=local_frenet_path_opt,
            s_map=s_map,
        )

        elapsed = time.time() - start_time
        print(f"[Planner] {plan_used} done in {elapsed:.3f}s, "
              f"path: {len(planned_path)} points")

        response_queue.put((planned_path, match_point_list, path_s_out, path_l_out))

    # ---- 主循环: 接收请求 → 规划 → 发送结果 ----
    while True:
        try:
            data = request_queue.get(timeout=1.0)  # 1秒超时, 允许检查退出
        except queue.Empty:
            continue

        if data is None:  # 停止信号
            break

        try:
            do_planning(data)
        except Exception as e:
            print(f"[Planner] error: {e}")
            import traceback; traceback.print_exc()
            # 发送空结果防止主线程阻塞
            response_queue.put(([], [0], [], []))


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
    运动规划器 (后台线程).

    用法:
        planner = MotionPlanner(global_path)
        planner.start()
        planner.plan(ego_state, obstacles, ...)  # 非阻塞发送
        if planner.poll_result():                 # 非阻塞检查
            path = planner.get_result()           # 获取结果
        planner.stop()
    """

    def __init__(self, global_frenet_path: List[Tuple[float, float, float, float]]):
        self.global_path = global_frenet_path
        self._request_queue: queue.Queue = queue.Queue()
        self._response_queue: queue.Queue = queue.Queue()
        self._thread: Optional[threading.Thread] = None
        self._is_running = False
        self._match_point_list = None

        if global_frenet_path:
            self._match_point_list = [0]

    def start(self):
        """启动规划后台线程"""
        if self._is_running:
            return
        self._thread = threading.Thread(
            target=_planning_thread,
            args=(self._request_queue, self._response_queue),
            daemon=True,
        )
        self._thread.start()
        self._is_running = True
        print("[Planner] thread started")

    def stop(self):
        """停止规划线程"""
        if not self._is_running:
            return
        self._request_queue.put(None)  # 停止信号
        self._thread.join(timeout=5)
        self._is_running = False
        print("[Planner] thread stopped")

    def plan(self, ego_state, obstacles,
             vehicle_v: Tuple[float, float],
             vehicle_a: Tuple[float, float],
             pred_loc: Tuple[float, float],
             vehicle_loc: Tuple[float, float]) -> Optional[List]:
        """触发一次规划 (非阻塞: 发送数据给后台线程)."""
        if not self._is_running:
            return None

        obs_xy = [(obs.x, obs.y) for obs in obstacles]

        data = (obs_xy, vehicle_loc, pred_loc,
                vehicle_v, vehicle_a,
                self.global_path, self._match_point_list)

        self._request_queue.put(data)
        return True

    def poll_result(self) -> bool:
        """检查是否有结果就绪 (非阻塞)."""
        return not self._response_queue.empty()

    def get_result(self) -> Optional[List]:
        """获取规划结果 (非阻塞, 队列空时返回None)."""
        try:
            result = self._response_queue.get_nowait()
            planned_path, self._match_point_list, path_s, path_l = result
            return planned_path
        except queue.Empty:
            return None
        except Exception as e:
            print(f"[Planner] get_result failed: {e}")
            return None
