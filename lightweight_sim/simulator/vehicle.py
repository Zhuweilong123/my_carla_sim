"""自车模型 — 运动学自行车模型 + 误差状态计算"""

import math
import numpy as np
from typing import List, Tuple, Optional
from .data_types import VehicleState, VehicleParams
from ..algorithms.utils.frenet import find_match_points, cal_s_l_deri_fun, cal_s_map_fun


class EgoVehicle:
    """
    自车模型.

    提供两种更新模式:
      - kinematic_step():  运动学自行车模型 (默认, 简单稳定)
      - dynamic_step():    动力学自行车模型 (4状态, 可对接LQR/MPC)
    """

    def __init__(self, state: VehicleState, params: Optional[VehicleParams] = None):
        self._state = state
        self.params = params or VehicleParams()
        self.length = self.params.wheelbase + 1.0  # 车身总长约=轴距+前后悬
        self.width = 2.0

        # 历史状态
        self._prev_state: Optional[VehicleState] = None

    # ---- 状态更新 ----

    def kinematic_step(self, steer: float, accel: float, dt: float) -> VehicleState:
        """
        运动学自行车模型更新一步.

        状态: [x, y, phi, vx]
        控制: [steer(前轮转角), accel(加速度)]
        """
        prev = self._state
        L = self.params.wheelbase

        # 更新速度
        v_new = max(0.0, prev.vx + accel * dt)  # 不允许负向速度 (简化)

        # 更新航向
        phi_new = prev.phi + (v_new / L) * math.tan(steer) * dt

        # 更新位置
        x_new = prev.x + v_new * math.cos(phi_new) * dt
        y_new = prev.y + v_new * math.sin(phi_new) * dt

        self._prev_state = prev
        self._state = VehicleState(
            x=x_new, y=y_new, phi=phi_new,
            vx=v_new, vy=0.0,  # 运动学模型假设vy=0 (无侧滑)
            r=(v_new / L) * math.tan(steer) if dt > 0 else 0.0,
            steer=steer, accel=accel,
            timestamp=prev.timestamp + dt,
        )
        return self._state

    def dynamic_step(self, steer: float, accel: float, dt: float) -> VehicleState:
        """
        动力学自行车模型更新一步 (使用现有4状态动力学).

        注意: 这里简化实现, 完整版需要积分A,B矩阵.
        对于LQR/MPC控制, 误差状态计算在get_error_state()中完成.
        """
        # 先用运动学近似, 然后根据动力学修正vy, r
        self.kinematic_step(steer, accel, dt)

        # 动力学修正 (简化: 从侧偏刚度估算vy)
        prev = self._prev_state
        if prev is not None and prev.vx > 0.1:
            Cf, Cr = self.params.Cf, self.params.Cr
            m = self.params.m
            a, b = self.params.a, self.params.b

            Vx = prev.vx
            vy_dot = ((Cf + Cr) / (m * Vx)) * self._state.vy + \
                     ((a * Cf - b * Cr) / (m * Vx) - Vx) * self._state.r - \
                     (Cf / m) * steer
            r_dot = ((a * Cf - b * Cr) / (self.params.Iz * Vx)) * self._state.vy + \
                    ((a**2 * Cf + b**2 * Cr) / (self.params.Iz * Vx)) * self._state.r - \
                    (a * Cf / self.params.Iz) * steer

            self._state.vy += vy_dot * dt
            self._state.r += r_dot * dt

        return self._state

    def step(self, steer: float, accel: float, dt: float,
             model: str = "kinematic") -> VehicleState:
        """统一的步进接口"""
        if model == "kinematic":
            return self.kinematic_step(steer, accel, dt)
        elif model == "dynamic":
            return self.dynamic_step(steer, accel, dt)
        else:
            raise ValueError(f"Unknown vehicle model: {model}")

    # ---- 状态查询 ----

    def get_state(self) -> VehicleState:
        return self._state

    def set_state(self, state: VehicleState):
        self._prev_state = self._state
        self._state = state

    def get_error_state(self,
                        ref_path: List[Tuple[float, float, float, float]],
                        ts: float = 0.1) -> np.ndarray:
        """
        计算横向误差状态 e_rr = [ed, ėd, eφ, ėφ] 供LQR/MPC使用.

        完全替代CARLA的 vehicle.get_location/velocity 等调用.

        Args:
            ref_path: 参考线 [(x,y,theta,kappa), ...]
            ts: 预测时间 (s), 用于补偿控制延迟
        Returns:
            np.array([ed, ed_dot, ephi, ephi_dot])
        """
        s = self._state

        # 预测位置 (补偿延迟)
        pred_x = s.x + s.vx * ts * math.cos(s.phi) - s.vy * ts * math.sin(s.phi)
        pred_y = s.y + s.vy * ts * math.cos(s.phi) + s.vx * ts * math.sin(s.phi)
        pred_phi = s.phi + s.r * ts

        # 匹配点查找
        match_idx, proj_list = find_match_points(
            [(pred_x, pred_y)], ref_path, is_first_run=True, pre_match_index=0
        )
        match_idx = match_idx[0]
        pro_x, pro_y, pro_theta, pro_kappa = proj_list[0]

        # 切向量和法向量
        tor_v = np.array([math.cos(ref_path[match_idx][2]),
                          math.sin(ref_path[match_idx][2])])
        n_v = np.array([-math.sin(ref_path[match_idx][2]),
                         math.cos(ref_path[match_idx][2])])

        # ed: 横向误差
        d_v = np.array([pred_x - ref_path[match_idx][0],
                        pred_y - ref_path[match_idx][1]])
        ed = float(np.dot(n_v, d_v))

        # es: 纵向偏移
        es = float(np.dot(tor_v, d_v))

        # theta_r: 投影点航向
        theta_r = ref_path[match_idx][2] + ref_path[match_idx][3] * es

        # ed_dot: 横向误差变化率
        Vx_w = s.vx * math.cos(s.phi) - s.vy * math.sin(s.phi)
        Vy_w = s.vx * math.sin(s.phi) + s.vy * math.cos(s.phi)
        ed_dot = Vy_w * math.cos(pred_phi - theta_r) + Vx_w * math.sin(pred_phi - theta_r)

        # ephi: 航向角误差 (sin近似)
        ephi = math.sin(pred_phi - theta_r)

        # S_dot
        denom = 1 - ref_path[match_idx][3] * ed
        if abs(denom) < 1e-10:
            denom = 1e-10
        S_dot = (Vx_w * math.cos(pred_phi - theta_r) -
                 Vy_w * math.sin(pred_phi - theta_r)) / denom

        # ephi_dot
        ephi_dot = s.r - ref_path[match_idx][3] * S_dot

        return np.array([ed, ed_dot, ephi, ephi_dot])

    # ---- 碰撞检测 ----

    def get_corners(self) -> np.ndarray:
        """返回自车四角在世界坐标系的坐标 (用于碰撞检测)"""
        s = self._state
        hl, hw = self.length / 2, self.width / 2
        corners = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
        cos_h, sin_h = math.cos(s.phi), math.sin(s.phi)
        rot = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        return corners @ rot.T + np.array([[s.x, s.y]])

    def check_collision_with_obstacle(self,
                                       obs_x: float, obs_y: float,
                                       obs_length: float, obs_width: float,
                                       obs_heading: float) -> bool:
        """检查自车是否与障碍物碰撞 (SAT)"""
        ego_corners = self.get_corners()

        hl, hw = obs_length / 2, obs_width / 2
        obs_corners = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
        cos_h, sin_h = math.cos(obs_heading), math.sin(obs_heading)
        rot = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        obs_corners = obs_corners @ rot.T + np.array([[obs_x, obs_y]])

        for corners in [ego_corners, obs_corners]:
            for i in range(4):
                edge = corners[(i + 1) % 4] - corners[i]
                axis = np.array([-edge[1], edge[0]])
                axis = axis / (np.linalg.norm(axis) + 1e-10)
                proj1 = ego_corners @ axis
                proj2 = obs_corners @ axis
                if proj1.max() < proj2.min() or proj2.max() < proj1.min():
                    return False
        return True
