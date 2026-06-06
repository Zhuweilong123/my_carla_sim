"""LQR横向控制器 (从 controller/Controller.py 解耦)

核心改动:
  取消 carla.Vehicle 依赖, 改为接收 VehicleState + ref_path
  矩阵计算 (A/B, Riccati迭代, 前馈) 完全保留原始实现.

原始文件: controller/Controller.py -> Lateral_LQR_controller
"""

import math
import numpy as np
from typing import List, Tuple


class LateralLQRController:
    """
    LQR无限时域最优横向控制.

    状态: [ed, ėd, eφ, ėφ]
    控制: δ (前轮转角, rad)

    组成:
      1. 状态反馈: u = -K @ e_rr
      2. 前馈控制: δ_ff (补偿道路曲率引起的稳态误差)
      最终: δ = -K @ e_rr + δ_ff
    """

    def __init__(self, vehicle_para: Tuple[float, ...],
                 Q: np.ndarray = None, R: float = 1.0,
                 ts: float = 0.1):
        """
        Args:
            vehicle_para: (a, b, m, Cf, Cr, Iz)
            Q: 4x4 状态权重矩阵
            R: 控制量权重标量
            ts: 离散化时间间隔 (s)
        """
        self.a, self.b, self.m, self.Cf, self.Cr, self.Iz = vehicle_para
        self.ts = ts
        self.R = R

        # 矩阵
        self.A = np.zeros((4, 4), dtype=np.float64)
        self.B = np.zeros((4, 1), dtype=np.float64)
        self.K: np.ndarray = None    # 反馈增益 (1x4)
        self.delta_f: float = 0.0    # 前馈转角

        # 匹配点索引 (加速搜索)
        self.min_index: int = 0

        # 初始化Q
        if Q is None:
            self.Q = np.eye(4)
            self.Q[0, 0] = 200   # ed
            self.Q[1, 1] = 1     # ėd
            self.Q[2, 2] = 50    # eφ
            self.Q[3, 3] = 1     # ėφ
        else:
            self.Q = Q

        # Debug 变量
        self.x_pre = 0.0
        self.y_pre = 0.0
        self.x_pro = 0.0
        self.y_pro = 0.0

    # =====================================================================
    # 新版接口: 直接接收数值状态 (解耦CARLA)
    # =====================================================================

    def control(self, x: float, y: float, phi: float,
                vx: float, vy: float, r: float,
                ref_path: List[Tuple[float, float, float, float]]) -> float:
        """新版控制接口: 从数值状态直接计算前轮转角

        Args:
            x, y, phi: 世界坐标系位置+航向
            vx, vy: 车体坐标系速度 (m/s)
            r: 横摆角速度 (rad/s)
            ref_path: 参考线 [(x,y,theta,kappa), ...]
        Returns:
            前轮转角 (rad)
        """
        # Step 1: 计算A, B矩阵 (需要Vx)
        V_length = math.sqrt(vx**2 + vy**2)
        beta = math.atan2(vy, vx)
        Vx = max(V_length * math.cos(beta), 0.005)
        if V_length * math.cos(beta) < 0:
            Vx = -max(abs(V_length * math.cos(beta)), 0.005)

        self._cal_A_B(Vx)
        self._solve_LQR()

        # Step 2: 计算误差状态 e_rr
        e_rr, k_r = self._cal_error_state(x, y, phi, vx, vy, r, ref_path)

        # Step 3: 前馈控制
        self._cal_feedforward(Vx, k_r)

        # Step 4: 总控制量
        steering = -np.dot(self.K, np.array(e_rr)) + self.delta_f
        return float(steering[0])

    # =====================================================================
    # A/B 矩阵
    # =====================================================================

    def _cal_A_B(self, Vx: float):
        """根据自行车模型和轮胎参数计算连续系统矩阵"""
        Vx = Vx + 0.0001  # 防除零
        a, b, Cf, Cr, m, Iz = self.a, self.b, self.Cf, self.Cr, self.m, self.Iz

        self.A.fill(0)
        self.A[0, 1] = 1
        self.A[1, 1] = (Cf + Cr) / (m * Vx)
        self.A[1, 2] = -(Cf + Cr) / m
        self.A[1, 3] = (a * Cf - b * Cr) / (m * Vx)
        self.A[2, 3] = 1
        self.A[3, 1] = (a * Cf - b * Cr) / (Iz * Vx)
        self.A[3, 2] = -(a * Cf - b * Cr) / Iz
        self.A[3, 3] = (a**2 * Cf + b**2 * Cr) / (Iz * Vx)

        self.B.fill(0)
        self.B[1, 0] = -Cf / m
        self.B[3, 0] = -a * Cf / Iz

    # =====================================================================
    # LQR 求解 — 离散Riccati迭代
    # =====================================================================

    def _solve_LQR(self):
        """迭代求解离散代数Riccati方程, 得到反馈增益K"""
        # 双线性变换离散化
        I4 = np.eye(4)
        temp = np.linalg.inv(I4 - (self.ts * self.A) / 2)
        A_d = temp @ (I4 + (self.ts * self.A) / 2)
        B_d = temp @ self.B * self.ts

        # Riccati迭代
        P = self.Q.copy()
        P_pre = self.Q.copy()
        max_itr = 5000
        eps = 0.1

        for _ in range(max_itr):
            P = (A_d.T @ P @ A_d -
                 A_d.T @ P @ B_d @
                 np.linalg.inv(self.R + B_d.T @ P @ B_d) @
                 (B_d.T @ P @ A_d) +
                 self.Q)
            if abs(P - P_pre).max() < eps:
                break
            P_pre = P

        self.K = np.linalg.inv(B_d.T @ P @ B_d + self.R) @ (B_d.T @ P @ A_d)

    # =====================================================================
    # 误差状态计算 (从 planner_utiles 解耦)
    # =====================================================================

    def _cal_error_state(self, x: float, y: float, phi: float,
                         vx: float, vy: float, r: float,
                         ref_path: List[Tuple[float, float, float, float]],
                         ts_predict: float = 0.1):
        """计算 e_rr = [ed, ėd, eφ, ėφ] 和投影点曲率 κ_r"""
        Vx = vx
        if Vx < 0.005:
            Vx = 0.005

        # 预测位置 (补偿控制延迟)
        x_pred = x + vx * ts_predict * math.cos(phi) - vy * ts_predict * math.sin(phi)
        y_pred = y + vy * ts_predict * math.cos(phi) + vx * ts_predict * math.sin(phi)
        phi_pred = phi + r * ts_predict

        self.x_pre = x_pred
        self.y_pre = y_pred

        # 1. 匹配点查找
        path_len = len(ref_path)
        if path_len == 0:
            return (0.0, 0.0, 0.0, 0.0), 0.0
        # 防止min_index越界 (参考线切换后可能超出新路径长度)
        self.min_index = min(self.min_index, path_len - 1)
        min_d = float('inf')
        for i in range(self.min_index, min(self.min_index + 50, path_len)):
            d = (ref_path[i][0] - x_pred)**2 + (ref_path[i][1] - y_pred)**2
            if d < min_d:
                min_d = d
                self.min_index = i

        min_idx = self.min_index
        mx, my, mtheta, mkappa = ref_path[min_idx]

        # 2. 切/法向量
        tor_v = np.array([math.cos(mtheta), math.sin(mtheta)])
        n_v = np.array([-math.sin(mtheta), math.cos(mtheta)])

        # 3. ed, es
        d_v = np.array([x_pred - mx, y_pred - my])
        ed = float(np.dot(n_v, d_v))
        es = float(np.dot(tor_v, d_v))

        # 4. 投影点
        self.x_pro, self.y_pro = np.array([mx, my]) + es * tor_v

        # 5. theta_r
        theta_r = mtheta + mkappa * es

        # 6. ėd
        Vx_w = vx * math.cos(phi) - vy * math.sin(phi)
        Vy_w = vx * math.sin(phi) + vy * math.cos(phi)
        ed_dot = Vy_w * math.cos(phi_pred - theta_r) + Vx_w * math.sin(phi_pred - theta_r)

        # 7. eφ
        ephi = math.sin(phi_pred - theta_r)

        # 8. S_dot
        denom = 1 - mkappa * ed
        if abs(denom) < 1e-10:
            denom = 1e-10
        S_dot = (Vx_w * math.cos(phi_pred - theta_r) -
                 Vy_w * math.sin(phi_pred - theta_r)) / denom

        # 9. ėφ
        ephi_dot = r - mkappa * S_dot

        k_r = mkappa
        e_rr = (ed, ed_dot, ephi, ephi_dot)

        return e_rr, k_r

    # =====================================================================
    # 前馈控制
    # =====================================================================

    def _cal_feedforward(self, Vx: float, k_r: float):
        """计算前馈转角 δ_ff"""
        a, b = self.a, self.b
        Cf, Cr = self.Cf, self.Cr
        m = self.m

        K3 = self.K[0, 2]  # eφ 对应的增益
        L = a + b

        delta_f = k_r * (L - b * K3 -
                         (b / Cf + a * K3 / Cr - a / Cr) * (m * Vx**2) / L)
        self.delta_f = delta_f  # rad (前馈公式本身输出rad)
