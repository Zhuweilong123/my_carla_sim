"""MPC横向控制器 (从 controller/Controller.py 解耦)

核心改动:
  取消 carla.Vehicle 依赖, 改为接收 VehicleState + ref_path
  预测模型、QP求解完全保留原始实现.
"""

import math
import numpy as np
import cvxopt
from typing import List, Tuple


class LateralMPCController:
    """
    MPC有限时域约束最优横向控制.

    预测区间 N=6, 控制区间 P=2, 状态维度 n=4.
    QP求解cvxopt, 约束 |δ| ≤ 1.
    """

    def __init__(self, vehicle_para: Tuple[float, ...],
                 Q: np.ndarray = None, F: np.ndarray = None,
                 R: float = 1.0, N: int = 6, P: int = 2, ts: float = 0.1):
        """
        Args:
            vehicle_para: (a, b, m, Cf, Cr, Iz)
            Q, F: 4x4 状态/终端权重
            R: 控制量权重
            N, P: 预测/控制区间
            ts: 离散化间隔
        """
        self.a, self.b, self.m, self.Cf, self.Cr, self.Iz = vehicle_para
        self.N = N
        self.P = P
        self.n = 4
        self.ts = ts

        self.A = np.zeros((4, 4), dtype=np.float64)
        self.B = np.zeros((4, 1), dtype=np.float64)
        self.C = np.zeros((4, 1), dtype=np.float64)

        # 离散化矩阵
        self.A_bar = self.B_bar = self.C_bar = None
        self.k_r: float = None
        self.min_index: int = 0

        # 权重
        self.Q = Q if Q is not None else self._default_Q()
        self.F = F if F is not None else np.eye(4)
        self.R = R

        # Debug
        self.x_pre = self.y_pre = 0.0
        self.x_pro = self.y_pro = 0.0

    @staticmethod
    def _default_Q():
        Q = np.eye(4)
        Q[0, 0] = 250  # ed
        Q[1, 1] = 1
        Q[2, 2] = 50   # eφ
        Q[3, 3] = 1
        return Q

    # =====================================================================
    # 新版接口
    # =====================================================================

    def control(self, x: float, y: float, phi: float,
                vx: float, vy: float, r: float,
                ref_path: List[Tuple[float, float, float, float]]) -> float:
        """MPC控制接口: 从数值状态计算前轮转角"""
        V_length = math.sqrt(vx**2 + vy**2)
        beta = math.atan2(vy, vx)
        Vx = max(V_length * math.cos(beta), 0.005)
        if V_length * math.cos(beta) < 0:
            Vx = -max(abs(V_length * math.cos(beta)), 0.005)

        self._cal_A_B_C(Vx)
        e_rr, k_r = self._cal_error_state(x, y, phi, vx, vy, r, ref_path)
        self.k_r = k_r
        self._discretize(Vx)

        steering = self._solve_QP(e_rr)
        return float(steering)

    # =====================================================================
    # A/B/C 矩阵
    # =====================================================================

    def _cal_A_B_C(self, Vx: float):
        Vx = max(Vx, 0.005)
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

        self.C.fill(0)
        self.C[1, 0] = (a * Cf + b * Cr) / (m * Vx) - Vx
        self.C[3, 0] = (a**2 * Cf + b**2 * Cr) / (Iz * Vx)

    # =====================================================================
    # 误差状态 (与LQR相同逻辑)
    # =====================================================================

    def _cal_error_state(self, x, y, phi, vx, vy, r, ref_path, ts_predict=0.1):
        Vx = max(abs(vx), 0.005)
        x_pred = x + vx * ts_predict * math.cos(phi) - vy * ts_predict * math.sin(phi)
        y_pred = y + vy * ts_predict * math.cos(phi) + vx * ts_predict * math.sin(phi)
        phi_pred = phi + r * ts_predict
        self.x_pre, self.y_pre = x_pred, y_pred

        path_len = len(ref_path)
        if path_len == 0:
            return (0.0, 0.0, 0.0, 0.0), 0.0
        self.min_index = min(self.min_index, path_len - 1)
        min_d = float('inf')
        for i in range(self.min_index, min(self.min_index + 50, path_len)):
            d = (ref_path[i][0] - x_pred)**2 + (ref_path[i][1] - y_pred)**2
            if d < min_d:
                min_d = d
                self.min_index = i

        min_idx = self.min_index
        mx, my, mtheta, mkappa = ref_path[min_idx]
        tor_v = np.array([math.cos(mtheta), math.sin(mtheta)])
        n_v = np.array([-math.sin(mtheta), math.cos(mtheta)])
        d_v = np.array([x_pred - mx, y_pred - my])
        ed = float(np.dot(n_v, d_v))
        es = float(np.dot(tor_v, d_v))
        self.x_pro, self.y_pro = np.array([mx, my]) + es * tor_v
        theta_r = mtheta + mkappa * es

        Vx_w = vx * math.cos(phi) - vy * math.sin(phi)
        Vy_w = vx * math.sin(phi) + vy * math.cos(phi)
        ed_dot = Vy_w * math.cos(phi_pred - theta_r) + Vx_w * math.sin(phi_pred - theta_r)
        ephi = math.sin(phi_pred - theta_r)

        denom = 1 - mkappa * ed
        if abs(denom) < 1e-10:
            denom = 1e-10
        S_dot = (Vx_w * math.cos(phi_pred - theta_r) -
                 Vy_w * math.sin(phi_pred - theta_r)) / denom
        ephi_dot = r - mkappa * S_dot

        return (ed, ed_dot, ephi, ephi_dot), mkappa

    # =====================================================================
    # 离散化
    # =====================================================================

    def _discretize(self, Vx: float):
        I4 = np.eye(4)
        temp = np.linalg.inv(I4 - (self.ts * self.A) / 2)
        self.A_bar = temp @ (I4 + (self.ts * self.A) / 2)
        self.B_bar = temp @ self.B * self.ts
        self.C_bar = temp @ self.C * self.ts * self.k_r * Vx

    # =====================================================================
    # QP 求解
    # =====================================================================

    def _solve_QP(self, e_rr: Tuple[float, ...]) -> float:
        N, P, n = self.N, self.P, self.n
        e_rr_arr = np.array(e_rr).reshape((n, 1))

        # 构造 M
        M = np.zeros(((N + 1) * n, n))
        M[0:n, :] = np.eye(n)
        for i in range(1, N + 1):
            M[i * n:(i + 1) * n, :] = self.A_bar @ M[(i - 1) * n:i * n, :]

        # 构造 C
        C_mat = np.zeros(((N + 1) * n, N * P))
        C_mat[n:2 * n, 0:P] = self.B_bar
        for i in range(2, N + 1):
            C_mat[i * n:(i + 1) * n, (i - 1) * P:i * P] = self.B_bar
            for j in range(i - 2, -1, -1):
                C_mat[i * n:(i + 1) * n, j * P:(j + 1) * P] = \
                    self.A_bar @ C_mat[i * n:(i + 1) * n, (j + 1) * P:(j + 2) * P]

        # 构造 Cc
        Cc = np.zeros(((N + 1) * n, 1))
        for i in range(1, N + 1):
            Cc[n * i:n * (i + 1), 0:1] = \
                self.A_bar @ Cc[n * (i - 1):n * i, 0:1] + self.C_bar

        # Q_bar, R_bar
        Q_bar = np.zeros(((N + 1) * n, (N + 1) * n))
        for i in range(N):
            Q_bar[i * n:(i + 1) * n, i * n:(i + 1) * n] = self.Q
        Q_bar[N * n:, N * n:] = self.F

        R_bar = np.zeros((N * P, N * P))
        for i in range(N):
            R_bar[i * P:(i + 1) * P, i * P:(i + 1) * P] = np.eye(P) * self.R

        # H, f
        H = C_mat.T @ Q_bar @ C_mat + R_bar
        E = C_mat.T @ Q_bar @ Cc + C_mat.T @ Q_bar @ M @ e_rr_arr
        H = 2 * H
        f = 2 * E

        # 约束: -1 ≤ u ≤ 1
        n_vars = N * P
        lb = -np.ones((n_vars, 1))
        ub = np.ones((n_vars, 1))
        G = np.concatenate((np.identity(n_vars), -np.identity(n_vars)))
        h = np.concatenate((ub, -lb))

        cvxopt.solvers.options['show_progress'] = False
        try:
            res = cvxopt.solvers.qp(
                cvxopt.matrix(H), cvxopt.matrix(f),
                G=cvxopt.matrix(G), h=cvxopt.matrix(h)
            )
            return res['x'][0]
        except Exception:
            # QP失败时回退到零转角
            return 0.0
