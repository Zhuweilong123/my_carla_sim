"""纵向PID控制器 (从 controller/Controller.py 解耦)"""

import math
import numpy as np
from collections import deque


class LongitudinalPIDController:
    """
    PID纵向速度控制.

    特性:
      - 积分分离: |error| > 1 km/h 时清零积分项, 防止超调
      - 滑动窗口: deque(maxlen=60) 限制积分/微分历史长度

    输入: 当前速度(m/s), 目标速度(km/h)
    输出: 加速度控制量 (正值=油门, 负值=刹车)
    """

    def __init__(self, K_P: float = 1.15, K_I: float = 0.0, K_D: float = 0.0,
                 dt: float = 0.01, error_threshold: float = 1.0):
        """
        Args:
            K_P: 比例系数
            K_I: 积分系数 (默认0, 纯P控制)
            K_D: 微分系数 (默认0)
            dt: 控制周期 (s)
            error_threshold: 积分分离阈值 (km/h)
        """
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.dt = dt
        self.error_threshold = error_threshold

        self.target_speed: float = 50.0  # km/h
        self.error_buffer = deque(maxlen=60)  # 误差滑动窗口

    def control(self, current_speed_ms: float) -> float:
        """
        计算加速度控制量.

        Args:
            current_speed_ms: 当前速度 (m/s)
        Returns:
            加速度控制量 (正值=油门, 负值=需刹车)
        """
        # 当前速度 (km/h)
        cur_speed = 3.6 * current_speed_ms

        # 误差
        error = self.target_speed - cur_speed
        self.error_buffer.append(error)

        # 积分项
        if len(self.error_buffer) >= 2:
            integral_error = sum(self.error_buffer) * self.dt
            differential_error = (self.error_buffer[-1] - self.error_buffer[-2]) / self.dt
        else:
            integral_error = 0.0
            differential_error = 0.0

        # 积分分离
        if abs(error) > self.error_threshold:
            integral_error = 0.0
            self.error_buffer.clear()

        return self.K_P * error + self.K_I * integral_error + self.K_D * differential_error

    def set_target(self, speed_kmh: float):
        self.target_speed = speed_kmh

    def reset(self):
        self.error_buffer.clear()
