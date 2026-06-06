"""横纵向联合控制器 — 门面模式统一接口 (从 controller/Controller.py 解耦)"""

from typing import List, Tuple, Optional
from .lat_lqr import LateralLQRController
from .lat_mpc import LateralMPCController
from .lon_pid import LongitudinalPIDController


class VehicleController:
    """
    横纵向联合控制器.

    用法:
        ctrl = VehicleController(vehicle_para, controller_type="LQR")
        ctrl.update_ref_path(ref_path)
        steer, throttle, brake = ctrl.step(x, y, phi, vx, vy, r, target_speed_kmh)

    内部组合:
        lat_controller: LQR 或 MPC
        lon_controller: PID
    """

    def __init__(self, vehicle_para: Tuple[float, ...],
                 controller_type: str = "LQR_controller",
                 target_speed_kmh: float = 50.0):
        """
        Args:
            vehicle_para: (a, b, m, Cf, Cr, Iz)
            controller_type: "LQR_controller" | "MPC_controller"
            target_speed_kmh: 目标速度 (km/h)
        """
        self.controller_type = controller_type
        self.vehicle_para = vehicle_para

        # 横向控制器
        if controller_type == "MPC_controller":
            self.lat = LateralMPCController(vehicle_para)
        else:
            self.lat = LateralLQRController(vehicle_para)

        # 纵向控制器
        self.lon = LongitudinalPIDController()
        self.lon.set_target(target_speed_kmh)

        # 当前使用的参考线
        self.ref_path: List[Tuple[float, float, float, float]] = []

    def update_ref_path(self, ref_path: List[Tuple[float, float, float, float]]):
        """更新参考线"""
        self.ref_path = ref_path

    def set_target_speed(self, speed_kmh: float):
        self.lon.set_target(speed_kmh)

    def step(self, x: float, y: float, phi: float,
             vx: float, vy: float, r: float) -> Tuple[float, float, float]:
        """
        一步控制计算.

        Args:
            x, y, phi: 世界坐标系位置+航向
            vx, vy: 车体坐标系速度 (m/s)
            r: 横摆角速度 (rad/s)
        Returns:
            (steer, throttle, brake)
              steer: 前轮转角 (rad)
              throttle: 油门 [0,1]
              brake: 刹车 [0,1]
        """
        if not self.ref_path:
            return 0.0, 0.0, 0.0

        # 横向: 计算方向盘转角
        raw_steer = self.lat.control(x, y, phi, vx, vy, r, self.ref_path)

        # 转向限幅
        steer = max(-1.0, min(1.0, raw_steer))

        # 纵向: 计算加速度 → 油门/刹车
        current_speed = (vx**2 + vy**2)**0.5
        accel = self.lon.control(current_speed)

        if accel >= 0:
            throttle = min(1.0, accel)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, abs(accel))

        return steer, throttle, brake
