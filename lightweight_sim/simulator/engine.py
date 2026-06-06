"""仿真引擎 — 时间步进与状态管理"""

import time
from typing import Optional
from .data_types import ScenarioConfig, VehicleState, ControlCommand
from .world import World, RoadDef
from .vehicle import EgoVehicle, VehicleParams
from .obstacle import ObstacleManager


class SimulationEngine:
    """
    仿真引擎 — 管理世界、自车、障碍物的状态更新.

    物理步长 dt = 0.05s (20Hz), 与CARLA同步模式一致.
    """

    def __init__(self, config: ScenarioConfig):
        self.config = config

        # 初始化世界
        self.world = World(config.road)

        # 初始化自车
        ego_state = VehicleState(
            x=config.ego_start_x,
            y=config.ego_start_y,
            phi=config.ego_start_phi,
            vx=config.ego_start_speed,
            vy=0.0,
            r=0.0,
            steer=0.0,
            accel=0.0,
            timestamp=0.0,
        )
        self.ego = EgoVehicle(ego_state, VehicleParams())

        # 初始化障碍物
        self.obstacles = ObstacleManager()
        self.obstacles.add_from_config(config.obstacles)

        # 时间管理
        self.sim_time: float = 0.0
        self.step_count: int = 0
        self.physics_dt: float = 0.05  # 20Hz

        # 目标
        self.target_speed: float = config.target_speed  # km/h
        self.destination = config.destination

        # 统计
        self.collision_occurred: bool = False
        self.reached_destination: bool = False

    def step(self, control: ControlCommand, dt: Optional[float] = None) -> VehicleState:
        """
        执行一步仿真.

        Args:
            control: 控制指令 (steer, throttle, brake)
            dt: 物理步长, 默认0.05s
        Returns:
            新的车辆状态
        """
        if dt is None:
            dt = self.physics_dt

        # 将throttle/brake转为加速度
        accel = control.throttle - control.brake * 2.0  # brake更强

        # 更新自车
        new_state = self.ego.kinematic_step(control.steer, accel, dt)

        # 更新障碍物
        self.obstacles.step(dt)

        # 碰撞检测
        self.collision_occurred = self.obstacles.check_collision(
            new_state.x, new_state.y,
            self.ego.length, self.ego.width, new_state.phi
        )

        # 检查是否到达终点
        if self.destination:
            dist = ((new_state.x - self.destination[0])**2 +
                    (new_state.y - self.destination[1])**2)**0.5
            if dist < 2.0:
                self.reached_destination = True

        self.sim_time += dt
        self.step_count += 1

        return new_state

    def get_state(self) -> VehicleState:
        return self.ego.get_state()

    def get_error_state(self, ts: float = 0.1):
        """获取误差状态 (供控制器使用)"""
        ref_path = self.world.ref_path_as_tuples
        if not ref_path:
            return None
        return self.ego.get_error_state(ref_path, ts)

    @property
    def is_done(self) -> bool:
        return self.collision_occurred or self.reached_destination
