"""统一数据结构定义 — 解耦CARLA依赖"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np


# =============================================================================
# 路径相关
# =============================================================================

@dataclass
class PathPoint:
    """笛卡尔坐标系下的路径点 (x, y, theta, kappa)"""
    x: float
    y: float
    theta: float      # 切向角 (rad), 切线与X轴夹角
    kappa: float      # 曲率 (1/m)

    def __iter__(self):
        return iter((self.x, self.y, self.theta, self.kappa))


@dataclass
class FrenetPoint:
    """Frenet坐标系下的路径点"""
    s: float           # 弧长 (m)
    l: float           # 横向偏移 (m)
    dl: float = 0.0    # dl/ds
    ddl: float = 0.0   # d²l/ds²


# =============================================================================
# 车辆状态
# =============================================================================

@dataclass
class VehicleState:
    """自车完整状态向量 (世界坐标系)"""
    x: float = 0.0        # 位置 X (m)
    y: float = 0.0        # 位置 Y (m)
    phi: float = 0.0      # 横摆角/航向角 (rad), 车轴与X轴夹角
    vx: float = 0.0       # 纵向速度 — 车体坐标系 (m/s)
    vy: float = 0.0       # 横向速度 — 车体坐标系 (m/s)
    r: float = 0.0        # 横摆角速度 (rad/s) = phi_dot
    steer: float = 0.0    # 当前前轮转角 (rad)
    accel: float = 0.0    # 当前加速度 (m/s²)
    timestamp: float = 0.0

    @property
    def speed(self) -> float:
        """合速度 (m/s)"""
        return np.sqrt(self.vx**2 + self.vy**2)

    @property
    def speed_kmh(self) -> float:
        """合速度 (km/h)"""
        return 3.6 * self.speed

    @property
    def world_velocity(self) -> Tuple[float, float]:
        """将车体坐标系速度转为世界坐标系 (Vx_world, Vy_world)"""
        Vx_w = self.vx * np.cos(self.phi) - self.vy * np.sin(self.phi)
        Vy_w = self.vx * np.sin(self.phi) + self.vy * np.cos(self.phi)
        return Vx_w, Vy_w

    @property
    def position(self) -> Tuple[float, float]:
        """位置 (x, y)"""
        return self.x, self.y


@dataclass
class VehicleParams:
    """车辆物理参数"""
    a: float = 1.015          # 质心到前轴距离 (m)
    b: float = 1.895          # 质心到后轴距离 (m)
    m: float = 1412.0         # 质量 (kg)
    Cf: float = -148970.0     # 前轮侧偏刚度 (N/rad, 负值)
    Cr: float = -82204.0      # 后轮侧偏刚度 (N/rad, 负值)
    Iz: float = 1537.0        # 绕Z轴转动惯量 (kg·m²)

    @property
    def wheelbase(self) -> float:
        return self.a + self.b


# =============================================================================
# 控制指令
# =============================================================================

@dataclass
class ControlCommand:
    """横纵向控制指令"""
    steer: float = 0.0     # 前轮转角 (rad), [-1, 1] 映射到物理转角
    throttle: float = 0.0  # 油门 [0, 1]
    brake: float = 0.0     # 刹车 [0, 1]
    gear: int = 1          # 档位: 1=前进, -1=倒车

    @classmethod
    def from_accel_steer(cls, accel: float, steer: float, dt: float = 0.05):
        """从加速度和转角生成控制指令 (兼容现有PID输出)"""
        c = cls(steer=steer)
        if accel >= 0:
            c.throttle = min(1.0, accel)
            c.brake = 0.0
        else:
            c.throttle = 0.0
            c.brake = min(1.0, abs(accel))
        return c


# =============================================================================
# 障碍物
# =============================================================================

@dataclass
class Obstacle:
    """障碍物描述"""
    id: int
    x: float
    y: float
    length: float = 4.5
    width: float = 2.0
    speed: float = 0.0         # 速度标量 (m/s)，沿heading方向; 0=静态
    heading: float = 0.0       # 运动方向 (rad)
    type: str = "vehicle"      # vehicle / pedestrian

    def step(self, dt: float):
        """更新障碍物位置 (匀速运动)"""
        if self.speed > 0:
            self.x += self.speed * np.cos(self.heading) * dt
            self.y += self.speed * np.sin(self.heading) * dt

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """返回障碍物 AABB (min_x, min_y, max_x, max_y) 用于碰撞检测"""
        hw = self.width / 2
        hl = self.length / 2
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        # 矩形四角在heading旋转后的投影范围
        corners = np.array([
            [-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]
        ])
        rot = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        rotated = corners @ rot.T + np.array([[self.x, self.y]])
        return (rotated[:, 0].min(), rotated[:, 1].min(),
                rotated[:, 0].max(), rotated[:, 1].max())


# =============================================================================
# 场景定义
# =============================================================================

@dataclass
class RoadSegment:
    """单段道路"""
    type: str = "waypoints"       # "straight" | "arc" | "waypoints"
    params: dict = field(default_factory=dict)
    lane_width: float = 3.5
    num_lanes: int = 2


@dataclass
class RoadDef:
    """道路定义"""
    segments: List[RoadSegment] = field(default_factory=list)
    lane_width: float = 3.5
    num_lanes: int = 2


@dataclass
class ScenarioConfig:
    """场景配置 (从YAML加载)"""
    name: str = "default"
    description: str = ""
    road: RoadDef = field(default_factory=RoadDef)
    ego_start_x: float = 20.0
    ego_start_y: float = 0.0
    ego_start_phi: float = 0.0
    ego_start_speed: float = 10.0   # m/s
    target_speed: float = 50.0      # km/h
    obstacles: List[dict] = field(default_factory=list)
    controller: str = "LQR_controller"
    planner: dict = field(default_factory=dict)
    destination: Optional[Tuple[float, float]] = None


# =============================================================================
# 仿真日志记录
# =============================================================================

@dataclass
class LogEntry:
    """单步仿真数据"""
    timestamp: float
    x: float
    y: float
    phi: float
    vx: float
    vy: float
    speed_kmh: float
    steer: float
    throttle: float
    brake: float
    ed: float
    ephi: float
    target_speed: float
