"""Shared data types for the lightweight simulator."""
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import math
from .steering import SteeringParams
from ..runtime_config import DEFAULT_RUNTIME_CONFIG

@dataclass
class PathPoint:
    x: float
    y: float
    theta: float
    kappa: float
    def __iter__(self):
        return iter((self.x, self.y, self.theta, self.kappa))

@dataclass
class FrenetPoint:
    s: float
    l: float
    dl: float = 0.0
    ddl: float = 0.0

@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    phi: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    r: float = 0.0
    steer: float = 0.0
    accel: float = 0.0
    timestamp: float = 0.0
    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)
    @property
    def speed_kmh(self) -> float:
        return self.speed * 3.6
    @property
    def world_velocity(self) -> Tuple[float, float]:
        c, s = math.cos(self.phi), math.sin(self.phi)
        return self.vx * c - self.vy * s, self.vx * s + self.vy * c
    @property
    def position(self) -> Tuple[float, float]:
        return self.x, self.y

@dataclass
class VehicleParams:
    a: float = 1.015
    b: float = 1.895
    m: float = 1412.0
    Cf: float = -148970.0
    Cr: float = -82204.0
    Iz: float = 1537.0
    max_steer: float = 0.5
    max_accel: float = 3.0
    max_decel: float = 6.0
    width: float = 2.0
    body_overhang: float = 1.0
    def __post_init__(self):
        positive = (
            self.a,
            self.b,
            self.m,
            self.Iz,
            self.max_steer,
            self.max_accel,
            self.max_decel,
            self.width,
            self.body_overhang,
        )
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError("vehicle geometry, mass, inertia and limits must be positive and finite")
        if not all(math.isfinite(v) and v < 0 for v in (self.Cf, self.Cr)):
            raise ValueError("cornering stiffness uses the negative-force sign convention")
    @property
    def lateral_tuple(self):
        return (self.a, self.b, self.m, self.Cf, self.Cr, self.Iz)
    @property
    def wheelbase(self) -> float:
        return self.a + self.b

@dataclass
class ControlCommand:
    steer: float = 0.0       # physical front-wheel angle in radians
    throttle: float = 0.0    # normalized [0, 1]
    brake: float = 0.0       # normalized [0, 1]
    gear: int = 1

@dataclass
class Obstacle:
    id: int
    x: float
    y: float
    length: float = 4.5
    width: float = 2.0
    speed: float = 0.0
    heading: float = 0.0
    type: str = "vehicle"
    def step(self, dt: float):
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

@dataclass
class RoadSegment:
    type: str = "waypoints"
    params: dict = field(default_factory=dict)
    lane_width: float = DEFAULT_RUNTIME_CONFIG.lane_width
    num_lanes: int = DEFAULT_RUNTIME_CONFIG.num_lanes

@dataclass
class RoadDef:
    segments: List[RoadSegment] = field(default_factory=list)
    lane_width: float = DEFAULT_RUNTIME_CONFIG.lane_width
    num_lanes: int = DEFAULT_RUNTIME_CONFIG.num_lanes

@dataclass
class ScenarioConfig:
    name: str = "default"
    description: str = ""
    road: RoadDef = field(default_factory=RoadDef)
    ego_start_x: float = 20.0
    ego_start_y: float = 0.0
    ego_start_phi: float = 0.0
    ego_start_speed: float = 10.0
    target_speed: float = 50.0
    speed_limit_type: str = "straight"
    speed_limit_kmh: float = 40.0
    target_speed_ratio: float = 0.85
    speed_limits: dict = field(default_factory=dict)
    max_lateral_accel_mps2: float = 2.0
    obstacles: List[dict] = field(default_factory=list)
    controller: str = DEFAULT_RUNTIME_CONFIG.controller
    planner: dict = field(default_factory=dict)
    destination: Optional[Tuple[float, float]] = None
    routing_map_id: Optional[str] = None
    routing_start_lane: int = -1
    routing_goal_lane: int = -1
    vehicle_model: str = "kinematic"
    maneuver: str = "cruise"
    parking_goal: Optional[Tuple[float, float, float]] = None
    vehicle_params: VehicleParams = field(default_factory=VehicleParams)
    steering: SteeringParams = field(default_factory=SteeringParams)
    physics_dt: float = DEFAULT_RUNTIME_CONFIG.physics_dt

@dataclass
class LogEntry:
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
