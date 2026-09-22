"""Framework-independent data contracts for parking planning and control."""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import math


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VehicleState:
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    steering: float = 0.0

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)


@dataclass(frozen=True)
class BoxObstacle:
    x: float
    y: float
    length: float
    width: float
    heading: float = 0.0


@dataclass(frozen=True)
class ParkingSlot:
    center_x: float
    center_y: float
    heading: float
    width: float = 4.5
    depth: float = 6.0

    @property
    def goal(self) -> Pose2D:
        return Pose2D(self.center_x, self.center_y, self.heading)


@dataclass(frozen=True)
class TrajectoryPoint:
    pose: Pose2D
    speed: float
    gear: int
    curvature: float = 0.0
    time_from_start: float = 0.0


@dataclass
class ParkingTrajectory:
    points: List[TrajectoryPoint] = field(default_factory=list)
    goal: Optional[Pose2D] = None
    planner_name: str = "reverse_parking_hermite"

    @property
    def empty(self) -> bool:
        return not self.points


@dataclass(frozen=True)
class ParkingConfig:
    wheelbase: float = 2.91
    vehicle_length: float = 4.9
    vehicle_width: float = 2.0
    obstacle_clearance: float = 0.15
    # Leave enough aisle distance to complete the forward 90-degree
    # alignment before changing to reverse.
    approach_distance: float = 16.0
    approach_speed: float = 0.8
    reverse_speed: float = 0.6
    sample_step: float = 0.2
    max_steer: float = 0.5
    position_tolerance: float = 0.6
    heading_tolerance: float = math.radians(15.0)
    speed_tolerance: float = 0.15


@dataclass(frozen=True)
class ControlCommand:
    steering: float = 0.0
    throttle: float = 0.0
    brake: float = 1.0
    gear: int = 0
