"""Deterministic reverse-parking planner.

The planner deliberately depends only on the standalone core contracts.  It
is a small, deterministic baseline that can later be replaced by Hybrid A*
without changing the controller or ROS adapter interfaces.
"""

from __future__ import annotations

import math
from typing import Iterable, List

from ..core.geometry import collides, derivative_heading, hermite_derivative, hermite_point
from ..core.types import (
    BoxObstacle,
    ParkingConfig,
    ParkingSlot,
    ParkingTrajectory,
    Pose2D,
    TrajectoryPoint,
    wrap_angle,
)


class ParkingPlanningError(RuntimeError):
    """Raised when no collision-free parking trajectory can be produced."""


class ReverseParkingPlanner:
    """Generate an approach segment followed by a reverse Hermite maneuver."""

    def __init__(self, config: ParkingConfig | None = None):
        self.config = config or ParkingConfig()

    def plan(
        self,
        start: Pose2D,
        slot: ParkingSlot,
        obstacles: Iterable[BoxObstacle] = (),
    ) -> ParkingTrajectory:
        obstacles = tuple(obstacles)
        cfg = self.config
        staging = Pose2D(
            slot.center_x + cfg.approach_distance * math.cos(slot.heading),
            slot.center_y + cfg.approach_distance * math.sin(slot.heading),
            slot.heading,
        )

        points: List[TrajectoryPoint] = []
        approach_distance = math.hypot(staging.x - start.x, staging.y - start.y)
        approach_scale = max(4.0, min(8.0, approach_distance * 0.75))
        approach_m0 = (
            approach_scale * math.cos(start.yaw),
            approach_scale * math.sin(start.yaw),
        )
        approach_m1 = (
            approach_scale * math.cos(staging.yaw),
            approach_scale * math.sin(staging.yaw),
        )
        approach_count = max(2, int(math.ceil(approach_distance / cfg.sample_step)))
        approach_time = 0.0
        previous = (start.x, start.y)
        for index in range(approach_count):
            t = index / (approach_count - 1)
            x, y = hermite_point(
                (start.x, start.y),
                (staging.x, staging.y),
                approach_m0,
                approach_m1,
                t,
            )
            derivative = hermite_derivative(
                (start.x, start.y),
                (staging.x, staging.y),
                approach_m0,
                approach_m1,
                t,
            )
            yaw = derivative_heading(derivative, reverse=False)
            if index:
                approach_time += math.hypot(x - previous[0], y - previous[1]) / max(cfg.approach_speed, 1e-3)
            previous = (x, y)
            # Keep the approach target moving.  The controller owns the final
            # stop at the staging pose so it can make the gear transition
            # deterministically instead of braking one lookahead distance too
            # early.
            speed = cfg.approach_speed
            points.append(TrajectoryPoint(Pose2D(x, y, yaw), speed, 1, time_from_start=approach_time))

        # The derivatives describe the direction of travel.  At both ends the
        # vehicle moves opposite to its body heading while reversing.
        tangent_scale = max(4.0, min(8.0, approach_distance * 0.75))
        m0 = (-tangent_scale * math.cos(staging.yaw), -tangent_scale * math.sin(staging.yaw))
        m1 = (-tangent_scale * math.cos(slot.heading), -tangent_scale * math.sin(slot.heading))
        reverse_distance = math.hypot(slot.center_x - staging.x, slot.center_y - staging.y)
        reverse_count = max(3, int(math.ceil(reverse_distance * 2.5 / cfg.sample_step)))
        time_offset = points[-1].time_from_start if points else 0.0
        previous = points[-1].pose
        for index in range(reverse_count):
            t = index / (reverse_count - 1)
            x, y = hermite_point((staging.x, staging.y), (slot.center_x, slot.center_y), m0, m1, t)
            derivative = hermite_derivative((staging.x, staging.y), (slot.center_x, slot.center_y), m0, m1, t)
            yaw = derivative_heading(derivative, reverse=True)
            pose = Pose2D(x, y, yaw)
            distance = math.hypot(pose.x - previous.x, pose.y - previous.y)
            curvature = 0.0 if distance < 1e-6 else wrap_angle(pose.yaw - previous.yaw) / distance
            speed = 0.0 if index == 0 else -cfg.reverse_speed
            time_offset += distance / max(cfg.reverse_speed, 1e-3)
            points.append(TrajectoryPoint(pose, speed, -1, curvature, time_offset))
            previous = pose

        # Collision checking is performed against the full footprint, not the
        # reference point, so the returned trajectory is safe to consume by a
        # downstream controller without knowing the planner internals.
        for index, point in enumerate(points):
            if collides(point.pose, obstacles, cfg.vehicle_length, cfg.vehicle_width, cfg.obstacle_clearance):
                raise ParkingPlanningError(f"trajectory collides with obstacle at point {index}")

        return ParkingTrajectory(points=points, goal=slot.goal)
