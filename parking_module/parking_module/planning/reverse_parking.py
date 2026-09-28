"""Deterministic reverse-parking planner.

The planner deliberately depends only on the standalone core contracts.  It
is a small, deterministic baseline that can later be replaced by Hybrid A*
without changing the controller or ROS adapter interfaces.
"""

from __future__ import annotations

import math
from typing import Iterable, List

from ..core.geometry import (
    collides,
    derivative_heading,
    hermite_derivative,
    hermite_point,
)
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


def _mod2pi(angle: float) -> float:
    return float(angle) % (2.0 * math.pi)


def _dubins_candidates(start: Pose2D, goal: Pose2D, curvature: float):
    """Return forward-only Dubins candidates in increasing path length."""

    if curvature <= 0.0 or not math.isfinite(curvature):
        return []
    dx = goal.x - start.x
    dy = goal.y - start.y
    distance = math.hypot(dx, dy)
    # Normalize the goal into a frame whose origin and heading are the start.
    local_x = math.cos(start.yaw) * dx + math.sin(start.yaw) * dy
    local_y = -math.sin(start.yaw) * dx + math.cos(start.yaw) * dy
    theta = math.atan2(local_y, local_x) if distance > 1e-9 else 0.0
    alpha = _mod2pi(-theta)
    beta = _mod2pi(goal.yaw - start.yaw - theta)
    d = distance * curvature
    candidates = []

    def add(word, values):
        if values is not None and all(value >= -1e-9 for value in values):
            values = tuple(max(0.0, float(value)) for value in values)
            candidates.append((sum(values), word, values))

    # The six standard Dubins words.  Segment lengths are normalized by the
    # minimum turning radius and converted to metres by the sampler.
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p2 = 2.0 + d * d - 2.0 * math.cos(alpha - beta) + 2.0 * d * (
        math.sin(alpha) - math.sin(beta)
    )
    if p2 >= 0.0:
        tmp1 = math.atan2(math.cos(beta) - math.cos(alpha), tmp0)
        add("LSL", (_mod2pi(-alpha + tmp1), math.sqrt(p2), _mod2pi(beta - tmp1)))

    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p2 = 2.0 + d * d - 2.0 * math.cos(alpha - beta) + 2.0 * d * (
        -math.sin(alpha) + math.sin(beta)
    )
    if p2 >= 0.0:
        tmp1 = math.atan2(math.cos(alpha) - math.cos(beta), tmp0)
        add("RSR", (_mod2pi(alpha - tmp1), math.sqrt(p2), _mod2pi(-beta + tmp1)))

    p2 = -2.0 + d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (
        math.sin(alpha) + math.sin(beta)
    )
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp2 = math.atan2(
            -math.cos(alpha) - math.cos(beta),
            d + math.sin(alpha) + math.sin(beta),
        ) - math.atan2(-2.0, p)
        add("LSR", (_mod2pi(-alpha + tmp2), p, _mod2pi(-beta + tmp2)))

    p2 = d * d - 2.0 + 2.0 * math.cos(alpha - beta) - 2.0 * d * (
        math.sin(alpha) + math.sin(beta)
    )
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp2 = math.atan2(
            math.cos(alpha) + math.cos(beta),
            d - math.sin(alpha) - math.sin(beta),
        ) - math.atan2(2.0, p)
        add("RSL", (_mod2pi(alpha - tmp2), p, _mod2pi(beta - tmp2)))

    tmp = (
        6.0 - d * d + 2.0 * math.cos(alpha - beta)
        + 2.0 * d * (math.sin(alpha) - math.sin(beta))
    ) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(tmp))
        tmp2 = math.atan2(
            math.cos(alpha) - math.cos(beta),
            d - math.sin(alpha) + math.sin(beta),
        )
        add("RLR", (_mod2pi(alpha - tmp2 + p / 2.0), p,
                     _mod2pi(alpha - beta - _mod2pi(alpha - tmp2 + p / 2.0) + p)))

    tmp = (
        6.0 - d * d + 2.0 * math.cos(alpha - beta)
        + 2.0 * d * (-math.sin(alpha) + math.sin(beta))
    ) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(tmp))
        tmp2 = math.atan2(
            math.cos(alpha) - math.cos(beta),
            d + math.sin(alpha) - math.sin(beta),
        )
        t = _mod2pi(-alpha - tmp2 + p / 2.0)
        add("LRL", (t, p, _mod2pi(beta - alpha + t + p)))

    candidates.sort(key=lambda item: item[0])
    return candidates


def _sample_dubins(
    start: Pose2D,
    goal: Pose2D,
    word: str,
    normalized_lengths,
    curvature: float,
    step: float,
):
    """Sample a Dubins path and return poses with forward travel headings."""

    radius = 1.0 / curvature
    x = y = yaw = 0.0
    local = [Pose2D(0.0, 0.0, 0.0)]
    for mode, normalized_length in zip(word, normalized_lengths):
        remaining = normalized_length * radius
        sign = 1.0 if mode == "L" else -1.0 if mode == "R" else 0.0
        while remaining > 1e-9:
            distance = min(float(step), remaining)
            if mode == "S":
                x += distance * math.cos(yaw)
                y += distance * math.sin(yaw)
            else:
                next_yaw = yaw + sign * curvature * distance
                x += (math.sin(next_yaw) - math.sin(yaw)) / (sign * curvature)
                y += (-math.cos(next_yaw) + math.cos(yaw)) / (sign * curvature)
                yaw = next_yaw
            local.append(Pose2D(x, y, yaw))
            remaining -= distance

    cos_yaw = math.cos(start.yaw)
    sin_yaw = math.sin(start.yaw)
    poses = [
        Pose2D(
            start.x + cos_yaw * pose.x - sin_yaw * pose.y,
            start.y + sin_yaw * pose.x + cos_yaw * pose.y,
            pose.yaw + start.yaw,
        )
        for pose in local
    ]
    if not poses:
        poses = [start]
    poses[0] = start
    poses[-1] = goal
    return poses


def _sampled_curvatures(poses: List[Pose2D]) -> List[float]:
    curvatures = [0.0]
    for previous, current in zip(poses[:-1], poses[1:]):
        distance = math.hypot(current.x - previous.x, current.y - previous.y)
        curvatures.append(
            wrap_angle(current.yaw - previous.yaw) / distance
            if distance > 1e-6 else 0.0
        )
    if len(curvatures) > 1:
        curvatures[0] = curvatures[1]
    return curvatures


class ReverseParkingPlanner:
    """Generate a curvature-feasible approach and reverse parking maneuver."""

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
        approach_distance = math.hypot(staging.x - start.x, staging.y - start.y)

        points: List[TrajectoryPoint] = []
        max_curvature = 0.95 * math.tan(cfg.max_steer) / cfg.wheelbase
        candidates = _dubins_candidates(start, staging, max_curvature)
        approach_poses = None
        approach_curvatures = None
        for _length, word, normalized_lengths in candidates:
            candidate = _sample_dubins(
                start, staging, word, normalized_lengths, max_curvature, cfg.sample_step
            )
            curvature = _sampled_curvatures(candidate)
            # Sampling and forcing the exact endpoint introduce a small
            # numerical curvature error at segment joins.
            if max(abs(value) for value in curvature) > max_curvature + 1e-4:
                continue
            if any(
                collides(
                    pose, obstacles, cfg.vehicle_length, cfg.vehicle_width,
                    cfg.obstacle_clearance,
                )
                for pose in candidate
            ):
                continue
            approach_poses = candidate
            approach_curvatures = curvature
            break
        if approach_poses is None:
            if not candidates:
                raise ParkingPlanningError("no curvature-feasible approach path")
            raise ParkingPlanningError("no collision-free approach path")
        approach_time = 0.0
        previous = start
        for index, pose in enumerate(approach_poses):
            if index:
                approach_time += math.hypot(pose.x - previous.x, pose.y - previous.y) / cfg.approach_speed
            previous = pose
            # Keep the approach target moving.  The controller owns the final
            # stop at the staging pose so it can make the gear transition
            # deterministically instead of braking one lookahead distance too
            # early.
            speed = cfg.approach_speed
            points.append(
                TrajectoryPoint(
                    pose,
                    speed,
                    1,
                    curvature=approach_curvatures[index],
                    time_from_start=approach_time,
                )
            )

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
