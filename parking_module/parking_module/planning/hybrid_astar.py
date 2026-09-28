"""Dependency-free hybrid A* planner for reverse-parking trajectories.

The lattice search is joined to the staging pose and parking slot with
collision-checked, steering-constrained Dubins segments. Search primitives are
resampled before they are handed to the shared parking controller.
"""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from ..core.geometry import collides
from ..core.types import (
    BoxObstacle,
    ParkingConfig,
    ParkingSlot,
    ParkingTrajectory,
    Pose2D,
    TrajectoryPoint,
    wrap_angle,
)
from .reverse_parking import (
    ParkingPlanningError,
    _dubins_candidates,
    _sample_dubins,
)


@dataclass(frozen=True)
class HybridAStarConfig:
    """Search discretization and cost settings."""

    xy_resolution: float = 1.0
    yaw_bins: int = 24
    step_size: float = 1.0
    collision_sample_step: float = 0.3
    search_margin: float = 18.0
    max_iterations: int = 16000
    reverse_penalty: float = 1.35
    gear_switch_penalty: float = 4.0
    steering_penalty: float = 0.15
    goal_position_tolerance: float = 1.0
    goal_heading_tolerance: float = math.radians(18.0)

    def __post_init__(self) -> None:
        if self.xy_resolution <= 0.0 or self.step_size <= 0.0:
            raise ValueError("hybrid A* resolutions must be positive")
        if self.collision_sample_step <= 0.0 or self.search_margin <= 0.0:
            raise ValueError("hybrid A* sampling settings must be positive")
        if self.yaw_bins < 8 or self.max_iterations <= 0:
            raise ValueError("hybrid A* discretization is too small")


@dataclass
class _SearchNode:
    pose: Pose2D
    gear: int
    curvature: float
    cost: float
    parent: Optional["_SearchNode"]
    edge_distance: float = 0.0


class HybridAStarPlanner:
    """Plan a sampled, collision-aware path with constrained gear segments."""

    def __init__(
        self,
        parking_config: ParkingConfig | None = None,
        search_config: HybridAStarConfig | None = None,
    ) -> None:
        self.config = parking_config or ParkingConfig()
        self.search = search_config or HybridAStarConfig()

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
        if collides(
            start, obstacles, cfg.vehicle_length, cfg.vehicle_width,
            cfg.obstacle_clearance,
        ):
            raise ParkingPlanningError("start pose collides with an obstacle")

        # Try the collision-checked analytic connection before expanding the
        # lattice.  A feasible Dubins shot from the actual start is both a
        # valid Hybrid A* solution and avoids accepting a much longer search
        # route whose final shot happens to curl around near the staging pose.
        start_node = _SearchNode(start, 1, 0.0, 0.0, None)
        approach = self._connect_forward(start_node, staging, obstacles)
        if approach is not None:
            approach = self._reconstruct(approach)
        else:
            approach = self._search_approach(start, staging, obstacles)
        if approach is None:
            raise ParkingPlanningError("hybrid A* could not find an approach path")
        points = self._approach_points(approach, cfg.approach_speed)
        self._append_reverse_segment(points, staging, slot, obstacles, cfg)
        if any(
            collides(
                point.pose, obstacles, cfg.vehicle_length, cfg.vehicle_width,
                cfg.obstacle_clearance,
            )
            for point in points
        ):
            raise ParkingPlanningError("hybrid A* path collides with an obstacle")
        return ParkingTrajectory(
            points=points,
            goal=slot.goal,
            planner_name="hybrid_astar_reverse_parking",
        )

    def _search_approach(
        self,
        start: Pose2D,
        goal: Pose2D,
        obstacles: Tuple[BoxObstacle, ...],
    ) -> Optional[List[_SearchNode]]:
        cfg = self.config
        search = self.search
        bounds = self._bounds(start, goal, obstacles)
        start_node = _SearchNode(start, 1, 0.0, 0.0, None)
        open_set = []
        counter = itertools.count()
        heapq.heappush(
            open_set,
            (self._heuristic(start, goal), next(counter), start_node),
        )
        best_cost: Dict[Tuple[int, int, int, int], float] = {
            self._key(start, 1, bounds): 0.0
        }
        steering_values = tuple(
            cfg.max_steer * ratio for ratio in (-1.0, -0.5, 0.0, 0.5, 1.0)
        )
        iterations = 0
        while open_set and iterations < search.max_iterations:
            _, _, current = heapq.heappop(open_set)
            iterations += 1
            if self._is_goal(current.pose, goal):
                connected = self._connect_forward(current, goal, obstacles)
                if connected is not None:
                    return self._reconstruct(connected)
            for gear in (1, -1):
                for steering in steering_values:
                    propagated = self._propagate(
                        current.pose, gear, steering, obstacles, bounds
                    )
                    if propagated is None:
                        continue
                    pose, curvature = propagated
                    key = self._key(pose, gear, bounds)
                    switch_cost = (
                        search.gear_switch_penalty
                        if gear != current.gear else 0.0
                    )
                    new_cost = (
                        current.cost + search.step_size
                        + (search.reverse_penalty if gear < 0 else 0.0)
                        + switch_cost
                        + search.steering_penalty
                        * abs(steering) / max(cfg.max_steer, 1e-6)
                    )
                    if new_cost >= best_cost.get(key, float("inf")):
                        continue
                    best_cost[key] = new_cost
                    node = _SearchNode(
                        pose, gear, curvature, new_cost, current,
                        search.step_size,
                    )
                    priority = new_cost + self._heuristic(pose, goal)
                    heapq.heappush(open_set, (priority, next(counter), node))
        return None

    def _propagate(self, pose, gear, steering, obstacles, bounds):
        cfg = self.config
        search = self.search
        substeps = max(1, int(math.ceil(search.step_size / search.collision_sample_step)))
        substep = search.step_size / substeps
        x, y, yaw = pose.x, pose.y, pose.yaw
        curvature = math.tan(steering) / cfg.wheelbase
        for _ in range(substeps):
            signed_distance = gear * substep
            next_yaw = wrap_angle(yaw + signed_distance * curvature)
            if abs(curvature) < 1e-9:
                x += signed_distance * math.cos(yaw)
                y += signed_distance * math.sin(yaw)
            else:
                x += (math.sin(next_yaw) - math.sin(yaw)) / curvature
                y += (math.cos(yaw) - math.cos(next_yaw)) / curvature
            candidate = Pose2D(x, y, next_yaw)
            if not self._inside_bounds(candidate, bounds):
                return None
            if collides(
                candidate, obstacles, cfg.vehicle_length, cfg.vehicle_width,
                cfg.obstacle_clearance,
            ):
                return None
            yaw = next_yaw
        return Pose2D(x, y, yaw), curvature

    def _approach_points(self, nodes: List[_SearchNode], speed: float):
        if not nodes:
            return []
        first_gear = nodes[1].gear if len(nodes) > 1 else 1
        points = [
            TrajectoryPoint(nodes[0].pose, first_gear * speed, first_gear, 0.0, 0.0)
        ]
        elapsed = 0.0
        previous = nodes[0]
        max_sample = max(0.05, self.config.sample_step)
        for node in nodes[1:]:
            gear = 1 if node.gear >= 0 else -1
            if gear != points[-1].gear:
                # Gear is attached to the outgoing segment. A duplicate pose
                # marks the exact place where the vehicle must stop and shift.
                points.append(
                    TrajectoryPoint(
                        previous.pose, 0.0, gear, 0.0, elapsed
                    )
                )
            distance = node.edge_distance or math.hypot(
                node.pose.x - previous.pose.x,
                node.pose.y - previous.pose.y,
            )
            count = max(1, int(math.ceil(distance / max_sample)))
            curvature = node.curvature
            for sample in range(1, count + 1):
                traveled = distance * sample / count
                pose = _integrate_distance(
                    previous.pose, gear * traveled, curvature
                )
                if sample == count:
                    pose = node.pose
                step_distance = math.hypot(
                    pose.x - points[-1].pose.x,
                    pose.y - points[-1].pose.y,
                )
                elapsed += step_distance / max(speed, 1e-3)
                points.append(
                    TrajectoryPoint(
                        pose, gear * speed, gear, curvature, elapsed
                    )
                )
            previous = node
        return points

    def _connect_forward(self, node, goal, obstacles):
        """Use a collision-checked Dubins shot to join the lattice to staging."""
        cfg = self.config
        position_error = math.hypot(goal.x - node.pose.x, goal.y - node.pose.y)
        heading_error = abs(wrap_angle(goal.yaw - node.pose.yaw))
        if position_error < 1e-6 and heading_error < 1e-6:
            if node.gear > 0:
                return node
            return _SearchNode(goal, 1, 0.0, node.cost, node, 0.0)

        max_curvature = 0.95 * math.tan(cfg.max_steer) / cfg.wheelbase
        for _length, word, lengths in _dubins_candidates(
            node.pose, goal, max_curvature
        ):
            poses = _sample_dubins(
                node.pose, goal, word, lengths, max_curvature,
                max(0.05, cfg.sample_step),
            )
            if any(
                collides(
                    pose, obstacles, cfg.vehicle_length, cfg.vehicle_width,
                    cfg.obstacle_clearance,
                )
                for pose in poses
            ):
                continue
            parent = node
            for previous, pose in zip(poses[:-1], poses[1:]):
                distance = math.hypot(
                    pose.x - previous.x, pose.y - previous.y
                )
                curvature = (
                    wrap_angle(pose.yaw - previous.yaw) / distance
                    if distance > 1e-6 else 0.0
                )
                parent = _SearchNode(
                    pose, 1, curvature,
                    parent.cost + distance, parent, distance,
                )
            if parent.pose == goal:
                return parent
        return None

    def _append_reverse_segment(self, points, staging, slot, obstacles, cfg):
        # In reverse, travel heading points opposite the vehicle body heading.
        travel_start = Pose2D(
            staging.x, staging.y, wrap_angle(staging.yaw + math.pi)
        )
        travel_goal = Pose2D(
            slot.center_x, slot.center_y,
            wrap_angle(slot.heading + math.pi),
        )
        max_curvature = 0.95 * math.tan(cfg.max_steer) / cfg.wheelbase
        selected = None
        for _length, word, lengths in _dubins_candidates(
            travel_start, travel_goal, max_curvature
        ):
            travel_poses = _sample_dubins(
                travel_start, travel_goal, word, lengths, max_curvature,
                max(0.05, cfg.sample_step),
            )
            body_poses = [
                Pose2D(pose.x, pose.y, wrap_angle(pose.yaw - math.pi))
                for pose in travel_poses
            ]
            if any(
                collides(
                    pose, obstacles, cfg.vehicle_length, cfg.vehicle_width,
                    cfg.obstacle_clearance,
                )
                for pose in body_poses
            ):
                continue
            selected = body_poses
            break
        if selected is None:
            raise ParkingPlanningError(
                "no collision-free, steering-feasible reverse path to the slot"
            )

        elapsed = points[-1].time_from_start if points else 0.0
        if not points or math.hypot(
            points[-1].pose.x - staging.x,
            points[-1].pose.y - staging.y,
        ) > 1e-6 or abs(wrap_angle(points[-1].pose.yaw - staging.yaw)) > 1e-6:
            raise ParkingPlanningError(
                "approach path did not connect exactly to the staging pose"
            )
        # Duplicate the staging pose to represent the stop-and-shift boundary.
        points.append(TrajectoryPoint(staging, 0.0, -1, 0.0, elapsed))
        previous = staging
        for index, pose in enumerate(selected[1:], start=1):
            distance = math.hypot(pose.x - previous.x, pose.y - previous.y)
            elapsed += distance / max(cfg.reverse_speed, 1e-3)
            signed_distance = -distance
            curvature = (
                wrap_angle(pose.yaw - previous.yaw) / signed_distance
                if distance > 1e-6 else 0.0
            )
            points.append(
                TrajectoryPoint(
                    pose, -cfg.reverse_speed, -1, curvature, elapsed
                )
            )
            previous = pose

    def _bounds(self, start, goal, obstacles):
        margin = self.search.search_margin
        xs = [start.x, goal.x]
        ys = [start.y, goal.y]
        for obstacle in obstacles:
            extent = math.hypot(obstacle.length, obstacle.width) / 2.0 + margin
            xs.extend((obstacle.x - extent, obstacle.x + extent))
            ys.extend((obstacle.y - extent, obstacle.y + extent))
        return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin

    @staticmethod
    def _inside_bounds(pose, bounds):
        return bounds[0] <= pose.x <= bounds[1] and bounds[2] <= pose.y <= bounds[3]

    def _key(self, pose, gear, bounds):
        search = self.search
        x_bin = int(round((pose.x - bounds[0]) / search.xy_resolution))
        y_bin = int(round((pose.y - bounds[2]) / search.xy_resolution))
        yaw_bin = int(_mod2pi(pose.yaw) / (2.0 * math.pi) * search.yaw_bins) % search.yaw_bins
        return x_bin, y_bin, yaw_bin, gear

    def _heuristic(self, pose, goal):
        return math.hypot(goal.x - pose.x, goal.y - pose.y) + 0.5 * abs(
            wrap_angle(goal.yaw - pose.yaw)
        )

    def _is_goal(self, pose, goal):
        return (
            math.hypot(goal.x - pose.x, goal.y - pose.y)
            <= self.search.goal_position_tolerance
            and abs(wrap_angle(goal.yaw - pose.yaw))
            <= self.search.goal_heading_tolerance
        )

    @staticmethod
    def _reconstruct(node):
        path = []
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse()
        return path


def _mod2pi(angle: float) -> float:
    return float(angle) % (2.0 * math.pi)


def _integrate_distance(pose: Pose2D, signed_distance: float, curvature: float) -> Pose2D:
    """Integrate a constant-curvature bicycle motion over signed distance."""
    yaw = wrap_angle(pose.yaw + signed_distance * curvature)
    if abs(curvature) < 1e-9:
        x = pose.x + signed_distance * math.cos(pose.yaw)
        y = pose.y + signed_distance * math.sin(pose.yaw)
    else:
        x = pose.x + (math.sin(yaw) - math.sin(pose.yaw)) / curvature
        y = pose.y + (math.cos(pose.yaw) - math.cos(yaw)) / curvature
    return Pose2D(x, y, yaw)


__all__ = ["HybridAStarConfig", "HybridAStarPlanner"]
