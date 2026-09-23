"""Obstacle-aware local planner expressed relative to a routed lane centerline."""

from __future__ import annotations

import math

from ..algorithms.planner.motion_planner import MotionPlanner
from ..algorithms.utils.frenet import find_match_points
from ..algorithms.utils.geometry import cal_heading_kappa


class RouteAwareMotionPlanner(MotionPlanner):
    """Use routing lane identity to make local detours return to the mission lane."""

    def __init__(
        self,
        global_frenet_path,
        *,
        lane_width: float,
        num_lanes: int,
        reference_lane_index: int,
        target_lane: int,
        drivable_left_boundary=(),
        drivable_right_boundary=(),
        corridor_margin_m: float = 1.1,
        horizon_points: int = 80,
        transition_distance_m: float = 12.0,
        collision_margin_m: float = 0.25,
        obstacle_longitudinal_min_m: float = -5.0,
        obstacle_longitudinal_max_m: float = 65.0,
        obstacle_lateral_clearance_m: float = 2.2,
        vehicle_length_m: float = 4.0,
        vehicle_width_m: float = 2.0,
    ) -> None:
        super().__init__(
            global_frenet_path,
            lane_width=lane_width,
            num_lanes=num_lanes,
            horizon_points=horizon_points,
            corridor_margin_m=corridor_margin_m,
            transition_distance_m=transition_distance_m,
            obstacle_longitudinal_min_m=obstacle_longitudinal_min_m,
            obstacle_longitudinal_max_m=obstacle_longitudinal_max_m,
            obstacle_lateral_clearance_m=obstacle_lateral_clearance_m,
        )
        if len(drivable_left_boundary) != len(drivable_right_boundary):
            raise ValueError("drivable boundaries must have matching point counts")
        if drivable_left_boundary and len(drivable_left_boundary) != len(global_frenet_path):
            raise ValueError("drivable boundaries must match the routing reference")
        if corridor_margin_m < 0.0:
            raise ValueError("corridor margin must be non-negative")
        if transition_distance_m <= 0.0:
            raise ValueError("transition distance must be positive")
        if collision_margin_m < 0.0:
            raise ValueError("collision margin must be non-negative")
        if vehicle_length_m <= 0.0 or vehicle_width_m <= 0.0:
            raise ValueError("vehicle dimensions must be positive")
        self.reference_lane_index = int(reference_lane_index)
        self.target_lane = int(target_lane)
        self.drivable_left_boundary = tuple(drivable_left_boundary)
        self.drivable_right_boundary = tuple(drivable_right_boundary)
        self.corridor_margin_m = float(corridor_margin_m)
        self.transition_distance_m = float(transition_distance_m)
        self.collision_margin_m = float(collision_margin_m)
        self.vehicle_length_m = float(vehicle_length_m)
        self.vehicle_width_m = float(vehicle_width_m)

    def _plan(self, pred_loc, vehicle_loc, obstacles):
        path = self.global_path
        if len(path) < 2:
            return []

        idx, _ = find_match_points([pred_loc], path, True, 0)
        start = max(0, int(idx[0]))
        horizon = min(len(path), start + self.horizon_points)
        ref = path[start:horizon]
        theta = ref[0][2]
        normal = (-math.sin(theta), math.cos(theta))
        l0 = normal[0] * (vehicle_loc[0] - ref[0][0]) + normal[1] * (
            vehicle_loc[1] - ref[0][1]
        )

        candidates = [
            self.lane_width * (lane_index - self.reference_lane_index)
            for lane_index in range(self.num_lanes)
        ]
        usable = self.num_lanes * self.lane_width / 2.0 - self.corridor_margin_m
        candidates.append(max(-usable, min(usable, l0)))
        candidates = sorted(set(round(value, 3) for value in candidates))

        obstacle_sl = []
        for ox, oy, length, width, speed, heading in obstacles:
            del length, width, speed, heading
            obstacle_index, _ = find_match_points([(ox, oy)], path, True, 0)
            index = max(0, min(len(path) - 1, int(obstacle_index[0])))
            reference = path[index]
            normal = (-math.sin(reference[2]), math.cos(reference[2]))
            lateral = (ox - reference[0]) * normal[0] + (
                oy - reference[1]
            ) * normal[1]
            obstacle_sl.append((index - start, lateral))

        def is_safe(target):
            return all(
                not (
                    self.obstacle_longitudinal_min_m
                    <= index_delta
                    <= self.obstacle_longitudinal_max_m
                    and abs(target - lateral) < self.obstacle_lateral_clearance_m
                )
                for index_delta, lateral in obstacle_sl
            )

        def is_in_corridor(target):
            return all(
                lower <= target <= upper
                for index, point in enumerate(ref)
                for lower, upper in [self._corridor_bounds(start + index, point)]
            )

        preferred = (
            self.lane_width * (self.target_lane - self.reference_lane_index)
            if 0 <= self.target_lane < self.num_lanes
            else l0
        )
        if not is_safe(preferred):
            preferred = l0

        candidates_with_paths = []
        for candidate in candidates:
            if not is_safe(candidate) or not is_in_corridor(candidate):
                continue
            candidate_path = self._build_candidate(ref, start, l0, candidate)
            if self._trajectory_is_safe(candidate_path, obstacles):
                candidates_with_paths.append((candidate, candidate_path))

        if not candidates_with_paths:
            self.logger.warning(
                "no collision-free local trajectory; start=%d obstacles=%d",
                start,
                len(obstacles),
            )
            return []

        _, result = min(
            candidates_with_paths,
            key=lambda item: (
                abs(item[0] - preferred),
                abs(item[0] - l0),
            ),
        )
        return result

    def _build_candidate(self, ref, start, l0, target):
        xy_points = []
        travelled = 0.0
        for index, point in enumerate(ref):
            if index > 0:
                previous = ref[index - 1]
                travelled += math.hypot(
                    point[0] - previous[0], point[1] - previous[1]
                )
            ratio = max(0.0, min(1.0, travelled / self.transition_distance_m))
            smooth = ratio * ratio * (3.0 - 2.0 * ratio)
            lateral = l0 + (target - l0) * smooth
            lower, upper = self._corridor_bounds(start + index, point)
            lateral = max(lower, min(upper, lateral))
            normal = (-math.sin(point[2]), math.cos(point[2]))
            xy_points.append(
                (point[0] + lateral * normal[0], point[1] + lateral * normal[1])
            )
        headings, curvatures = cal_heading_kappa(xy_points)
        return [
            (x, y, heading, curvature)
            for (x, y), heading, curvature in zip(
                xy_points, headings, curvatures
            )
        ]

    def _trajectory_is_safe(self, candidate_path, obstacles):
        ego_half_length = self.vehicle_length_m / 2.0
        ego_half_width = self.vehicle_width_m / 2.0
        for px, py, heading, _ in candidate_path:
            tangent = (math.cos(heading), math.sin(heading))
            normal = (-math.sin(heading), math.cos(heading))
            for ox, oy, length, width, speed, obstacle_heading in obstacles:
                del speed
                dx = ox - px
                dy = oy - py
                longitudinal = abs(dx * tangent[0] + dy * tangent[1])
                lateral = abs(dx * normal[0] + dy * normal[1])
                heading_delta = obstacle_heading - heading
                obstacle_half_length = (
                    abs(math.cos(heading_delta)) * length / 2.0
                    + abs(math.sin(heading_delta)) * width / 2.0
                )
                obstacle_half_width = (
                    abs(math.sin(heading_delta)) * length / 2.0
                    + abs(math.cos(heading_delta)) * width / 2.0
                )
                if (
                    longitudinal
                    <= ego_half_length + obstacle_half_length + self.collision_margin_m
                    and lateral
                    <= ego_half_width + obstacle_half_width + self.collision_margin_m
                ):
                    return False
        return True

    def _corridor_bounds(self, index, reference):
        if not self.drivable_left_boundary:
            usable = self.num_lanes * self.lane_width / 2.0 - self.corridor_margin_m
            return -usable, usable
        left = self.drivable_left_boundary[index]
        right = self.drivable_right_boundary[index]
        normal = (-math.sin(reference[2]), math.cos(reference[2]))
        left_lateral = (left[0] - reference[0]) * normal[0] + (
            left[1] - reference[1]
        ) * normal[1]
        right_lateral = (right[0] - reference[0]) * normal[0] + (
            right[1] - reference[1]
        ) * normal[1]
        lower = min(left_lateral, right_lateral) + self.corridor_margin_m
        upper = max(left_lateral, right_lateral) - self.corridor_margin_m
        if lower > upper:
            raise ValueError("drivable corridor is narrower than twice the margin")
        return lower, upper
