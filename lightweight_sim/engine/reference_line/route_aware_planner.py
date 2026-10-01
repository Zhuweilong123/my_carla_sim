"""Obstacle-aware local planner expressed relative to a routed lane centerline."""

from __future__ import annotations

import math

from ..algorithms.planner.motion_planner import MotionPlanner
from ..algorithms.utils.frenet import find_match_points
from ..algorithms.utils.quintic import sample_quintic_path
from ..runtime_config import DEFAULT_RUNTIME_CONFIG


class CorridorMotionPlanner(MotionPlanner):
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
        transition_distance_m: float = DEFAULT_RUNTIME_CONFIG.local_transition_distance_m,
        sampling_resolution_m: float = DEFAULT_RUNTIME_CONFIG.local_path_sampling_resolution_m,
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
            sampling_resolution_m=sampling_resolution_m,
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

    def _build_candidate(self, ref, indices, l0, target):
        return sample_quintic_path(
            ref, l0, target, self.transition_distance_m, self.sampling_resolution_m,
            lateral_bounds=[self._corridor_bounds(index, point)
                            for index, point in zip(indices, ref)],
        )

    def _trajectory_is_safe(self, candidate_path, obstacles):
        """Separating-axis rectangle test using both bodies' axes."""
        for px, py, heading, _ in candidate_path:
            ego_axes = ((math.cos(heading), math.sin(heading)),
                        (-math.sin(heading), math.cos(heading)))
            for ox, oy, length, width, _, obstacle_heading in obstacles:
                obstacle_axes = ((math.cos(obstacle_heading), math.sin(obstacle_heading)),
                                 (-math.sin(obstacle_heading), math.cos(obstacle_heading)))
                separated = False
                for ax, ay in ego_axes + obstacle_axes:
                    distance = abs((ox-px)*ax+(oy-py)*ay)
                    ego_radius = (self.vehicle_length_m/2*abs(ax*ego_axes[0][0]+ay*ego_axes[0][1])
                                  +self.vehicle_width_m/2*abs(ax*ego_axes[1][0]+ay*ego_axes[1][1]))
                    obstacle_radius = (length/2*abs(ax*obstacle_axes[0][0]+ay*obstacle_axes[0][1])
                                       +width/2*abs(ax*obstacle_axes[1][0]+ay*obstacle_axes[1][1]))
                    if distance > ego_radius+obstacle_radius+self.collision_margin_m:
                        separated = True
                        break
                if not separated:
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


class BaselinePathPlanner(CorridorMotionPlanner):
    """Lane-target quintic candidate algorithm retained for comparisons."""

    def _plan(self, pred_loc, vehicle_loc, obstacles):
        path = self.global_path
        if len(path) < 2:
            return []

        idx, projections = find_match_points([pred_loc], path, True, 0)
        start = max(0, int(idx[0]))
        closed = math.hypot(path[0][0] - path[-1][0], path[0][1] - path[-1][1]) < 1e-6
        cycle_size = len(path) - 1 if closed else len(path)
        start = min(start, cycle_size - 1)
        indices = [
            (start + offset) % cycle_size if closed else start + offset
            for offset in range(min(self.horizon_points, cycle_size - start if not closed else cycle_size))
        ]
        if closed and len(indices) < min(self.horizon_points, cycle_size):
            indices.extend(
                range(0, min(self.horizon_points, cycle_size) - len(indices))
            )
        ref = [path[index] for index in indices]
        if len(ref) < 2:
            return []
        # Anchor the transition at the projected planning start, not a sparse
        # reference vertex which can lie far behind the vehicle.
        ref[0] = projections[0]
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
            index_delta = index - start
            if closed:
                index_delta %= cycle_size
                if index_delta > cycle_size / 2.0:
                    index_delta -= cycle_size
            obstacle_sl.append((index_delta, lateral))

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
                for index, point in zip(indices, ref)
                for lower, upper in [self._corridor_bounds(index, point)]
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
            candidate_path = self._build_candidate(ref, indices, l0, candidate)
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


# Compatibility for callers of the original candidate planner.
RouteAwareMotionPlanner = BaselinePathPlanner
