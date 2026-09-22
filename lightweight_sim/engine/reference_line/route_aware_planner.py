"""Obstacle-aware local planner expressed relative to a routed lane centerline."""

from __future__ import annotations

import math

from ..algorithms.planner.motion_planner import MotionPlanner
from ..algorithms.utils.frenet import find_match_points


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
    ) -> None:
        super().__init__(global_frenet_path, lane_width=lane_width, num_lanes=num_lanes)
        self.reference_lane_index = int(reference_lane_index)
        self.target_lane = int(target_lane)

    def _plan(self, pred_loc, vehicle_loc, obstacles):
        path = self.global_path
        if len(path) < 2:
            return []

        idx, _ = find_match_points([pred_loc], path, True, 0)
        start = max(0, int(idx[0]))
        horizon = min(len(path), start + 80)
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
        usable = self.num_lanes * self.lane_width / 2.0 - 1.1
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
                not (-5 <= index_delta <= 65 and abs(target - lateral) < 2.2)
                for index_delta, lateral in obstacle_sl
            )

        valid = [candidate for candidate in candidates if is_safe(candidate)]
        preferred = (
            self.lane_width * (self.target_lane - self.reference_lane_index)
            if 0 <= self.target_lane < self.num_lanes
            else l0
        )
        if not is_safe(preferred):
            preferred = l0
        target = min(
            valid or [max(-usable, min(usable, l0))],
            key=lambda candidate: (abs(candidate - preferred), abs(candidate - l0)),
        )

        result = []
        travelled = 0.0
        transition_distance = 35.0
        for index, point in enumerate(ref):
            if index > 0:
                previous = ref[index - 1]
                travelled += math.hypot(
                    point[0] - previous[0], point[1] - previous[1]
                )
            ratio = max(0.0, min(1.0, travelled / transition_distance))
            smooth = ratio * ratio * (3.0 - 2.0 * ratio)
            lateral = l0 + (target - l0) * smooth
            normal = (-math.sin(point[2]), math.cos(point[2]))
            result.append(
                (
                    point[0] + lateral * normal[0],
                    point[1] + lateral * normal[1],
                    point[2],
                    point[3],
                )
            )
        return result
