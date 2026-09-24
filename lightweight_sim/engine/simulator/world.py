"""Road geometry with safe joining of consecutive segments."""

import math

from ..algorithms.utils.geometry import cal_heading_kappa
from .data_types import PathPoint
from ..reference_line.core import _round_polyline_corners
from ._world_impl import *  # noqa: F401,F403


_LegacyWorld = World


class World(_LegacyWorld):
    """Keep coincident segment endpoints without flagging them as gaps."""

    def __init__(self, road_def=None):
        super().__init__(road_def)
        self.reference_lane_index = (
            self.road_def.reference_lane_index
            if 0 <= self.road_def.reference_lane_index < self.num_lanes
            else -1
        )

    def get_lane_center(self, lane_idx):
        if self.reference_lane_index >= 0:
            return (lane_idx - self.reference_lane_index) * self.lane_width
        return super().get_lane_center(lane_idx)

    def get_lane_boundaries(self):
        if self.reference_lane_index >= 0:
            return [
                (index - self.reference_lane_index - 0.5) * self.lane_width
                for index in range(self.num_lanes + 1)
            ]
        return super().get_lane_boundaries()

    def _signed_lateral_to_reference(self, x, y):
        """Return left-positive lateral distance to the closest path segment."""
        if len(self._ref_path) < 2:
            return 0.0
        best_distance_sq = float("inf")
        best_lateral = 0.0
        for first, second in zip(self._ref_path[:-1], self._ref_path[1:]):
            dx = second.x - first.x
            dy = second.y - first.y
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                continue
            projection = ((x - first.x) * dx + (y - first.y) * dy) / length_sq
            projection = max(0.0, min(1.0, projection))
            px = first.x + projection * dx
            py = first.y + projection * dy
            offset_x = x - px
            offset_y = y - py
            distance_sq = offset_x * offset_x + offset_y * offset_y
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_lateral = (-dy * offset_x + dx * offset_y) / math.sqrt(length_sq)
        return best_lateral

    def is_on_road(self, x, y, margin=0.0):
        if self.reference_lane_index < 0:
            return super().is_on_road(x, y, margin)
        lateral = self._signed_lateral_to_reference(x, y)
        right_boundary = -(self.reference_lane_index + 0.5) * self.lane_width
        left_boundary = (self.num_lanes - self.reference_lane_index - 0.5) * self.lane_width
        return right_boundary - margin <= lateral <= left_boundary + margin

    def _build_ref_path(self, points):
        rounded = _round_polyline_corners(
            points,
            max_curvature_1pm=self.road_def.max_reference_curvature_1pm,
            angle_threshold_rad=self.road_def.junction_angle_threshold_rad,
            spacing_m=1.0,
        )
        headings, curvatures = cal_heading_kappa(rounded)
        return [
            PathPoint(
                float(point[0]),
                float(point[1]),
                float(headings[index]),
                float(curvatures[index]),
            )
            for index, point in enumerate(rounded)
        ]

    def _generate_road(self):
        points = []
        if self.road_def.segments:
            for segment in self.road_def.segments:
                generated = self._generate_segment(segment)
                if points and generated:
                    gap = math.hypot(
                        generated[0][0] - points[-1][0],
                        generated[0][1] - points[-1][1],
                    )
                    if gap < 0.01:
                        generated = generated[1:]
                    elif gap > 0.5:
                        raise ValueError("Road segments are disconnected")
                points.extend(generated)
        else:
            points = self._generate_straight(200, 0)

        self._raw_waypoints = points
        if len(points) >= 2:
            self._ref_path = self._build_ref_path(points)
            self._s_map = [0.0]
            for first, second in zip(self._ref_path[:-1], self._ref_path[1:]):
                self._s_map.append(
                    self._s_map[-1]
                    + math.hypot(second.x - first.x, second.y - first.y)
                )
