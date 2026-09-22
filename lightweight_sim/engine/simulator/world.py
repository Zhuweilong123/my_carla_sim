"""Road geometry with safe joining of consecutive segments."""

import math

from ..reference_line.core import _round_polyline_corners
from ._world_impl import *  # noqa: F401,F403


_LegacyWorld = World


class World(_LegacyWorld):
    """Keep coincident segment endpoints without flagging them as gaps."""

    def _build_ref_path(self, points):
        rounded = _round_polyline_corners(
            points,
            max_curvature_1pm=self.road_def.max_reference_curvature_1pm,
            angle_threshold_rad=self.road_def.junction_angle_threshold_rad,
            spacing_m=1.0,
        )
        return super()._build_ref_path(rounded)

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
