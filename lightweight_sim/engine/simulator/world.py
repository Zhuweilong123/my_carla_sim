"""Road geometry with safe joining of consecutive segments."""

import math

from ._world_impl import *  # noqa: F401,F403


_LegacyWorld = World


class World(_LegacyWorld):
    """Keep coincident segment endpoints without flagging them as gaps."""

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
