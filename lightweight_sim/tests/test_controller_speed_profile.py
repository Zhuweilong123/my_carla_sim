import math

import pytest

from lightweight_sim.engine.ros_nodes.controller_node import (
    curvature_limited_speed_kmh,
    maximum_path_curvature,
)


def test_local_plan_curvature_limits_target_speed():
    path = [
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.05),
        (2.0, 0.0, 0.0, 0.15),
        (3.0, 0.0, 0.0, 0.01),
    ]

    curvature = maximum_path_curvature(path, x=0.0, y=0.0, lookahead_m=2.0)
    target = curvature_limited_speed_kmh(42.5, curvature, 2.0)

    assert curvature == pytest.approx(0.15)
    assert target == pytest.approx(math.sqrt(2.0 / 0.15) * 3.6)
    assert target < 42.5
