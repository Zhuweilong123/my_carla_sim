import math
from pathlib import Path

import pytest

from lightweight_sim.engine.reference_line import ReferenceLineCore, RouteAwareMotionPlanner
from lightweight_sim.engine.routing import (
    Pose2D,
    RoutePlan,
    RouteRequest,
    RouteSegment,
    RoutingCore,
    load_map,
)


MAP_DIR = Path(__file__).parents[1] / "config" / "maps"


def test_reference_line_stitches_and_smooths_curve_route():
    road_map = load_map(MAP_DIR / "curve_90deg.json")
    route = RoutingCore([road_map]).route(
        RouteRequest(
            map_id=road_map.map_id,
            start=Pose2D(10.0, 1.75),
            goal=Pose2D(100.0, 140.0),
            goal_lane=1,
        )
    )

    reference = ReferenceLineCore([road_map]).build(route, reference_id=11)

    assert reference.success
    assert reference.reference_id == 11
    assert reference.reference_lane_index == 1
    assert reference.lane_width == pytest.approx(3.5)
    assert reference.num_lanes == 2
    assert [segment.edge_id for segment in reference.segments] == [
        "curve_l1_straight",
        "curve_l1_arc",
        "curve_l1_exit",
    ]
    assert len(reference.points) > 200
    assert reference.points[0][:2] == pytest.approx((0.0, 1.75))
    assert reference.points[-1][:2] == pytest.approx((98.25, 150.0))
    assert all(math.isfinite(value) for point in reference.points for value in point)
    assert any(abs(point[3]) > 1e-4 for point in reference.points)
    assert len(reference.left_boundary) == len(reference.points)
    assert len(reference.right_boundary) == len(reference.points)
    assert len(reference.drivable_left_boundary) == len(reference.points)
    assert len(reference.drivable_right_boundary) == len(reference.points)
    for point, left, right in zip(
        reference.points, reference.left_boundary, reference.right_boundary
    ):
        normal_x = left[0] - right[0]
        normal_y = left[1] - right[1]
        normal_length = math.hypot(normal_x, normal_y)
        assert normal_length == pytest.approx(3.5, abs=0.08)
        normal_x /= normal_length
        normal_y /= normal_length
        lateral = (point[0] - right[0]) * normal_x + (point[1] - right[1]) * normal_y
        assert 0.1 - 1e-6 <= lateral <= normal_length - 0.1 + 1e-6
    assert all(
        math.dist(left[:2], right[:2]) > 6.5
        for left, right in zip(
            reference.drivable_left_boundary, reference.drivable_right_boundary
        )
    )


def test_reference_line_rejects_non_topological_edge_sequence():
    road_map = load_map(MAP_DIR / "demo_grid.json")
    bad_route = RoutePlan(
        route_id=1,
        request_id=2,
        map_id=road_map.map_id,
        success=True,
        segments=(
            RouteSegment("r0_main", "r0", "r0_l0", 0, "straight", 100.0, 40.0),
            RouteSegment("r2_left", "r2", "r2_l1", 1, "straight", 100.0, 40.0),
        ),
    )

    reference = ReferenceLineCore([road_map]).build(bad_route, reference_id=3)

    assert not reference.success
    assert "not topologically connected" in reference.failure_reason


def test_route_aware_local_planner_returns_to_routing_lane_after_detour():
    route_path = [(float(x), -3.5, 0.0, 0.0) for x in range(0, 201)]
    planner = RouteAwareMotionPlanner(
        route_path,
        lane_width=3.5,
        num_lanes=3,
        reference_lane_index=0,
        target_lane=0,
        drivable_left_boundary=[(float(x), 5.25) for x in range(0, 201)],
        drivable_right_boundary=[(float(x), -5.25) for x in range(0, 201)],
    )

    detour = planner._plan(
        pred_loc=(20.0, -3.5),
        vehicle_loc=(20.0, -3.5),
        obstacles=[(55.0, -3.5, 4.5, 2.0, 0.0, 0.0)],
    )
    return_path = planner._plan(
        pred_loc=(80.0, 0.0),
        vehicle_loc=(80.0, 0.0),
        obstacles=[],
    )

    assert detour[-1][1] > -1.0
    assert return_path[-1][1] < -2.5
    assert all(-4.15 <= point[1] <= 4.15 for point in detour)
    assert any(abs(point[2]) > 1e-3 for point in detour)
    assert planner._trajectory_is_safe(
        detour,
        [(55.0, -3.5, 4.5, 2.0, 0.0, 0.0)],
    )
    assert not planner._trajectory_is_safe(
        [(55.0, -3.5, 0.0, 0.0)],
        [(55.0, -3.5, 4.5, 2.0, 0.0, 0.0)],
    )
