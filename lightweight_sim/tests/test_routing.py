import json
from pathlib import Path

import pytest

from lightweight_sim.engine.routing import (
    Pose2D,
    RouteRequest,
    RoutingCore,
    load_map,
    map_from_dict,
)


MAP_PATH = Path(__file__).parents[1] / "config" / "maps" / "demo_grid.json"
MAP_DIR = MAP_PATH.parent


def test_demo_map_routes_through_requested_branch():
    core = RoutingCore([load_map(MAP_PATH)])
    plan = core.route(
        RouteRequest(
            map_id="demo_grid",
            start=Pose2D(10.0, 0.0, 0.0),
            goal=Pose2D(190.0, 80.0, 0.0),
            goal_lane=1,
        )
    )

    assert plan.success
    assert plan.target_lane == 1
    assert [segment.edge_id for segment in plan.segments] == [
        "r0_main",
        "r1_left",
        "r2_left",
    ]
    assert plan.points[0][:2] == pytest.approx((0.0, 0.0))
    assert plan.points[-1][:2] == pytest.approx((200.0, 80.0))
    assert any(abs(point[3]) > 1e-6 for point in plan.points)
    assert plan.total_length_m == pytest.approx(280.0)


def test_route_policy_can_select_main_branch_without_lane_filter():
    core = RoutingCore([load_map(MAP_PATH)])
    plan = core.route(
        RouteRequest(
            map_id="demo_grid",
            start=Pose2D(10.0, 0.0),
            goal=Pose2D(190.0, 0.0),
        )
    )

    assert plan.success
    assert [segment.edge_id for segment in plan.segments] == ["r0_main", "r1_main"]
    assert plan.target_lane == 0


def test_unknown_map_returns_structured_failure():
    core = RoutingCore([load_map(MAP_PATH)])
    plan = core.route(
        RouteRequest(
            map_id="missing",
            start=Pose2D(0.0, 0.0),
            goal=Pose2D(1.0, 1.0),
        )
    )

    assert not plan.success
    assert "unknown map" in plan.failure_reason
    assert plan.points == ()


def test_map_validation_rejects_disconnected_successor():
    data = json.loads(MAP_PATH.read_text(encoding="utf-8"))
    data["edges"][0]["successors"] = ["r2_left"]

    with pytest.raises(ValueError, match="does not start"):
        map_from_dict(data)


@pytest.mark.parametrize(
    ("map_name", "start", "goal", "goal_lane", "expected_edges"),
    [
        (
            "straight_obstacle.json",
            Pose2D(20.0, -1.75),
            Pose2D(180.0, -1.75),
            0,
            ["straight_l0"],
        ),
        (
            "three_lane_double_obs.json",
            Pose2D(20.0, -3.5),
            Pose2D(600.0, -3.5),
            0,
            ["straight_l0"],
        ),
        (
            "curve_90deg.json",
            Pose2D(10.0, 1.75),
            Pose2D(100.0, 140.0),
            1,
            ["curve_l1_straight", "curve_l1_arc", "curve_l1_exit"],
        ),
    ],
)
def test_scenario_maps_are_routable(
    map_name, start, goal, goal_lane, expected_edges
):
    road_map = load_map(MAP_DIR / map_name)
    plan = RoutingCore([road_map]).route(
        RouteRequest(
            map_id=road_map.map_id,
            start=start,
            goal=goal,
            goal_lane=goal_lane,
        )
    )

    assert plan.success
    assert [segment.edge_id for segment in plan.segments] == expected_edges
    assert plan.points
