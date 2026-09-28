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
from lightweight_sim.engine.simulator.scenarios import make_scenario


MAP_PATH = Path(__file__).parents[1] / "config" / "maps" / "demo_grid.json"
MAP_DIR = MAP_PATH.parent


def test_demo_map_routes_through_requested_left_turn():
    core = RoutingCore([load_map(MAP_PATH)])
    plan = core.route(
        RouteRequest(
            map_id="demo_grid",
            start=Pose2D(20.0, -1.75, 0.0),
            goal=Pose2D(101.75, 160.0, 1.5708),
            start_lane=1,
            goal_lane=1,
        )
    )

    assert plan.success
    assert plan.target_lane == 1
    assert any(segment.maneuver == "left" for segment in plan.segments)
    assert all(segment.lane_index == 1 for segment in plan.segments)
    assert plan.points[0][:2] == pytest.approx((12.0, -1.75))
    assert plan.points[-1][:2] == pytest.approx((101.75, 188.0))
    assert any(abs(point[3]) > 1e-6 for point in plan.points)
    assert 250.0 < plan.total_length_m < 300.0


def test_route_policy_can_select_main_branch_without_lane_filter():
    core = RoutingCore([load_map(MAP_PATH)])
    plan = core.route(
        RouteRequest(
            map_id="demo_grid",
            start=Pose2D(20.0, -5.25),
            goal=Pose2D(180.0, -5.25),
            start_lane=0,
            goal_lane=0,
        )
    )

    assert plan.success
    assert plan.target_lane == 0
    assert all(segment.lane_index == 0 for segment in plan.segments)
    assert all(segment.maneuver == "straight" for segment in plan.segments)


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
    data["edges"][0]["successors"] = [data["edges"][1]["id"]]

    with pytest.raises(ValueError, match="does not start"):
        map_from_dict(data)


def test_map_validation_rejects_collapsed_lane_boundaries():
    data = json.loads(MAP_PATH.read_text(encoding="utf-8"))
    centerline = data["edges"][0]["centerline"]
    data["edges"][0]["left_boundary"] = centerline
    data["edges"][0]["right_boundary"] = centerline

    with pytest.raises(ValueError, match="collapsed lane boundaries"):
        map_from_dict(data)


@pytest.mark.parametrize(
    ("map_name", "start", "goal", "goal_lane", "expected_edges"),
    [
        (
            "straight_cruise.json",
            Pose2D(20.0, -1.75),
            Pose2D(180.0, -1.75),
            0,
            ["cruise_l0"],
        ),
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


@pytest.mark.parametrize(
    ("scenario_name", "map_id"),
    [("obstacle", "straight_obstacle"), ("three_lane", "three_lane_double_obs")],
)
def test_obstacle_scenarios_declare_routing_mission(scenario_name, map_id):
    config = make_scenario(scenario_name)

    assert config.routing_map_id == map_id
    assert config.routing_start_lane == 0
    assert config.routing_goal_lane == 0
    assert config.destination is not None


@pytest.mark.parametrize(
    ("scenario_name", "map_id", "start_lane", "goal_lane"),
    [
        ("default", "straight_cruise", 0, 0),
        ("curve", "curve_90deg", 1, 1),
    ],
)
def test_cruise_scenarios_declare_routing_mission(
    scenario_name, map_id, start_lane, goal_lane
):
    config = make_scenario(scenario_name)

    assert config.routing_map_id == map_id
    assert config.routing_start_lane == start_lane
    assert config.routing_goal_lane == goal_lane
    assert config.destination is not None
