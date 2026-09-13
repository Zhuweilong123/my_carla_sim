"""Headless scenario factories shared by ROS 2 and other front ends."""

import math

from .data_types import RoadDef, RoadSegment, ScenarioConfig


def default_config() -> ScenarioConfig:
    road = RoadDef(
        segments=[RoadSegment("straight", {"length": 200, "heading": 0, "start": (0, 0)})],
        lane_width=3.5,
        num_lanes=2,
    )
    return ScenarioConfig(
        name="straight_200m",
        description="200 m straight-road cruise",
        road=road,
        ego_start_x=20.0,
        ego_start_y=0.0,
        ego_start_phi=0.0,
        ego_start_speed=10.0,
        target_speed=20.0,
        controller="LQR_controller",
        destination=(190.0, 0.0),
    )


def straight_with_obstacle() -> ScenarioConfig:
    lane_y = -1.75
    road = RoadDef(
        segments=[RoadSegment("straight", {"length": 200, "heading": 0, "start": (0, 0)})],
        lane_width=3.5,
        num_lanes=2,
    )
    return ScenarioConfig(
        name="straight_obstacle",
        description="200 m straight road with a static obstacle",
        road=road,
        ego_start_x=20.0,
        ego_start_y=lane_y,
        ego_start_phi=0.0,
        ego_start_speed=10.0,
        target_speed=40.0,
        obstacles=[
            {"id": 1, "x": 60.0, "y": lane_y, "length": 4.5, "width": 2.0}
        ],
        controller="LQR_controller",
        destination=(180.0, lane_y),
    )


def three_lane_double_obstacle() -> ScenarioConfig:
    lane0, lane1 = -3.5, 0.0
    road = RoadDef(
        segments=[RoadSegment("straight", {"length": 1000, "heading": 0, "start": (0, 0)})],
        lane_width=3.5,
        num_lanes=3,
    )
    return ScenarioConfig(
        name="three_lane_double_obs",
        description="Three lanes with two sequential obstacles",
        road=road,
        ego_start_x=20.0,
        ego_start_y=lane0,
        ego_start_phi=0.0,
        ego_start_speed=5.56,
        target_speed=40.0,
        obstacles=[
            {"id": 1, "x": 200.0, "y": lane0, "length": 4.5, "width": 2.0},
            {"id": 2, "x": 400.0, "y": lane1, "length": 4.5, "width": 2.0},
        ],
        controller="LQR_controller",
        destination=(600.0, lane0),
    )


def curve_scenario() -> ScenarioConfig:
    road = RoadDef(
        segments=[
            RoadSegment("straight", {"length": 50, "heading": 0, "start": (0, 0)}),
            RoadSegment(
                "arc",
                {
                    "radius": 50,
                    "angle": math.pi / 2,
                    "center": (50, 50),
                    "start_angle": -math.pi / 2,
                },
            ),
            RoadSegment(
                "straight",
                {"length": 100, "heading": math.pi / 2, "start": (100, 50)},
            ),
        ],
        lane_width=3.5,
        num_lanes=2,
    )
    return ScenarioConfig(
        name="curve_90deg",
        description="90 degree curve",
        road=road,
        ego_start_x=10.0,
        ego_start_y=1.75,
        ego_start_phi=0.0,
        ego_start_speed=8.0,
        target_speed=30.0,
        controller="LQR_controller",
    )


SCENARIOS = {
    "default": default_config,
    "obstacle": straight_with_obstacle,
    "three_lane": three_lane_double_obstacle,
    "curve": curve_scenario,
}


def make_scenario(name: str) -> ScenarioConfig:
    try:
        return SCENARIOS[name]()
    except KeyError as exc:
        raise ValueError(f"unknown scenario: {name}; choose from {sorted(SCENARIOS)}") from exc
