"""Headless scenario factories shared by ROS 2 and other front ends."""

import math

from .data_types import RoadDef, RoadSegment, ScenarioConfig
from ..runtime_config import DEFAULT_RUNTIME_CONFIG


def default_config() -> ScenarioConfig:
    lane_y = -1.75
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
        ego_start_y=lane_y,
        ego_start_phi=0.0,
        ego_start_speed=10.0,
        speed_limit_type="straight",
        controller="LQR_controller",
        destination=(190.0, lane_y),
        routing_map_id="straight_cruise",
        routing_start_lane=0,
        routing_goal_lane=0,
    )


def demo_grid_scenario() -> ScenarioConfig:
    """Follow the left branch of the built-in demo-grid routing map."""

    road = RoadDef(
        segments=[
            RoadSegment(
                "straight",
                {"length": 100.0, "heading": 0.0, "start": (0.0, 0.0)},
            ),
            RoadSegment(
                "arc",
                {
                    "radius": 20.0,
                    "angle": math.pi / 2.0,
                    "center": (100.0, 20.0),
                    "start_angle": -math.pi / 2.0,
                    "resolution": 2.0,
                },
            ),
            RoadSegment(
                "straight",
                {"length": 56.5, "heading": math.pi / 2.0, "start": (120.0, 20.0)},
            ),
            RoadSegment(
                "straight",
                {"length": 80.0, "heading": 0.0, "start": (120.0, 76.5)},
            ),
        ],
        lane_width=3.5,
        num_lanes=2,
    )
    return ScenarioConfig(
        name="demo_grid",
        description="Demo grid junction with a left-turn route",
        road=road,
        ego_start_x=10.0,
        ego_start_y=0.0,
        ego_start_phi=0.0,
        ego_start_speed=3.0,
        speed_limit_type="straight",
        controller="LQR_controller",
        destination=(200.0, 76.5),
        routing_map_id="demo_grid",
        routing_start_lane=0,
        routing_goal_lane=0,
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
        speed_limit_type="straight",
        obstacles=[
            {"id": 1, "x": 60.0, "y": lane_y, "length": 4.5, "width": 2.0}
        ],
        controller="LQR_controller",
        destination=(180.0, lane_y),
        routing_map_id="straight_obstacle",
        routing_start_lane=0,
        routing_goal_lane=0,
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
        speed_limit_type="straight",
        obstacles=[
            {"id": 1, "x": 200.0, "y": lane0, "length": 4.5, "width": 2.0},
            {"id": 2, "x": 400.0, "y": lane1, "length": 4.5, "width": 2.0},
        ],
        controller="LQR_controller",
        destination=(600.0, lane0),
        routing_map_id="three_lane_double_obs",
        routing_start_lane=0,
        routing_goal_lane=0,
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
        speed_limit_type="curve",
        controller="LQR_controller",
        destination=(98.25, 150.0),
        routing_map_id="curve_90deg",
        routing_start_lane=1,
        routing_goal_lane=1,
    )


def reverse_parking() -> ScenarioConfig:
    """Static parking-lot scene for reverse-in maneuver development.

    The global path is the driving aisle. The parking slot is bounded by two
    side walls and a rear wall; an external parking controller should publish
    ``ControlCommand(gear=-1, throttle=..., steer=...)`` to enter it.
    """

    slot_x = 46.0
    slot_y = 7.5
    # The car faces back toward the aisle after reversing into the slot.
    slot_heading = -math.pi / 2.0
    road = RoadDef(
        segments=[RoadSegment("straight", {"length": 100, "heading": 0, "start": (0, 0)})],
        lane_width=3.5,
        num_lanes=8,
    )
    obstacles = [
        # Slot side walls: 4.2 m clear width, 6 m usable depth.
        {"id": 101, "x": slot_x - 2.25, "y": slot_y, "length": 6.0, "width": 0.25, "heading": slot_heading},
        {"id": 102, "x": slot_x + 2.25, "y": slot_y, "length": 6.0, "width": 0.25, "heading": slot_heading},
        # Rear wall.
        {"id": 103, "x": slot_x, "y": slot_y + 3.0, "length": 4.5, "width": 0.25, "heading": 0.0},
    ]
    return ScenarioConfig(
        name="reverse_parking",
        description="Reverse parking into a bounded perpendicular slot",
        road=road,
        ego_start_x=30.0,
        ego_start_y=0.0,
        ego_start_phi=0.0,
        ego_start_speed=0.0,
        speed_limit_type="parking",
        obstacles=obstacles,
        controller="LQR_controller",
        destination=(slot_x, slot_y),
        vehicle_model="kinematic",
        maneuver="reverse_parking",
        parking_goal=(slot_x, slot_y, slot_heading),
    )


SCENARIOS = {
    "default": default_config,
    "demo_grid": demo_grid_scenario,
    "obstacle": straight_with_obstacle,
    "three_lane": three_lane_double_obstacle,
    "curve": curve_scenario,
    "reverse_parking": reverse_parking,
}


def make_scenario(name: str) -> ScenarioConfig:
    try:
        config = SCENARIOS[name]()
    except KeyError as exc:
        raise ValueError(f"unknown scenario: {name}; choose from {sorted(SCENARIOS)}") from exc
    speed_limits = {
        "default": DEFAULT_RUNTIME_CONFIG.default_speed_limit_kmh,
        "straight": DEFAULT_RUNTIME_CONFIG.straight_speed_limit_kmh,
        "curve": DEFAULT_RUNTIME_CONFIG.curve_speed_limit_kmh,
        "intersection": DEFAULT_RUNTIME_CONFIG.intersection_speed_limit_kmh,
        "lane_change": DEFAULT_RUNTIME_CONFIG.lane_change_speed_limit_kmh,
        "parking": DEFAULT_RUNTIME_CONFIG.parking_speed_limit_kmh,
    }
    speed_type = getattr(config, "speed_limit_type", "default")
    speed_limit = float(speed_limits.get(speed_type, speed_limits["default"]))
    config.speed_limits = speed_limits
    config.speed_limit_kmh = speed_limit
    config.target_speed_ratio = DEFAULT_RUNTIME_CONFIG.target_speed_ratio
    config.target_speed = speed_limit * config.target_speed_ratio
    return config
