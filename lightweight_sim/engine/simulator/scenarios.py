"""Scenario factories, including the three-lane figure-eight road."""

import math

from ._scenarios_impl import *  # noqa: F401,F403


def figure_eight_three_lane() -> ScenarioConfig:
    """Build a closed figure-eight centerline with three 3.5 m lanes.

    The centerline is sampled as x=a*cos(t), y=b*sin(2*t).  It produces
    two smooth loops crossing at the origin, with the vehicle starting at
    the right-hand loop and travelling upward through the crossing.
    """

    half_length = 78.0
    half_height = 42.0
    sample_count = 241
    points = [
        (
            half_length * math.cos(2.0 * math.pi * i / (sample_count - 1)),
            half_height * math.sin(4.0 * math.pi * i / (sample_count - 1)),
        )
        for i in range(sample_count)
    ]
    road = RoadDef(
        segments=[RoadSegment("waypoints", {"points": points})],
        lane_width=3.5,
        num_lanes=3,
    )
    return ScenarioConfig(
        name="figure_eight_three_lane",
        description="Closed figure-eight road with three lanes",
        road=road,
        ego_start_x=half_length,
        ego_start_y=0.0,
        ego_start_phi=math.pi / 2.0,
        ego_start_speed=6.0,
        target_speed=30.0,
        controller="LQR_controller",
        destination=None,
    )


# SCENARIOS is the same dictionary imported from the original implementation;
# adding to it also makes the original make_scenario() see this new factory.
SCENARIOS["figure_eight"] = figure_eight_three_lane
