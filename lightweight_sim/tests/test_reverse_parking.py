import pytest

from lightweight_sim.engine.simulator.data_types import ControlCommand
from lightweight_sim.engine.simulator.reverse_engine import SimulationEngine
from lightweight_sim.engine.simulator.scenarios import make_scenario


def test_reverse_parking_scene_has_slot_geometry():
    config = make_scenario("reverse_parking")

    assert config.maneuver == "reverse_parking"
    assert config.parking_goal == pytest.approx((46.0, 7.5, 1.57079632679))
    assert len(config.obstacles) == 3
    assert config.destination == pytest.approx((46.0, 7.5))


def test_reverse_gear_produces_signed_longitudinal_motion():
    engine = SimulationEngine(make_scenario("reverse_parking"))

    state = engine.step(ControlCommand(gear=-1, throttle=1.0), dt=0.1)

    assert state.vx < 0.0
    assert state.x < 30.0
