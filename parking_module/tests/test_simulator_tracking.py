"""Closed-loop parking regression against the simulator's vehicle model."""

import math

import pytest

from lightweight_sim.engine.simulator.data_types import ControlCommand as SimCommand
from lightweight_sim.engine.simulator.reverse_engine import SimulationEngine
from lightweight_sim.engine.simulator.scenarios import make_scenario
from parking_module.control import ParkingController
from parking_module.core.types import BoxObstacle, ParkingConfig, ParkingSlot, Pose2D, VehicleState
from parking_module.planning import ReverseParkingPlanner


def test_parking_approach_tracks_at_ten_kmh_and_finishes_in_slot():
    config = make_scenario("reverse_parking")
    engine = SimulationEngine(config)
    state = engine.get_state()
    slot = ParkingSlot(*config.parking_goal)
    obstacles = tuple(
        BoxObstacle(item["x"], item["y"], item["length"], item["width"], item["heading"])
        for item in config.obstacles
    )
    parking_config = ParkingConfig(approach_speed=config.target_speed / 3.6)
    trajectory = ReverseParkingPlanner(parking_config).plan(
        Pose2D(state.x, state.y, state.phi), slot, obstacles
    )
    controller = ParkingController(parking_config)
    first_reverse = next(i for i, point in enumerate(trajectory.points) if point.gear < 0)

    vehicle_max_curvature = math.tan(config.vehicle_params.max_steer) / config.vehicle_params.wheelbase
    assert max(abs(point.curvature) for point in trajectory.points[:first_reverse]) < vehicle_max_curvature
    assert config.target_speed == pytest.approx(10.2)

    max_path_error = 0.0
    max_approach_speed = 0.0
    entered_reverse = False
    reverse_entry_heading_error = None
    for _ in range(1000):
        state = engine.get_state()
        command = controller.command(
            VehicleState(state.x, state.y, state.phi, state.vx, state.vy, state.steer),
            trajectory,
        )
        if command.gear < 0 and not entered_reverse:
            entered_reverse = True
            reverse_entry_heading_error = abs(
                (state.phi - trajectory.points[first_reverse].pose.yaw + math.pi)
                % (2.0 * math.pi) - math.pi
            )
        if not entered_reverse:
            max_approach_speed = max(max_approach_speed, state.vx)
        state = engine.step(
            SimCommand(
                steer=command.steering,
                throttle=command.throttle,
                brake=command.brake,
                gear=command.gear,
            ),
            dt=config.physics_dt,
        )
        phase = trajectory.points[first_reverse:] if entered_reverse else trajectory.points[:first_reverse]
        max_path_error = max(
            max_path_error,
            min(math.hypot(point.pose.x - state.x, point.pose.y - state.y) for point in phase),
        )
        if engine.is_done:
            break

    assert 2.6 <= max_approach_speed <= 2.9
    assert entered_reverse
    assert reverse_entry_heading_error < math.radians(10.0)
    assert max_path_error < 0.25
    assert engine.reached_destination
    assert not engine.collision_occurred
    assert not engine.offroad_occurred
    assert math.hypot(state.x - slot.center_x, state.y - slot.center_y) < 0.31
