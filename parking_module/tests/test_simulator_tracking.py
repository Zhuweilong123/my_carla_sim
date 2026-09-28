"""Closed-loop parking regression against the simulator's vehicle model."""

import math

import pytest

from lightweight_sim.engine.simulator.data_types import ControlCommand as SimCommand
from lightweight_sim.engine.simulator.reverse_engine import SimulationEngine
from lightweight_sim.engine.simulator.scenarios import make_scenario, select_parking_slot
from parking_module.control import ParkingController
from parking_module.core.types import BoxObstacle, ParkingConfig, ParkingSlot, Pose2D, VehicleState
from parking_module.planning import HybridAStarPlanner, ReverseParkingPlanner


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


def test_hybrid_astar_tracks_continuous_approach_and_finishes_in_slot_three():
    config = make_scenario("reverse_parking")
    select_parking_slot(config, 3)
    engine = SimulationEngine(config)
    state = engine.get_state()
    slot = ParkingSlot(*config.parking_goal)
    obstacles = tuple(
        BoxObstacle(item["x"], item["y"], item["length"], item["width"], item["heading"])
        for item in config.obstacles
    )
    parking_config = ParkingConfig(approach_speed=config.target_speed / 3.6)
    trajectory = HybridAStarPlanner(parking_config).plan(
        Pose2D(state.x, state.y, state.phi), slot, obstacles
    )
    controller = ParkingController(parking_config)
    path_s = [0.0]
    for first, second in zip(trajectory.points, trajectory.points[1:]):
        path_s.append(
            path_s[-1]
            + math.hypot(
                second.pose.x - first.pose.x,
                second.pose.y - first.pose.y,
            )
        )
    first_reverse = next(
        index for index, point in enumerate(trajectory.points) if point.gear < 0
    )

    # A clear direct shot should keep the approach compact instead of making
    # the controller depend on a long search loop near the staging pose.
    assert path_s[first_reverse] < 30.0

    max_path_error = 0.0
    max_approach_speed = 0.0
    max_progress_excess = 0.0
    last_position = None
    traveled_distance = 0.0
    previous_progress = 0.0
    entered_reverse = False
    reverse_entry_heading_error = None
    for _ in range(1600):
        state = engine.get_state()
        moved = (
            0.0
            if last_position is None
            else math.hypot(state.x - last_position[0], state.y - last_position[1])
        )
        traveled_distance += moved
        command = controller.command(
            VehicleState(state.x, state.y, state.phi, state.vx, state.vy, state.steer),
            trajectory,
        )
        progress = path_s[controller._progress_index]
        max_progress_excess = max(
            max_progress_excess, progress - previous_progress - moved
        )
        previous_progress = progress
        last_position = (state.x, state.y)
        if command.gear < 0 and not entered_reverse:
            entered_reverse = True
            reverse_entry_heading_error = abs(
                math.atan2(
                    math.sin(state.phi - slot.heading),
                    math.cos(state.phi - slot.heading),
                )
            )
        if not entered_reverse:
            max_approach_speed = max(max_approach_speed, state.speed)

        state = engine.step(
            SimCommand(
                steer=command.steering,
                throttle=command.throttle,
                brake=command.brake,
                gear=command.gear,
            ),
            dt=config.physics_dt,
        )
        max_path_error = max(
            max_path_error,
            min(
                math.hypot(point.pose.x - state.x, point.pose.y - state.y)
                for point in trajectory.points
            ),
        )
        if engine.is_done:
            break

    assert 2.6 <= max_approach_speed <= 2.9
    assert entered_reverse
    assert reverse_entry_heading_error < math.radians(10.0)
    assert max_progress_excess < 0.6
    assert path_s[controller._progress_index] - traveled_distance < 1.0
    assert max_path_error < 0.25
    assert engine.reached_destination
    assert not engine.collision_occurred
    assert not engine.offroad_occurred
    assert math.hypot(state.x - slot.center_x, state.y - slot.center_y) < 0.31
