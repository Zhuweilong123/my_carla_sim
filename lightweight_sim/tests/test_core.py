import math

import pytest

from lightweight_sim.algorithms.planner.motion_planner import MotionPlanner
from lightweight_sim.algorithms.controller.lat_lqr import LateralLQRController
from lightweight_sim.algorithms.controller.combined import VehicleController
from lightweight_sim.simulator.data_types import (
    ControlCommand,
    Obstacle,
    RoadDef,
    RoadSegment,
    ScenarioConfig,
    VehicleState,
)
from lightweight_sim.simulator.engine import SimulationEngine
from lightweight_sim.simulator.obstacle import ObstacleManager
from lightweight_sim.simulator.vehicle import EgoVehicle, VehicleParams
from lightweight_sim.simulator.world import World


def test_engine_uses_fixed_physics_and_clamps_commands():
    engine = SimulationEngine(ScenarioConfig(ego_start_speed=0.0))
    state = engine.step(
        ControlCommand(steer=99.0, throttle=2.0, brake=-1.0),
        dt=0.1,
    )

    assert engine.step_count == 1
    assert engine.sim_time == pytest.approx(0.1)
    assert state.vx == pytest.approx(0.3)
    assert abs(state.steer) <= engine.ego.params.max_steer


def test_dynamic_vehicle_model_produces_lateral_state():
    vehicle = EgoVehicle(
        VehicleState(vx=15.0),
        VehicleParams(),
    )
    state = vehicle.step(0.1, 0.0, 0.05, model="dynamic")

    assert state.vx > 0.0
    assert math.isfinite(state.vy)
    assert math.isfinite(state.r)


def test_rotated_rectangle_collision_uses_geometry():
    manager = ObstacleManager()
    manager.add_obstacle(Obstacle(id=1, x=4.0, y=0.0, length=4.0, width=2.0))

    assert manager.check_collision(0.0, 0.0, 5.0, 2.0, 0.0)
    assert not manager.check_collision(0.0, 5.0, 5.0, 2.0, 0.0)


def test_disconnected_road_is_rejected():
    road = RoadDef(
        segments=[
            RoadSegment("straight", {"length": 10.0}),
            RoadSegment("straight", {"length": 10.0, "start": (30.0, 0.0)}),
        ]
    )

    with pytest.raises(ValueError, match="disconnected"):
        World(road)


def test_latest_planner_can_choose_a_free_lane():
    road = RoadDef(num_lanes=3)
    world = World(road)
    planner = MotionPlanner(world.ref_path_as_tuples, lane_width=3.5, num_lanes=3)
    result = planner._plan(
        pred_loc=(20.0, 0.0),
        vehicle_loc=(20.0, 0.0),
        obstacles=[(55.0, 0.0, 4.5, 2.0, 0.0, 0.0)],
    )

    assert result
    assert max(abs(point[1]) for point in result) > 2.0
def test_road_boundary_uses_segment_projection():
    world = World(RoadDef(num_lanes=3))
    assert world.is_on_road(24.17, -3.5)
    assert not world.is_on_road(24.17, -6.0)


def test_lateral_controller_converges_toward_reference():
    engine = SimulationEngine(
        ScenarioConfig(
            road=RoadDef(num_lanes=3),
            ego_start_y=-3.5,
            ego_start_speed=10.0,
            target_speed=40.0,
        )
    )
    controller = LateralLQRController(
        (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0)
    )
    path = engine.world.ref_path_as_tuples
    start_y = engine.get_state().y
    for _ in range(80):
        state = engine.get_state()
        steer = controller.control(
            state.x, state.y, state.phi, state.vx, state.vy, state.r, path
        )
        engine.step(ControlCommand(steer=steer, throttle=0.0), dt=0.05)
    assert abs(engine.get_state().y) < abs(start_y)
    assert not engine.offroad_occurred
def test_replanning_preserves_safe_lane_change():
    road = RoadDef(
        segments=[
            RoadSegment(
                "straight",
                {"length": 1000.0, "heading": 0.0, "start": (0.0, 0.0)},
            )
        ],
        num_lanes=3,
    )
    engine = SimulationEngine(
        ScenarioConfig(
            road=road,
            ego_start_y=-3.5,
            ego_start_speed=5.56,
            target_speed=40.0,
            obstacles=[
                {"id": 1, "x": 200.0, "y": -3.5},
                {"id": 2, "x": 400.0, "y": 0.0},
            ],
        )
    )
    planner = MotionPlanner(engine.world.ref_path_as_tuples, 3.5, 3)
    controller = VehicleController(
        (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0),
        target_speed_kmh=40.0,
    )
    for step in range(700):
        state = engine.get_state()
        if step % 50 == 0:
            predicted = (state.x + state.vx * 0.2, state.y)
            obstacle_data = [
                (o.x, o.y, o.length, o.width, o.speed, o.heading)
                for o in engine.obstacles.get_all()
            ]
            controller.update_ref_path(
                planner._plan(predicted, (state.x, state.y), obstacle_data)
            )
        steer, throttle, brake = controller.step(
            state.x, state.y, state.phi, state.vx, state.vy, state.r
        )
        engine.step(
            ControlCommand(steer=steer, throttle=throttle, brake=brake),
            dt=0.05,
        )
        if engine.is_done:
            break

    assert not engine.collision_occurred
    assert not engine.offroad_occurred
    assert engine.get_state().y > 2.0