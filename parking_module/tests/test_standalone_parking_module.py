import math

from parking_module.control import ParkingController
from parking_module.core.types import BoxObstacle, ParkingSlot, Pose2D, VehicleState
from parking_module.planning import ReverseParkingPlanner


def test_planner_is_standalone_and_collision_free_for_demo_slot():
    slot = ParkingSlot(46.0, 7.5, -math.pi / 2.0)
    obstacles = (
        BoxObstacle(43.75, 7.5, 6.0, 0.25, -math.pi / 2.0),
        BoxObstacle(48.25, 7.5, 6.0, 0.25, -math.pi / 2.0),
        BoxObstacle(46.0, 10.5, 4.5, 0.25, 0.0),
    )
    trajectory = ReverseParkingPlanner().plan(Pose2D(30.0, 0.0, 0.0), slot, obstacles)
    assert len(trajectory.points) > 10
    assert {point.gear for point in trajectory.points} == {1, -1}
    assert trajectory.points[-1].pose == slot.goal
    first_reverse = next(index for index, point in enumerate(trajectory.points) if point.gear < 0)
    assert abs(trajectory.points[first_reverse - 1].pose.yaw - slot.heading) < math.radians(2.0)


def test_controller_emits_reverse_command_for_reverse_segment():
    slot = ParkingSlot(46.0, 7.5, -math.pi / 2.0)
    trajectory = ReverseParkingPlanner().plan(Pose2D(30.0, 0.0, 0.0), slot)
    controller = ParkingController()
    reverse_index = next(index for index, point in enumerate(trajectory.points) if point.gear < 0 and point.speed < 0.0)
    point = trajectory.points[reverse_index]
    command = controller.command(VehicleState(point.pose.x, point.pose.y, point.pose.yaw, 0.0), trajectory)
    assert command.gear == -1
    assert math.isfinite(command.steering)
    assert -0.5 <= command.steering <= 0.5


def test_controller_does_not_jump_to_reverse_before_staging():
    slot = ParkingSlot(46.0, 7.5, -math.pi / 2.0)
    trajectory = ReverseParkingPlanner().plan(Pose2D(30.0, 0.0, 0.0), slot)
    controller = ParkingController()
    command = controller.command(VehicleState(44.0, 1.8, 0.3, 1.5), trajectory)
    assert command.gear == 1


def test_controller_brakes_at_staging_before_reverse():
    slot = ParkingSlot(46.0, 7.5, -math.pi / 2.0)
    trajectory = ReverseParkingPlanner().plan(Pose2D(30.0, 0.0, 0.0), slot)
    controller = ParkingController()
    command = controller.command(VehicleState(45.98, -8.16, -1.3, 0.6), trajectory)
    assert command.gear == 0
    assert command.brake == 1.0


def test_controller_tapers_speed_near_parking_goal():
    slot = ParkingSlot(46.0, 7.5, -math.pi / 2.0)
    trajectory = ReverseParkingPlanner().plan(Pose2D(30.0, 0.0, 0.0), slot)
    controller = ParkingController()
    command = controller.command(VehicleState(46.0, 7.2, -math.pi / 2.0, -0.5), trajectory)
    assert command.throttle == 0.0
    assert command.brake > 0.0
    assert command.gear == -1
