"""Optional ROS 2 adapter for the standalone parking planner/controller.

The core planner and controller do not import ROS or simulator code.  This
file is the only integration boundary and can be omitted in non-ROS usage.
"""

import json
import math
from dataclasses import replace
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .qos import command_qos, latched_path_qos, sensor_data_qos

from lightweight_sim_msgs.msg import ControlCommand as RosControlCommand
from lightweight_sim_msgs.msg import (
    ObstacleArray,
    Path as RosPath,
    PathPoint,
    VehicleState as RosVehicleState,
)

from .control import ParkingController
from .core.types import BoxObstacle, ParkingConfig, ParkingSlot, Pose2D, VehicleState
from .planning import ParkingPlanningError, create_planner


_PATH_SEQUENCE_VERSION_BITS = 20
_PATH_SEQUENCE_VERSION_MASK = (1 << _PATH_SEQUENCE_VERSION_BITS) - 1


def _encode_path_sequence(run_id: int, version: int) -> int:
    """Encode Path.sequence run/version IDs without importing simulator code."""
    if not 0 < run_id < (1 << 44) or not 0 < version <= _PATH_SEQUENCE_VERSION_MASK:
        raise ValueError("parking path sequence is out of range")
    return (run_id << _PATH_SEQUENCE_VERSION_BITS) | version


class ParkingControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_controller_node")
        self._state: Optional[VehicleState] = None
        self._state_stamp = None
        self._obstacles = []
        self._obstacle_stamp = None
        self._inputs_ready = False
        self._slot: Optional[ParkingSlot] = None
        self._trajectory = None
        self._run_id: Optional[int] = None
        self._path_version = 0
        self._controller = ParkingController()
        self._last_plan_signature = None
        self._last_command_gear = None
        self.declare_parameter("output_topic", "control_command")
        self.declare_parameter("planner_type", "hybrid_astar")
        self._planner_type = str(self.get_parameter("planner_type").value)
        try:
            self._planner = create_planner(self._planner_type)
        except ValueError as error:
            raise ValueError(
                f"invalid parking_controller_node planner_type={self._planner_type!r}: {error}"
            ) from error
        self._state_sub = self.create_subscription(RosVehicleState, "vehicle/state", self._on_state, sensor_data_qos())
        self._obstacle_sub = self.create_subscription(ObstacleArray, "obstacles", self._on_obstacles, sensor_data_qos())
        self._context_sub = self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())
        self._command_pub = self.create_publisher(
            RosControlCommand,
            str(self.get_parameter("output_topic").value),
            command_qos(),
        )
        # The safety supervisor and GUI consume the same current-run path
        # contract as they do for local cruise planning.
        self._path_pub = self.create_publisher(RosPath, "planned_path", latched_path_qos())
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "parking planner/controller ready; "
            f"planner_type={self._planner_type}; awaiting parking context"
        )

    def _on_state(self, message: RosVehicleState) -> None:
        self._state = VehicleState(message.x, message.y, message.yaw, message.vx, message.vy, message.steering_angle)
        self._state_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        self._update_input_readiness()

    def _on_obstacles(self, message: ObstacleArray) -> None:
        self._obstacles = [BoxObstacle(item.x, item.y, item.length, item.width, item.heading) for item in message.obstacles]
        self._obstacle_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        self._update_input_readiness()

    def _update_input_readiness(self) -> None:
        if self._state_stamp is not None and self._state_stamp == self._obstacle_stamp:
            self._inputs_ready = True

    def _on_context(self, message: String) -> None:
        try:
            context = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        run_id = context.get("run_id")
        if not isinstance(run_id, int) or isinstance(run_id, bool) or run_id <= 0:
            return
        if self._run_id is not None and run_id <= self._run_id:
            return
        self._run_id = run_id
        self._path_version = 0
        # Sensor topics are independent DDS streams. Discard the previous
        # run's cached pose/obstacles and wait for a synchronized new snapshot
        # before planning against the reset vehicle position.
        self._state = None
        self._state_stamp = None
        self._obstacles = []
        self._obstacle_stamp = None
        self._inputs_ready = False
        self._slot = None
        self._trajectory = None
        self._last_plan_signature = None
        self._last_command_gear = None
        self._controller.reset()
        if context.get("maneuver") != "reverse_parking":
            return
        goal = context.get("parking_goal")
        if not isinstance(goal, (list, tuple)) or len(goal) != 3:
            self.get_logger().error("reverse-parking context has no valid parking_goal")
            return
        try:
            values = tuple(float(value) for value in goal)
        except (TypeError, ValueError):
            self.get_logger().error("reverse-parking goal contains non-numeric values")
            return
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("reverse-parking goal contains non-finite values")
            return
        self._slot = ParkingSlot(*values)
        try:
            target_speed_kmh = float(context.get("target_speed_kmh", ParkingConfig().approach_speed * 3.6))
        except (TypeError, ValueError):
            self.get_logger().error("reverse-parking target speed is invalid")
            self._slot = None
            return
        if not math.isfinite(target_speed_kmh) or target_speed_kmh <= 0.0:
            self.get_logger().error("reverse-parking target speed must be positive")
            self._slot = None
            return
        try:
            vehicle = context.get("vehicle_parameters")
            dimensions = {}
            if vehicle is not None:
                a, b = float(vehicle["a"]), float(vehicle["b"])
                overhang = float(vehicle["body_overhang"])
                if not all(math.isfinite(v) and v > 0 for v in (a, b, overhang)):
                    raise ValueError("invalid vehicle axle distances/body overhang")
                dimensions = dict(wheelbase=a+b, vehicle_length=a+b+overhang,
                                  vehicle_width=float(vehicle["width"]),
                                  max_steer=float(vehicle["max_steer"]))
            parking_config = replace(ParkingConfig(), approach_speed=target_speed_kmh / 3.6,
                                     **dimensions)
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f"invalid parking vehicle parameters: {error}")
            self._slot = None
            return
        self._planner = create_planner(self._planner_type, parking_config)
        self._controller = ParkingController(parking_config)

    def _tick(self) -> None:
        if (
            not self._inputs_ready
            or self._state is None
            or self._slot is None
            or self._run_id is None
        ):
            return
        signature = (self._slot, tuple(self._obstacles))
        if signature != self._last_plan_signature:
            try:
                self._trajectory = self._planner.plan(Pose2D(self._state.x, self._state.y, self._state.yaw), self._slot, self._obstacles)
                self._controller.reset()
                self._last_plan_signature = signature
                self.get_logger().info(
                    "parking trajectory planned: "
                    f"points={len(self._trajectory.points)} "
                    f"start=({self._state.x:.2f},{self._state.y:.2f},{self._state.yaw:.2f})"
                )
            except ParkingPlanningError as error:
                self._trajectory = None
                self._last_plan_signature = signature
                self.get_logger().error(f"parking planning failed: {error}")
                self._publish_trajectory()
        if self._trajectory is None:
            self._publish_brake()
            return
        self._publish_trajectory()
        command = self._controller.command(self._state, self._trajectory)
        message = RosControlCommand()
        message.steering_angle = float(command.steering)
        message.throttle = float(command.throttle)
        message.brake = float(command.brake)
        message.gear = int(command.gear)
        if command.gear != self._last_command_gear:
            self.get_logger().info(
                "parking gear transition: "
                f"gear={command.gear} pose=({self._state.x:.2f},{self._state.y:.2f}) "
                f"yaw={self._state.yaw:.2f} speed={self._state.vx:.2f} "
                f"throttle={command.throttle:.2f} brake={command.brake:.2f}"
            )
            self._last_command_gear = command.gear
        self._command_pub.publish(message)

    def _publish_trajectory(self) -> None:
        """Publish the active parking path as a fresh run-scoped plan heartbeat."""
        if self._run_id is None:
            return
        self._path_version = (self._path_version % _PATH_SEQUENCE_VERSION_MASK) + 1
        message = RosPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.sequence = _encode_path_sequence(self._run_id, self._path_version)
        message.points = [
            PathPoint(
                x=float(point.pose.x),
                y=float(point.pose.y),
                theta=float(point.pose.yaw),
                kappa=float(point.curvature),
            )
            for point in (self._trajectory.points if self._trajectory is not None else ())
        ]
        self._path_pub.publish(message)

    def _publish_brake(self) -> None:
        message = RosControlCommand()
        message.brake = 1.0
        message.gear = 0
        self._command_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
