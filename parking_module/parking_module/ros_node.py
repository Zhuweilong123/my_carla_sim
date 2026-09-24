"""Optional ROS 2 adapter for the standalone parking planner/controller.

The core planner and controller do not import ROS or simulator code.  This
file is the only integration boundary and can be omitted in non-ROS usage.
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .qos import command_qos, latched_path_qos, sensor_data_qos

from lightweight_sim_msgs.msg import ControlCommand as RosControlCommand
from lightweight_sim_msgs.msg import ObstacleArray, VehicleState as RosVehicleState

from .control import ParkingController
from .core.types import BoxObstacle, ParkingSlot, Pose2D, VehicleState
from .planning import ParkingPlanningError, ReverseParkingPlanner


class ParkingControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_controller_node")
        self._state: Optional[VehicleState] = None
        self._obstacles = []
        self._slot: Optional[ParkingSlot] = None
        self._trajectory = None
        self._planner = ReverseParkingPlanner()
        self._controller = ParkingController()
        self._last_plan_signature = None
        self.declare_parameter("output_topic", "control_command")
        self._state_sub = self.create_subscription(RosVehicleState, "vehicle/state", self._on_state, sensor_data_qos())
        self._obstacle_sub = self.create_subscription(ObstacleArray, "obstacles", self._on_obstacles, sensor_data_qos())
        self._context_sub = self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())
        self._command_pub = self.create_publisher(
            RosControlCommand,
            str(self.get_parameter("output_topic").value),
            command_qos(),
        )
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info("standalone parking controller ready")

    def _on_state(self, message: RosVehicleState) -> None:
        self._state = VehicleState(message.x, message.y, message.yaw, message.vx, message.vy, message.steering_angle)

    def _on_obstacles(self, message: ObstacleArray) -> None:
        self._obstacles = [BoxObstacle(item.x, item.y, item.length, item.width, item.heading) for item in message.obstacles]

    def _on_context(self, message: String) -> None:
        try:
            context = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if context.get("maneuver") != "reverse_parking":
            self._slot = None
            self._trajectory = None
            self._last_plan_signature = None
            return
        goal = context.get("parking_goal")
        if not isinstance(goal, (list, tuple)) or len(goal) != 3:
            return
        self._slot = ParkingSlot(float(goal[0]), float(goal[1]), float(goal[2]))

    def _tick(self) -> None:
        if self._state is None or self._slot is None:
            return
        signature = (self._slot, tuple(self._obstacles))
        if signature != self._last_plan_signature:
            try:
                self._trajectory = self._planner.plan(Pose2D(self._state.x, self._state.y, self._state.yaw), self._slot, self._obstacles)
                self._controller.reset()
                self._last_plan_signature = signature
                self.get_logger().info(f"parking trajectory planned: points={len(self._trajectory.points)}")
            except ParkingPlanningError as error:
                self._trajectory = None
                self._last_plan_signature = signature
                self.get_logger().error(f"parking planning failed: {error}")
        if self._trajectory is None:
            self._publish_brake()
            return
        command = self._controller.command(self._state, self._trajectory)
        message = RosControlCommand()
        message.steering_angle = float(command.steering)
        message.throttle = float(command.throttle)
        message.brake = float(command.brake)
        message.gear = int(command.gear)
        self._command_pub.publish(message)

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
