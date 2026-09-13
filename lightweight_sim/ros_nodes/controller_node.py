"""ROS 2 adapter for the lateral/longitudinal vehicle controller."""

from typing import Optional

import rclpy
from lightweight_sim_msgs.msg import ControlCommand, Path as RosPath
from lightweight_sim_msgs.msg import VehicleState as RosVehicleState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ..algorithms.controller.combined import VehicleController
from .planner_node import message_to_state, path_to_tuples


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_node")
        self.declare_parameter("controller", "LQR_controller")
        self.declare_parameter("target_speed_kmh", 40.0)
        self.declare_parameter("control_period", 0.05)
        self.declare_parameter("state_timeout", 0.25)
        self.controller = VehicleController(
            (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0),
            controller_type=str(self.get_parameter("controller").value),
            target_speed_kmh=float(self.get_parameter("target_speed_kmh").value),
        )
        self.state: Optional[object] = None
        self.state_time = None
        self.reference_path = []
        self.planned_path = []
        self.last_sequence = -1

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_sub = self.create_subscription(
            RosVehicleState, "/vehicle/state", self._on_state, 10
        )
        self.reference_sub = self.create_subscription(
            RosPath, "/reference_path", self._on_reference, latched_qos
        )
        self.planned_sub = self.create_subscription(
            RosPath, "/planned_path", self._on_planned, 1
        )
        self.command_pub = self.create_publisher(ControlCommand, "/control_command", 10)
        period = float(self.get_parameter("control_period").value)
        self.timer = self.create_timer(period, self._on_timer)

    def _on_state(self, message: RosVehicleState) -> None:
        self.state = message_to_state(message)
        self.state_time = self.get_clock().now()

    def _on_reference(self, message: RosPath) -> None:
        path = path_to_tuples(message)
        if path:
            self.reference_path = path

    def _on_planned(self, message: RosPath) -> None:
        if int(message.sequence) < self.last_sequence:
            return
        self.last_sequence = int(message.sequence)
        self.planned_path = path_to_tuples(message)

    def _publish_command(self, steer: float, throttle: float, brake: float) -> None:
        message = ControlCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.steering_angle = float(steer)
        message.throttle = float(throttle)
        message.brake = float(brake)
        message.gear = 1
        self.command_pub.publish(message)

    def _on_timer(self) -> None:
        if self.state is None or self.state_time is None:
            self._publish_command(0.0, 0.0, 1.0)
            return
        age = (self.get_clock().now() - self.state_time).nanoseconds / 1e9
        timeout = float(self.get_parameter("state_timeout").value)
        path = self.planned_path or self.reference_path
        if age > timeout or not path:
            self._publish_command(0.0, 0.0, 1.0)
            return
        self.controller.update_ref_path(path)
        self.controller.set_target_speed(
            float(self.get_parameter("target_speed_kmh").value)
        )
        steer, throttle, brake = self.controller.step(
            self.state.x,
            self.state.y,
            self.state.phi,
            self.state.vx,
            self.state.vy,
            self.state.r,
        )
        self._publish_command(steer, throttle, brake)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
