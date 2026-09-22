"""Select exactly one controller candidate for the simulator actuator topic."""

import json
from typing import Dict, Optional

import rclpy
from lightweight_sim_msgs.msg import ControlCommand, ControlMode, VehicleState
from rclpy.node import Node
from std_msgs.msg import String

from .qos import command_qos, latched_path_qos, sensor_data_qos, status_qos


class ControllerManager(Node):
    """Mode arbiter for cruise, parking and emergency-stop commands.

    Controllers publish candidates on private topics.  Only this node writes
    the final ``control_command`` topic, which prevents controller conflicts
    during mode transitions.
    """

    MODES = {
        "CRUISE": "control_command/cruise",
        "PARKING": "control_command/parking",
        "MANUAL": "control_command/manual",
    }

    def __init__(self) -> None:
        super().__init__("controller_manager")
        self.declare_parameter("default_mode", "CRUISE")
        self.declare_parameter("default_control_source", "AUTO")
        self.declare_parameter("switch_hold_s", 0.5)
        self.declare_parameter("command_timeout", 0.35)
        requested = self._normalize(str(self.get_parameter("default_mode").value))
        self._mode = requested or "CRUISE"
        self._control_source = self._normalize_source(
            str(self.get_parameter("default_control_source").value)
        )
        self._commands: Dict[str, ControlCommand] = {}
        self._received_at: Dict[str, object] = {}
        self._vehicle_speed = 0.0
        self._hold_until = self.get_clock().now()
        self._run_id = 0

        self._output_pub = self.create_publisher(ControlCommand, "control_command", command_qos())
        self._status_pub = self.create_publisher(ControlMode, "control_mode/status", status_qos())
        # Keep the request subscription volatile so CLI tools and external
        # mode sources with the default ROS QoS can switch modes as well.
        self.create_subscription(ControlMode, "control_mode", self._on_mode, command_qos())
        self.create_subscription(VehicleState, "vehicle/state", self._on_state, sensor_data_qos())
        self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())
        for mode, topic in self.MODES.items():
            self.create_subscription(
                ControlCommand,
                topic,
                lambda message, selected=mode: self._on_candidate(selected, message),
                command_qos(),
            )
        self._timer = self.create_timer(0.02, self._tick)
        self._publish_status("startup")
        self.get_logger().info(f"controller manager ready; mode={self._mode}")

    @staticmethod
    def _normalize(mode: str) -> Optional[str]:
        mode = mode.strip().upper()
        if mode == "EMERGENCY_STOP":
            return mode
        return mode if mode in ControllerManager.MODES else None

    @staticmethod
    def _normalize_source(source: str) -> str:
        return "MANUAL" if source.strip().upper() == "MANUAL" else "AUTO"

    def _on_mode(self, message: ControlMode) -> None:
        mode = self._normalize(message.mode)
        if mode is None:
            self.get_logger().warning(f"ignoring unsupported control mode: {message.mode}")
            self._publish_status("rejected")
            return
        if mode != self._mode:
            self._mode = mode
            self._hold_until = self.get_clock().now() + rclpy.duration.Duration(
                seconds=float(self.get_parameter("switch_hold_s").value)
            )
            self.get_logger().info(f"control mode switched to {mode} (source={message.source})")
        source = self._normalize_source(message.control_source)
        if source != self._control_source:
            self._control_source = source
            self._hold_until = self.get_clock().now() + rclpy.duration.Duration(
                seconds=float(self.get_parameter("switch_hold_s").value)
            )
            self.get_logger().info(
                f"control source switched to {source} (source={message.source})"
            )
        self._publish_status("accepted")

    def _on_state(self, message: VehicleState) -> None:
        self._vehicle_speed = float(message.vx)

    def _on_context(self, message: String) -> None:
        """Invalidate controller candidates when the simulator starts a run."""
        try:
            context = json.loads(message.data)
            run_id = int(context["run_id"])
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return
        if run_id <= self._run_id:
            return
        self._run_id = run_id
        self._commands.clear()
        self._received_at.clear()
        self._hold_until = self.get_clock().now() + rclpy.duration.Duration(
            seconds=float(self.get_parameter("switch_hold_s").value)
        )
        self._publish_status("route_reset")

    def _on_candidate(self, mode: str, message: ControlCommand) -> None:
        self._commands[mode] = message
        self._received_at[mode] = self.get_clock().now()

    def _publish_status(self, source: str) -> None:
        message = ControlMode()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mode = self._mode
        message.control_source = self._control_source
        message.source = f"controller_manager:{source}"
        self._status_pub.publish(message)

    def _brake_command(self) -> ControlCommand:
        message = ControlCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.brake = 1.0
        # Brake direction is resolved by the simulator from signed vx.  Keep
        # the arbiter's protective command in neutral so it cannot request a
        # drive direction while stopping.
        message.gear = 0
        return message

    def _tick(self) -> None:
        if self._mode == "EMERGENCY_STOP":
            self._output_pub.publish(self._brake_command())
            return
        if self.get_clock().now() < self._hold_until:
            self._output_pub.publish(self._brake_command())
            return
        selected = "MANUAL" if self._control_source == "MANUAL" else self._mode
        received_at = self._received_at.get(selected)
        command = self._commands.get(selected)
        if received_at is None or command is None:
            self._output_pub.publish(self._brake_command())
            return
        age = (self.get_clock().now() - received_at).nanoseconds / 1e9
        if age > float(self.get_parameter("command_timeout").value):
            self._output_pub.publish(self._brake_command())
            return
        self._output_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
