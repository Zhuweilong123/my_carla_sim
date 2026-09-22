"""ROS 2 GUI node with keyboard-driven runtime scenario selection."""

import importlib
import sys
import json
from typing import Optional

import rclpy
from std_msgs.msg import String
from lightweight_sim_msgs.msg import ControlCommand, ControlMode
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

_visualization = importlib.import_module("lightweight_sim.visualization")
sys.modules.setdefault("lightweight_sim.engine.visualization", _visualization)
for _name in ("colors", "hud", "renderer", "ros_gui"):
    _module = importlib.import_module(f"lightweight_sim.visualization.{_name}")
    sys.modules.setdefault(f"lightweight_sim.engine.visualization.{_name}", _module)

from ._gui_node_impl import *  # noqa: F401,F403,E402
from .route_session import parse_context, decode_sequence
from .qos import command_qos


_LegacyGuiNode = GuiNode


class GuiNode(_LegacyGuiNode):
    """Forward GUI scene-selection actions to the simulator parameter API."""

    def __init__(self) -> None:
        super().__init__()
        self._requested_mode = str(self.get_parameter("initial_mode").value).upper()
        self._control_source = str(
            self.get_parameter("initial_control_source").value
        ).upper()
        self.snapshot.mode = self._requested_mode
        self.snapshot.control_source = self._control_source
        self.route_context = None
        self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())
        self.create_subscription(String, "tracking/metrics", self._on_tracking, sensor_data_qos())
        self.scenario_client = self.create_client(
            SetParameters, "simulator_node/set_parameters"
        )
        self.mode_pub = self.create_publisher(ControlMode, "control_mode", latched_path_qos())
        self.manual_pub = self.create_publisher(
            ControlCommand, "control_command/manual", command_qos()
        )
        self.create_subscription(
            ControlMode, "control_mode/status", self._on_mode_status, latched_path_qos()
        )
        self.mode_timer = self.create_timer(1.0, self._publish_requested_mode)
        self._publish_requested_mode()

    def _on_context(self, message):
        context = parse_context(message)
        if self.route_context and context["run_id"] <= self.route_context["run_id"]:
            return
        self.route_context = context
        self.view.target_speed_kmh = context["target_speed_kmh"]
        self.view.lane_width = context["lane_width"]
        self.view.num_lanes = context["num_lanes"]
        self.snapshot.planned_path = []
        self.snapshot.tracking_metrics = None
        self.snapshot._tracking_monitor = None
        self.view.hud.ed_history.clear()
        self.view.hud.ephi_history.clear()

    def _on_tracking(self, message):
        measured = json.loads(message.data)
        if self.route_context and measured["run_id"] == self.route_context["run_id"]:
            self.snapshot.tracking_metrics = measured

    def _on_planned(self, message):
        if self.route_context and decode_sequence(message.sequence)[0] == self.route_context["run_id"]:
            super()._on_planned(message)

    def _handle_action(self, action: GuiAction) -> None:
        if action.kind == "set_mode":
            self._set_mode(str(action.value).upper())
            return
        if action.kind == "toggle_control_source":
            self._set_control_source(
                "MANUAL" if self._control_source == "AUTO" else "AUTO"
            )
            return
        if action.kind == "manual_control":
            self._publish_manual_command(action.value)
            return
        if action.kind == "switch_scenario":
            self._call_scenario(str(action.value))
            return
        super()._handle_action(action)

    def _set_mode(self, mode: str) -> None:
        if mode not in {"CRUISE", "PARKING", "EMERGENCY_STOP"}:
            self.get_logger().warning(f"unsupported control mode: {mode}")
            return
        self._requested_mode = mode
        self.snapshot.mode = mode
        if mode == "PARKING":
            self._call_scenario("reverse_parking")
        elif mode == "CRUISE" and self.snapshot.status.scenario == "reverse_parking":
            self._call_scenario("default")
        self._publish_requested_mode()
        self.get_logger().info(f"requested control mode: {mode}")

    def _set_control_source(self, source: str) -> None:
        if source not in {"AUTO", "MANUAL"}:
            return
        self._control_source = source
        self.snapshot.control_source = source
        self._publish_requested_mode()
        self.get_logger().info(f"requested control source: {source}")

    def _publish_requested_mode(self) -> None:
        message = ControlMode()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mode = self._requested_mode
        message.control_source = self._control_source
        message.source = "gui"
        self.mode_pub.publish(message)

    def _on_mode_status(self, message: ControlMode) -> None:
        if message.mode:
            self.snapshot.mode = message.mode.upper()
        if message.control_source:
            self._control_source = message.control_source.upper()
            self.snapshot.control_source = self._control_source

    def _publish_manual_command(self, command) -> None:
        if not isinstance(command, dict):
            return
        message = ControlCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.steering_angle = max(
            -0.45, min(0.45, float(command.get("steering_angle", 0.0)))
        )
        message.throttle = max(
            0.0, min(0.35, float(command.get("throttle", 0.0)))
        )
        message.brake = max(0.0, min(1.0, float(command.get("brake", 0.0))))
        message.gear = int(command.get("gear", 1))
        self.manual_pub.publish(message)

    def _call_scenario(self, name: str) -> None:
        if not self.scenario_client.service_is_ready():
            self.get_logger().warning(
                "simulator parameter service is not available; cannot switch scenario"
            )
            return

        value = ParameterValue()
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = name
        parameter = Parameter()
        parameter.name = "scenario"
        parameter.value = value
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.scenario_client.call_async(request)
        future.add_done_callback(self._on_scenario_response)

    def _on_scenario_response(self, future) -> None:
        try:
            response = future.result()
            result = response.results[0]
            if not result.successful:
                self.get_logger().error(f"scenario switch rejected: {result.reason}")
        except Exception as exc:
            self.get_logger().error(f"scenario switch failed: {exc}")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GuiNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass
