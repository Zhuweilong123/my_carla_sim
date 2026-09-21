"""ROS 2 GUI node with keyboard-driven runtime scenario selection."""

import importlib
import sys
import json
from typing import Optional

import rclpy
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

_visualization = importlib.import_module("lightweight_sim.visualization")
sys.modules.setdefault("lightweight_sim.engine.visualization", _visualization)
for _name in ("colors", "hud", "renderer", "ros_gui"):
    _module = importlib.import_module(f"lightweight_sim.visualization.{_name}")
    sys.modules.setdefault(f"lightweight_sim.engine.visualization.{_name}", _module)

from ._gui_node_impl import *  # noqa: F401,F403,E402
from .route_session import parse_context, decode_sequence


_LegacyGuiNode = GuiNode


class GuiNode(_LegacyGuiNode):
    """Forward GUI scene-selection actions to the simulator parameter API."""

    def __init__(self) -> None:
        super().__init__()
        self.route_context = None
        self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())
        self.create_subscription(String, "tracking/metrics", self._on_tracking, sensor_data_qos())
        self.scenario_client = self.create_client(
            SetParameters, "simulator_node/set_parameters"
        )

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
        if action.kind == "switch_scenario":
            self._call_scenario(str(action.value))
            return
        super()._handle_action(action)

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
