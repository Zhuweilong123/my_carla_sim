"""ROS 2 GUI node with keyboard-driven runtime scenario selection."""

import importlib
import sys
import json
import math
from typing import Optional

import rclpy
from std_msgs.msg import String
from lightweight_sim_msgs.msg import ControlCommand, ControlMode, Obstacle as RosObstacle
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from lightweight_sim_msgs.srv import EditScene
from std_srvs.srv import SetBool

_visualization = importlib.import_module("lightweight_sim.visualization")
sys.modules.setdefault("lightweight_sim.engine.visualization", _visualization)
for _name in ("colors", "hud", "renderer", "ros_gui"):
    _module = importlib.import_module(f"lightweight_sim.visualization.{_name}")
    sys.modules.setdefault(f"lightweight_sim.engine.visualization.{_name}", _module)

from ._gui_node_impl import *  # noqa: F401,F403,E402
from .route_session import parse_context, decode_sequence
from .qos import command_qos
from ..simulator.data_types import Obstacle


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
        self._pending_route = None
        self._pending_reference_line = None
        self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())
        self.create_subscription(String, "tracking/metrics", self._on_tracking, sensor_data_qos())
        self.scenario_client = self.create_client(
            SetParameters, "simulator_node/set_parameters"
        )
        self.scene_edit_client = self.create_client(EditScene, "sim/edit_scene")
        self._edit_pause_pending = False
        self.mode_pub = self.create_publisher(ControlMode, "control_mode", latched_path_qos())
        self.manual_pub = self.create_publisher(
            ControlCommand, "control_command/manual", command_qos()
        )
        self.create_subscription(
            ControlMode, "control_mode/status", self._on_mode_status, latched_path_qos()
        )
        self.declare_parameter("mode_publish_period", 1.0)
        self.mode_timer = self.create_timer(
            float(self.get_parameter("mode_publish_period").value),
            self._publish_requested_mode,
        )
        self._publish_requested_mode()

    def _on_context(self, message):
        context = parse_context(message)
        if self.route_context and context["run_id"] <= self.route_context["run_id"]:
            return
        self.route_context = context
        self.view.reset_display_state()
        self.snapshot.scene_editing = False
        self.snapshot.scene_edit_message = ""
        self.view.target_speed_kmh = context["target_speed_kmh"]
        self.view.lane_width = context["lane_width"]
        self.view.num_lanes = context["num_lanes"]
        self.snapshot.road_network = [
            [(float(point[0]), float(point[1])) for point in road]
            for road in context.get("road_network", [])
        ]
        self.snapshot.road_network_num_lanes = int(
            context.get("road_network_num_lanes", 0)
        )
        self.snapshot.parking_slots = list(context.get("parking_slots", []))
        self.snapshot.selected_parking_slot_id = context.get(
            "selected_parking_slot_id"
        )
        pose = context.get("ego_start_pose")
        self.snapshot.draft_ego_pose = tuple(float(value) for value in pose) if pose else None
        self.snapshot.draft_obstacles = [
            Obstacle(
                id=int(item["id"]),
                x=float(item["x"]),
                y=float(item["y"]),
                length=float(item.get("length", 4.5)),
                width=float(item.get("width", 2.0)),
                speed=float(item.get("speed", 0.0)),
                heading=float(item.get("heading", 0.0)),
                type=str(item.get("type", "static")),
            )
            for item in context.get("scene_obstacles", [])
        ]
        self.snapshot.selected_obstacle_id = None
        if self.snapshot.reference_line_request_id != context["run_id"]:
            self.snapshot.reference_line_path = []
            self.snapshot.reference_line_request_id = 0
            self.snapshot.reference_lane_index = int(
                context.get("reference_lane_index", -1)
            )
            self.snapshot.routing_left_boundary = []
            self.snapshot.routing_right_boundary = []
            self.snapshot.drivable_left_boundary = []
            self.snapshot.drivable_right_boundary = []
        if self.snapshot.routing_request_id != context["run_id"]:
            self.snapshot.routing_path = []
            self.snapshot.routing_request_id = 0
        self.snapshot.state = None
        self.snapshot.obstacles = []
        # Routing overlays are tied to the previous run ID; clear them until
        # matching route and reference-line messages arrive.
        self.snapshot.planned_path = []
        self.snapshot.tracking_metrics = None
        self.snapshot._tracking_monitor = None
        self.view.hud.ed_history.clear()
        self.view.hud.ephi_history.clear()
        for attribute, callback in (
            ("_pending_route", self._on_routing),
            ("_pending_reference_line", self._on_routing_reference),
        ):
            pending = getattr(self, attribute)
            if pending is None:
                continue
            pending_run = int(pending.request_id)
            if pending_run < context["run_id"]:
                setattr(self, attribute, None)
            elif pending_run == context["run_id"]:
                setattr(self, attribute, None)
                callback(pending)

    def _on_routing(self, message):
        if self.route_context:
            message_run = int(message.request_id)
            current_run = int(self.route_context["run_id"])
            if message_run > current_run:
                self._pending_route = message
                return
            if message_run < current_run:
                return
        super()._on_routing(message)

    def _on_routing_reference(self, message):
        if self.route_context:
            message_run = int(message.request_id)
            current_run = int(self.route_context["run_id"])
            if message_run > current_run:
                self._pending_reference_line = message
                return
            if message_run < current_run:
                return
        super()._on_routing_reference(message)

    def _on_tracking(self, message):
        measured = json.loads(message.data)
        if self.route_context and measured["run_id"] == self.route_context["run_id"]:
            self.snapshot.tracking_metrics = measured

    def _on_planned(self, message):
        if self.route_context and decode_sequence(message.sequence)[0] == self.route_context["run_id"]:
            super()._on_planned(message)

    def _handle_action(self, action: GuiAction) -> None:
        if self.snapshot.scene_editing and action.kind in {"toggle_pause", "step", "reset"}:
            self.get_logger().warning("apply or cancel scene edits before continuing")
            return
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
            self._switch_scenario(str(action.value))
            return
        if action.kind == "select_parking_slot":
            self._select_parking_slot(int(action.value))
            return
        if action.kind == "toggle_scene_edit":
            self._toggle_scene_edit()
            return
        if action.kind == "scene_edit_tool":
            self.snapshot.scene_edit_tool = str(action.value)
            self.snapshot.selected_obstacle_id = None
            return
        if action.kind == "scene_map_click":
            self._scene_map_click(action.value)
            return
        if action.kind == "scene_edit_rotate":
            self._scene_edit_rotate(int(action.value))
            return
        if action.kind == "scene_edit_resize":
            self._scene_edit_resize(int(action.value))
            return
        if action.kind == "apply_scene_edit":
            self._apply_scene_edit()
            return
        if action.kind == "cancel_scene_edit":
            self.snapshot.scene_editing = False
            self.snapshot.selected_obstacle_id = None
            self.snapshot.scene_edit_message = ""
            return
        super()._handle_action(action)

    def _set_mode(self, mode: str) -> None:
        if mode not in {"CRUISE", "PARKING", "EMERGENCY_STOP"}:
            self.get_logger().warning(f"unsupported control mode: {mode}")
            return
        self._requested_mode = mode
        self.snapshot.mode = mode
        if mode == "PARKING" and self.snapshot.status.scenario != "reverse_parking":
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

    def _switch_scenario(self, name: str) -> None:
        """Keep scene shortcuts and controller mode consistent."""
        if name == "reverse_parking":
            self._requested_mode = "PARKING"
            self.snapshot.mode = "PARKING"
            self._publish_requested_mode()
        elif self._requested_mode == "PARKING":
            self._requested_mode = "CRUISE"
            self.snapshot.mode = "CRUISE"
            self._publish_requested_mode()
        self._call_scenario(name)

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
        self._call_parameter("scenario", name)

    def _select_parking_slot(self, slot_id: int) -> None:
        if not self.route_context or self.route_context.get("maneuver") != "reverse_parking":
            return
        if (
            self.snapshot.status.running
            and not self.snapshot.status.paused
            and self.snapshot.status.step_count > 0
        ):
            self.get_logger().warning("pause the simulation before changing the parking slot")
            return
        slots = self.route_context.get("parking_slots", [])
        slot = next(
            (item for item in slots if int(item.get("id", -1)) == slot_id),
            None,
        )
        if slot is None or slot.get("occupied", False):
            self.get_logger().warning(f"parking slot {slot_id} is unavailable")
            return
        self._call_parameter("parking_slot_id", slot_id)

    def _toggle_scene_edit(self) -> None:
        if self.snapshot.scene_editing:
            self.snapshot.scene_editing = False
            self.snapshot.selected_obstacle_id = None
            self.snapshot.scene_edit_message = ""
            return
        if not self.route_context or self.snapshot.draft_ego_pose is None:
            self.get_logger().warning("scene context is not ready for editing")
            return
        if self._edit_pause_pending:
            return
        if self.snapshot.status.paused:
            self._enter_scene_edit()
            return
        if not self.pause_client.service_is_ready():
            self.get_logger().warning("sim/pause service is not available")
            return
        self._edit_pause_pending = True
        run_id = self.route_context["run_id"]
        self.snapshot.scene_edit_message = "Pausing simulation for scene editing..."
        request = SetBool.Request()
        request.data = True
        future = self.pause_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._on_pause_for_scene_edit(completed, run_id)
        )

    def _on_pause_for_scene_edit(self, future, run_id: int) -> None:
        self._edit_pause_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"could not pause for scene editing: {exc}")
            return
        if not response.success:
            self.get_logger().warning(
                f"could not pause for scene editing: {response.message}"
            )
            return
        if not self.route_context or self.route_context["run_id"] != run_id:
            return
        self._enter_scene_edit()

    def _enter_scene_edit(self) -> None:
        self.snapshot.scene_editing = True
        self.snapshot.scene_edit_tool = "ego"
        self.snapshot.selected_obstacle_id = None
        self.snapshot.draft_obstacle_length = 4.5
        self.snapshot.draft_obstacle_width = 2.0
        self.snapshot.draft_obstacle_heading = 0.0
        self.snapshot.scene_edit_message = (
            "EGO: click to set start pose | ADD: click to place | "
            "MOVE: select then click | DEL: click obstacle"
        )

    def _scene_map_click(self, screen_position) -> None:
        if not self.snapshot.scene_editing or self.snapshot.draft_ego_pose is None:
            return
        x, y = self.view.camera.screen_to_world(*screen_position)
        tool = self.snapshot.scene_edit_tool
        if tool == "ego":
            self.snapshot.draft_ego_pose = (
                float(x), float(y), self.snapshot.draft_ego_pose[2]
            )
            return
        if tool == "add":
            next_id = max((item.id for item in self.snapshot.draft_obstacles), default=0) + 1
            self.snapshot.draft_obstacles.append(
                Obstacle(
                    id=next_id,
                    x=float(x),
                    y=float(y),
                    length=self.snapshot.draft_obstacle_length,
                    width=self.snapshot.draft_obstacle_width,
                    speed=0.0,
                    heading=self.snapshot.draft_obstacle_heading,
                    type="static",
                )
            )
            return
        if tool == "move" and self.snapshot.selected_obstacle_id is not None:
            selected = next(
                (item for item in self.snapshot.draft_obstacles
                 if item.id == self.snapshot.selected_obstacle_id),
                None,
            )
            if selected is not None:
                selected.x = float(x)
                selected.y = float(y)
            self.snapshot.selected_obstacle_id = None
            self.snapshot.scene_edit_message = "Obstacle moved"
            return
        obstacle = self._nearest_draft_obstacle(x, y)
        if obstacle is None:
            self.snapshot.scene_edit_message = "No obstacle near that point"
            return
        if tool == "delete":
            self.snapshot.draft_obstacles.remove(obstacle)
            self.snapshot.selected_obstacle_id = None
        elif tool == "move":
            self.snapshot.selected_obstacle_id = obstacle.id
            self.snapshot.scene_edit_message = "Click a new location for the selected obstacle"

    def _nearest_draft_obstacle(self, x: float, y: float):
        candidates = sorted(
            self.snapshot.draft_obstacles,
            key=lambda item: math.hypot(item.x - x, item.y - y),
        )
        if not candidates:
            return None
        obstacle = candidates[0]
        reach = max(2.0, 0.5 * math.hypot(obstacle.length, obstacle.width) + 0.5)
        return obstacle if math.hypot(obstacle.x - x, obstacle.y - y) <= reach else None

    def _scene_edit_rotate(self, direction: int) -> None:
        if not self.snapshot.scene_editing:
            return
        delta = math.radians(5.0) * direction
        if self.snapshot.scene_edit_tool == "ego" and self.snapshot.draft_ego_pose:
            x, y, yaw = self.snapshot.draft_ego_pose
            self.snapshot.draft_ego_pose = (x, y, yaw + delta)
            self.snapshot.scene_edit_message = "Vehicle heading adjusted by 5 degrees"
            return
        if self.snapshot.scene_edit_tool == "add":
            self.snapshot.draft_obstacle_heading += delta
            self.snapshot.scene_edit_message = "New obstacle heading adjusted by 5 degrees"
            return
        obstacle = next(
            (item for item in self.snapshot.draft_obstacles
             if item.id == self.snapshot.selected_obstacle_id),
            None,
        )
        if obstacle is not None:
            obstacle.heading += delta
            self.snapshot.scene_edit_message = "Selected obstacle heading adjusted by 5 degrees"

    def _scene_edit_resize(self, direction: int) -> None:
        if not self.snapshot.scene_editing:
            return
        selected = next(
            (item for item in self.snapshot.draft_obstacles
             if item.id == self.snapshot.selected_obstacle_id),
            None,
        )
        if self.snapshot.scene_edit_tool == "add":
            self.snapshot.draft_obstacle_length = max(
                0.5, min(50.0, self.snapshot.draft_obstacle_length + 0.5 * direction)
            )
            self.snapshot.draft_obstacle_width = max(
                0.5, min(50.0, self.snapshot.draft_obstacle_width + 0.25 * direction)
            )
        elif selected is not None:
            selected.length = max(0.5, min(50.0, selected.length + 0.5 * direction))
            selected.width = max(0.5, min(50.0, selected.width + 0.25 * direction))
        if self.snapshot.scene_edit_tool == "add" or selected is not None:
            self.snapshot.scene_edit_message = (
                "Obstacle dimensions: "
                f"{self.snapshot.draft_obstacle_length:.1f} x "
                f"{self.snapshot.draft_obstacle_width:.1f} m"
                if self.snapshot.scene_edit_tool == "add"
                else f"Selected dimensions: {selected.length:.1f} x {selected.width:.1f} m"
            )

    def _apply_scene_edit(self) -> None:
        if not self.snapshot.scene_editing or self.snapshot.draft_ego_pose is None:
            return
        if not self.scene_edit_client.service_is_ready():
            self.snapshot.scene_edit_message = "Scene edit service is unavailable"
            return
        request = EditScene.Request()
        request.ego_x, request.ego_y, request.ego_yaw = self.snapshot.draft_ego_pose
        request.obstacles = [
            RosObstacle(
                id=item.id,
                x=item.x,
                y=item.y,
                length=item.length,
                width=item.width,
                speed=item.speed,
                heading=item.heading,
                type=item.type,
            )
            for item in self.snapshot.draft_obstacles
        ]
        self.snapshot.scene_edit_message = "Applying scene..."
        future = self.scene_edit_client.call_async(request)
        future.add_done_callback(self._on_scene_edit_response)

    def _on_scene_edit_response(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                self.snapshot.scene_edit_message = response.message
                self.get_logger().warning(response.message)
                return
            self.snapshot.scene_editing = False
            self.snapshot.selected_obstacle_id = None
            self.snapshot.scene_edit_message = response.message
        except Exception as exc:
            self.snapshot.scene_edit_message = f"Scene edit failed: {exc}"
            self.get_logger().error(self.snapshot.scene_edit_message)

    def _call_parameter(self, name: str, value) -> None:
        if not self.scenario_client.service_is_ready():
            self.get_logger().warning(
                "simulator parameter service is not available; cannot update selection"
            )
            return

        parameter_value = ParameterValue()
        if isinstance(value, str):
            parameter_value.type = ParameterType.PARAMETER_STRING
            parameter_value.string_value = value
        else:
            parameter_value.type = ParameterType.PARAMETER_INTEGER
            parameter_value.integer_value = int(value)
        parameter = Parameter()
        parameter.name = name
        parameter.value = parameter_value
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
