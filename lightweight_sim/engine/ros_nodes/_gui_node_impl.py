"""ROS 2 adapter that connects simulator topics and services to the GUI view."""

import rclpy
from std_msgs.msg import String
from lightweight_sim_msgs.msg import ControlCommand, ObstacleArray, Path as RosPath
from lightweight_sim_msgs.msg import RoutePlan as RosRoutePlan
from lightweight_sim_msgs.msg import ReferenceLine as RosReferenceLine
from lightweight_sim_msgs.msg import SimulationStatus, VehicleState as RosVehicleState
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool, Trigger

from ..simulator.data_types import Obstacle, PathPoint
from ..runtime_config import DEFAULT_RUNTIME_CONFIG
from ..visualization.ros_gui import GuiAction, GuiControl, GuiSnapshot, GuiStatus, RosGuiView
from .planner_node import message_to_state, path_to_tuples
from .qos import command_qos, latched_path_qos, sensor_data_qos, status_qos
from .route_session import parse_context


class GuiNode(Node):
    """Keep ROS transport separate from rendering and input handling."""

    def __init__(self) -> None:
        super().__init__("simulator_gui")
        self.declare_parameter("screen_width", 1200)
        self.declare_parameter("screen_height", 800)
        self.declare_parameter("lane_width", DEFAULT_RUNTIME_CONFIG.lane_width)
        self.declare_parameter("num_lanes", DEFAULT_RUNTIME_CONFIG.num_lanes)
        self.declare_parameter("target_speed_kmh", DEFAULT_RUNTIME_CONFIG.target_speed_kmh)
        self.declare_parameter("initial_mode", "CRUISE")
        self.declare_parameter("initial_control_source", "AUTO")

        self.snapshot = GuiSnapshot()
        self.view = RosGuiView(
            width=int(self.get_parameter("screen_width").value),
            height=int(self.get_parameter("screen_height").value),
            lane_width=float(self.get_parameter("lane_width").value),
            num_lanes=int(self.get_parameter("num_lanes").value),
            target_speed_kmh=float(self.get_parameter("target_speed_kmh").value),
        )

        sensor_qos = sensor_data_qos()
        self.create_subscription(
            RosVehicleState, "vehicle/state", self._on_state, sensor_qos
        )
        self.create_subscription(
            ObstacleArray, "obstacles", self._on_obstacles, sensor_qos
        )
        self.create_subscription(
            RosPath, "reference_path", self._on_reference, latched_path_qos()
        )
        self.create_subscription(
            RosPath, "planned_path", self._on_planned, latched_path_qos()
        )
        self.create_subscription(
            RosRoutePlan, "routing/route", self._on_routing, latched_path_qos()
        )
        self.create_subscription(
            RosReferenceLine,
            "routing/reference_line",
            self._on_routing_reference,
            latched_path_qos(),
        )
        self.create_subscription(
            String, "sim/context", self._on_context, latched_path_qos()
        )
        self.create_subscription(
            SimulationStatus, "sim/status", self._on_status, status_qos()
        )
        self.create_subscription(
            ControlCommand, "control_command", self._on_control, command_qos()
        )

        self.reset_client = self.create_client(Empty, "sim/reset")
        self.pause_client = self.create_client(SetBool, "sim/pause")
        self.step_client = self.create_client(Trigger, "sim/step")
        self.get_logger().info("ROS 2 GUI ready; waiting for simulator topics")

    def _on_state(self, message: RosVehicleState) -> None:
        self.snapshot.state = message_to_state(message)

    def _on_obstacles(self, message: ObstacleArray) -> None:
        self.snapshot.obstacles = [
            Obstacle(
                id=item.id,
                x=item.x,
                y=item.y,
                length=item.length,
                width=item.width,
                speed=item.speed,
                heading=item.heading,
                type=item.type,
            )
            for item in message.obstacles
        ]

    def _on_reference(self, message: RosPath) -> None:
        self.snapshot.reference_path = [
            PathPoint(x=p[0], y=p[1], theta=p[2], kappa=p[3])
            for p in path_to_tuples(message)
        ]

    def _on_planned(self, message: RosPath) -> None:
        self.snapshot.planned_path = path_to_tuples(message)

    def _on_routing(self, message: RosRoutePlan) -> None:
        self.snapshot.routing_request_id = int(message.request_id)
        if not message.success:
            self.snapshot.routing_path = []
            return
        self.snapshot.routing_path = [
            PathPoint(x=point.x, y=point.y, theta=point.theta, kappa=point.kappa)
            for point in message.points
        ]

    def _on_context(self, message: String) -> None:
        context = parse_context(message)
        if int(context["run_id"]) == self.snapshot.routing_request_id:
            return
        self.snapshot.routing_request_id = 0
        self.snapshot.routing_path = []
        self.snapshot.reference_line_path = []
        self.snapshot.routing_left_boundary = []
        self.snapshot.routing_right_boundary = []
        self.snapshot.drivable_left_boundary = []
        self.snapshot.drivable_right_boundary = []

    def _on_routing_reference(self, message: RosReferenceLine) -> None:
        if not message.success:
            self.snapshot.reference_line_path = []
            self.snapshot.routing_left_boundary = []
            self.snapshot.routing_right_boundary = []
            self.snapshot.drivable_left_boundary = []
            self.snapshot.drivable_right_boundary = []
            return
        self.snapshot.reference_line_path = [
            PathPoint(x=point.x, y=point.y, theta=point.theta, kappa=point.kappa)
            for point in message.points
        ]
        self.snapshot.routing_left_boundary = [
            PathPoint(x=point.x, y=point.y, theta=point.theta, kappa=point.kappa)
            for point in message.left_boundary
        ]
        self.snapshot.routing_right_boundary = [
            PathPoint(x=point.x, y=point.y, theta=point.theta, kappa=point.kappa)
            for point in message.right_boundary
        ]
        self.snapshot.drivable_left_boundary = [
            PathPoint(x=point.x, y=point.y, theta=point.theta, kappa=point.kappa)
            for point in message.drivable_left_boundary
        ]
        self.snapshot.drivable_right_boundary = [
            PathPoint(x=point.x, y=point.y, theta=point.theta, kappa=point.kappa)
            for point in message.drivable_right_boundary
        ]

    def _on_status(self, message: SimulationStatus) -> None:
        self.snapshot.status = GuiStatus(
            running=message.running,
            paused=message.paused,
            done=message.done,
            collision=message.collision,
            offroad=message.offroad,
            reached=message.reached,
            step_count=message.step_count,
            sim_time=message.sim_time,
            scenario=message.scenario,
            termination_reason=message.termination_reason,
        )

    def _on_control(self, message: ControlCommand) -> None:
        self.snapshot.control = GuiControl(
            steering_angle=message.steering_angle,
            throttle=message.throttle,
            brake=message.brake,
        )

    def _handle_action(self, action: GuiAction) -> None:
        if action.kind == "quit":
            self._shutdown_requested = True
        elif action.kind == "reset":
            self._call_reset()
        elif action.kind == "toggle_pause":
            self._call_pause(not self.snapshot.status.paused)
        elif action.kind == "step":
            self._call_step()

    def _call_reset(self) -> None:
        if not self.reset_client.service_is_ready():
            self.get_logger().warning("sim/reset service is not available")
            return
        self.reset_client.call_async(Empty.Request())

    def _call_pause(self, paused: bool) -> None:
        if not self.pause_client.service_is_ready():
            self.get_logger().warning("sim/pause service is not available")
            return
        request = SetBool.Request()
        request.data = paused
        self.pause_client.call_async(request)

    def _call_step(self) -> None:
        if not self.step_client.service_is_ready():
            self.get_logger().warning("sim/step service is not available")
            return
        self.step_client.call_async(Trigger.Request())

    def run(self) -> None:
        self._shutdown_requested = False
        try:
            while rclpy.ok() and not self._shutdown_requested:
                rclpy.spin_once(self, timeout_sec=0.0)
                for action in self.view.poll_actions():
                    self._handle_action(action)
                self.view.render(self.snapshot)
        finally:
            self.destroy_node()
            self.view.close()


def main(args=None) -> None:
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
