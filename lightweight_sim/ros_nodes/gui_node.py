"""ROS 2 adapter that connects simulator topics and services to the GUI view."""

import rclpy
from lightweight_sim_msgs.msg import ControlCommand, ObstacleArray, Path as RosPath
from lightweight_sim_msgs.msg import SimulationStatus, VehicleState as RosVehicleState
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool, Trigger

from ..simulator.data_types import Obstacle, PathPoint
from ..visualization.ros_gui import GuiAction, GuiControl, GuiSnapshot, GuiStatus, RosGuiView
from .planner_node import message_to_state, path_to_tuples
from .qos import command_qos, latched_path_qos, sensor_data_qos, status_qos


class GuiNode(Node):
    """Keep ROS transport separate from rendering and input handling."""

    def __init__(self) -> None:
        super().__init__("simulator_gui")
        self.declare_parameter("screen_width", 1200)
        self.declare_parameter("screen_height", 800)
        self.declare_parameter("lane_width", 3.5)
        self.declare_parameter("num_lanes", 2)
        self.declare_parameter("target_speed_kmh", 40.0)

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
