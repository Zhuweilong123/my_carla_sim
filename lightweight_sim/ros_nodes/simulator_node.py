"""ROS 2 node that exposes the deterministic SimulationEngine."""

import math
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, SetBool, Trigger

from ..simulator.data_types import ControlCommand
from ..simulator.engine import SimulationEngine
from ..simulator.scenarios import make_scenario
from .protocol import encode_obstacles, encode_path


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def seconds_to_time(seconds: float) -> Time:
    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    return Time(sec=whole, nanosec=int((seconds - whole) * 1e9))


class SimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("simulator_node")
        self.declare_parameter("scenario", "obstacle")
        self.declare_parameter("physics_dt", 0.05)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")

        scenario_name = str(self.get_parameter("scenario").value)
        self.physics_dt = float(self.get_parameter("physics_dt").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.publish_clock_enabled = bool(self.get_parameter("publish_clock").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.engine = SimulationEngine(make_scenario(scenario_name))
        self.command = ControlCommand()
        self.last_command_time = self.get_clock().now()
        self.paused = False

        self.state_pub = self.create_publisher(Odometry, "/vehicle/state", 10)
        self.obstacle_pub = self.create_publisher(Float64MultiArray, "/obstacles", 10)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.reference_pub = self.create_publisher(
            Float64MultiArray, "/reference_path", latched_qos
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.command_sub = self.create_subscription(
            Float64MultiArray, "/control_command", self._on_command, 10
        )
        self.reset_srv = self.create_service(Empty, "/sim/reset", self._on_reset)
        self.pause_srv = self.create_service(SetBool, "/sim/pause", self._on_pause)
        self.step_srv = self.create_service(Trigger, "/sim/step", self._on_step)
        self.timer = self.create_timer(self.physics_dt, self._on_timer)
        self._publish_reference(sequence=0)
        self.get_logger().info("simulator ready: scenario=%s dt=%.3f", scenario_name, self.physics_dt)

    def _on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) < 3:
            self.get_logger().warning("ignoring short control command")
            return
        self.command = ControlCommand(
            steer=float(message.data[0]),
            throttle=float(message.data[1]),
            brake=float(message.data[2]),
        )
        self.last_command_time = self.get_clock().now()

    def _command_is_stale(self) -> bool:
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        return age > self.command_timeout

    def _advance_once(self) -> None:
        command = ControlCommand(brake=1.0) if self._command_is_stale() else self.command
        self.engine.step(command, dt=self.physics_dt)
        self._publish_state()
        if self.engine.is_done:
            self.paused = True
            self.get_logger().warning(
                "simulation stopped: collision=%s offroad=%s reached=%s",
                self.engine.collision_occurred,
                self.engine.offroad_occurred,
                self.engine.reached_destination,
            )

    def _on_timer(self) -> None:
        if not self.paused:
            self._advance_once()

    def _on_step(self, _request, response):
        self._advance_once()
        response.success = True
        response.message = f"step={self.engine.step_count} sim_time={self.engine.sim_time:.3f}"
        return response

    def _on_reset(self, _request, response):
        self.engine.reset()
        self.command = ControlCommand()
        self.last_command_time = self.get_clock().now()
        self.paused = False
        self._publish_reference(sequence=0)
        self._publish_state()
        return response

    def _on_pause(self, request, response):
        self.paused = bool(request.data)
        response.success = True
        response.message = "paused" if self.paused else "running"
        return response

    def _publish_reference(self, sequence: int) -> None:
        message = Float64MultiArray()
        message.data = encode_path(self.engine.world.ref_path_as_tuples, sequence)
        self.reference_pub.publish(message)

    def _publish_state(self) -> None:
        stamp = seconds_to_time(self.engine.sim_time)
        state = self.engine.get_state()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.orientation = yaw_to_quaternion(state.phi)
        odom.twist.twist.linear.x = state.vx
        odom.twist.twist.linear.y = state.vy
        odom.twist.twist.angular.z = state.r
        self.state_pub.publish(odom)

        obstacles = Float64MultiArray()
        obstacles.data = encode_obstacles(self.engine.obstacles.get_all())
        self.obstacle_pub.publish(obstacles)

        if self.publish_clock_enabled:
            clock = Clock()
            clock.clock = stamp
            self.clock_pub.publish(clock)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[SimulatorNode] = None
    try:
        node = SimulatorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
