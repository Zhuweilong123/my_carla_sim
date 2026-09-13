"""ROS 2 adapter for the latest-only obstacle-aware planner."""

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from ..algorithms.planner.motion_planner import MotionPlanner
from ..simulator.data_types import Obstacle, VehicleState
from .protocol import decode_obstacles, decode_path, encode_path


def odometry_to_state(message: Odometry) -> VehicleState:
    q = message.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
    return VehicleState(
        x=message.pose.pose.position.x,
        y=message.pose.pose.position.y,
        phi=yaw,
        vx=message.twist.twist.linear.x,
        vy=message.twist.twist.linear.y,
        r=message.twist.twist.angular.z,
    )


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_node")
        self.declare_parameter("plan_period", 0.5)
        self.declare_parameter("lane_width", 3.5)
        self.declare_parameter("num_lanes", 2)
        self.declare_parameter("prediction_time", 0.2)
        self.path = []
        self.state: Optional[VehicleState] = None
        self.obstacles = []
        self.sequence = 0
        self.planner: Optional[MotionPlanner] = None
        self.plan_pending = False

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_sub = self.create_subscription(
            Float64MultiArray, "/reference_path", self._on_path, latched_qos
        )
        self.state_sub = self.create_subscription(Odometry, "/vehicle/state", self._on_state, 10)
        self.obstacle_sub = self.create_subscription(
            Float64MultiArray, "/obstacles", self._on_obstacles, 10
        )
        self.result_pub = self.create_publisher(Float64MultiArray, "/planned_path", 1)
        period = float(self.get_parameter("plan_period").value)
        self.plan_timer = self.create_timer(period, self._request_plan)
        self.poll_timer = self.create_timer(0.02, self._poll_result)

    def _on_path(self, message: Float64MultiArray) -> None:
        _sequence, path = decode_path(message.data)
        if not path:
            return
        self.path = path
        if self.planner is not None:
            self.planner.stop()
        self.planner = MotionPlanner(
            path,
            lane_width=float(self.get_parameter("lane_width").value),
            num_lanes=int(self.get_parameter("num_lanes").value),
        )
        self.planner.start()

    def _on_state(self, message: Odometry) -> None:
        self.state = odometry_to_state(message)

    def _on_obstacles(self, message: Float64MultiArray) -> None:
        self.obstacles = [
            Obstacle(
                id=item[0],
                x=item[1],
                y=item[2],
                length=item[3],
                width=item[4],
                speed=item[5],
                heading=item[6],
            )
            for item in decode_obstacles(message.data)
        ]

    def _request_plan(self) -> None:
        if self.planner is None or self.state is None or self.plan_pending:
            return
        prediction_time = float(self.get_parameter("prediction_time").value)
        state = self.state
        pred_loc = (
            state.x + state.vx * prediction_time * math.cos(state.phi)
            - state.vy * prediction_time * math.sin(state.phi),
            state.y + state.vy * prediction_time * math.cos(state.phi)
            + state.vx * prediction_time * math.sin(state.phi),
        )
        self.plan_pending = self.planner.plan(
            ego_state=state,
            obstacles=self.obstacles,
            pred_loc=pred_loc,
            vehicle_loc=(state.x, state.y),
        )

    def _poll_result(self) -> None:
        if self.planner is None or not self.plan_pending or not self.planner.poll_result():
            return
        path = self.planner.get_result() or []
        self.sequence += 1
        message = Float64MultiArray()
        message.data = encode_path(path, self.sequence)
        self.result_pub.publish(message)
        self.plan_pending = False

    def destroy_node(self):
        if self.planner is not None:
            self.planner.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
