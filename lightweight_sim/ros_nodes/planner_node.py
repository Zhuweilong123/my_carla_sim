"""ROS 2 adapter for the latest-only obstacle-aware planner."""

import math
from typing import Optional

import rclpy
from lightweight_sim_msgs.msg import ObstacleArray
from lightweight_sim_msgs.msg import Path as RosPath
from lightweight_sim_msgs.msg import PathPoint
from lightweight_sim_msgs.msg import VehicleState as RosVehicleState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ..algorithms.planner.motion_planner import MotionPlanner
from ..simulator.data_types import Obstacle, VehicleState


def message_to_state(message: RosVehicleState) -> VehicleState:
    return VehicleState(
        x=message.x,
        y=message.y,
        phi=message.yaw,
        vx=message.vx,
        vy=message.vy,
        r=message.yaw_rate,
        steer=message.steering_angle,
        accel=message.acceleration,
        timestamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
    )


def path_to_tuples(message: RosPath):
    return [(p.x, p.y, p.theta, p.kappa) for p in message.points]


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
            RosPath, "/reference_path", self._on_path, latched_qos
        )
        self.state_sub = self.create_subscription(
            RosVehicleState, "/vehicle/state", self._on_state, 10
        )
        self.obstacle_sub = self.create_subscription(
            ObstacleArray, "/obstacles", self._on_obstacles, 10
        )
        self.result_pub = self.create_publisher(RosPath, "/planned_path", 1)
        period = float(self.get_parameter("plan_period").value)
        self.plan_timer = self.create_timer(period, self._request_plan)
        self.poll_timer = self.create_timer(0.02, self._poll_result)

    def _on_path(self, message: RosPath) -> None:
        path = path_to_tuples(message)
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

    def _on_state(self, message: RosVehicleState) -> None:
        self.state = message_to_state(message)

    def _on_obstacles(self, message: ObstacleArray) -> None:
        self.obstacles = [
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
        message = RosPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.sequence = self.sequence
        message.points = [
            PathPoint(x=p[0], y=p[1], theta=p[2], kappa=p[3]) for p in path
        ]
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
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass
