"""ROS 2 node exposing the independent lane-level A* routing core."""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from lightweight_sim_msgs.msg import PathPoint, RoutePlan as RosRoutePlan
from lightweight_sim_msgs.msg import RouteRequest as RosRouteRequest
from lightweight_sim_msgs.msg import RouteSegment as RosRouteSegment
from lightweight_sim_msgs.srv import ComputeRoute
from rclpy.node import Node

from .core import RoutingCore
from .models import Pose2D, RouteRequest
from ..ros_nodes.qos import latched_path_qos


class RoutingNode(Node):
    """Load one or more maps and serve deterministic route requests."""

    def __init__(self) -> None:
        super().__init__("routing_node")
        default_map_dir = str(
            Path(get_package_share_directory("lightweight_sim"))
            / "config"
            / "maps"
        )
        self.declare_parameter("map_file", "")
        self.declare_parameter("map_dir", default_map_dir)
        self.declare_parameter("request_topic", "routing/request")
        self.declare_parameter("route_topic", "routing/route")
        self.declare_parameter("service_name", "routing/compute_route")
        self.declare_parameter("turn_penalty_s", 2.0)
        self.declare_parameter("lane_change_penalty_s", 1.0)
        self.declare_parameter("u_turn_penalty_s", 30.0)
        self.declare_parameter("sample_spacing_m", 1.0)
        self.route_pub = self.create_publisher(
            RosRoutePlan,
            str(self.get_parameter("route_topic").value),
            latched_path_qos(),
        )
        self.request_sub = self.create_subscription(
            RosRouteRequest,
            str(self.get_parameter("request_topic").value),
            self._on_route_request,
            latched_path_qos(),
        )
        self.service = self.create_service(
            ComputeRoute,
            str(self.get_parameter("service_name").value),
            self._on_compute_route,
        )
        map_file = str(self.get_parameter("map_file").value).strip()
        if map_file:
            map_paths = [self._resolve_path(map_file)]
        else:
            map_dir = Path(self._resolve_path(str(self.get_parameter("map_dir").value)))
            map_paths = sorted(str(path) for path in map_dir.glob("*.json"))
        if not map_paths:
            raise RuntimeError("routing node found no JSON maps")
        self.core = RoutingCore.from_paths(
            map_paths,
            turn_penalty_s=float(self.get_parameter("turn_penalty_s").value),
            lane_change_penalty_s=float(
                self.get_parameter("lane_change_penalty_s").value
            ),
            u_turn_penalty_s=float(
                self.get_parameter("u_turn_penalty_s").value
            ),
            sample_spacing_m=float(
                self.get_parameter("sample_spacing_m").value
            ),
        )
        self.get_logger().info(
            "routing node ready; maps=%s service=%s"
            % (
                ",".join(self.core.map_ids()),
                str(self.get_parameter("service_name").value),
            )
        )

    @staticmethod
    def _resolve_path(value: str) -> str:
        path = Path(os.path.expandvars(value)).expanduser()
        if path.is_absolute():
            return str(path)
        package_root = Path(get_package_share_directory("lightweight_sim"))
        return str(package_root / path)

    def _on_compute_route(self, request, response):
        plan = self.core.route(
            self._to_core_request(request.request),
            request_id=int(request.request.request_id) or None,
        )
        response.plan = self._publish_plan(plan)
        return response

    def _on_route_request(self, request: RosRouteRequest) -> None:
        plan = self.core.route(
            self._to_core_request(request),
            request_id=int(request.request_id) or None,
        )
        self._publish_plan(plan)

    @staticmethod
    def _to_core_request(request) -> RouteRequest:
        return RouteRequest(
            map_id=str(request.map_id),
            start=Pose2D(
                float(request.start_x),
                float(request.start_y),
                float(request.start_yaw),
            ),
            goal=Pose2D(
                float(request.goal_x),
                float(request.goal_y),
                float(request.goal_yaw),
            ),
            start_lane=int(request.start_lane),
            goal_lane=int(request.goal_lane),
            route_policy=str(request.route_policy or "fastest"),
            allow_u_turn=bool(request.allow_u_turn),
        )

    def _publish_plan(self, plan):
        message = self._to_ros_plan(plan)
        self.route_pub.publish(message)
        self.get_logger().info(
            "route request=%d route=%d success=%s segments=%d length=%.2f reason=%s"
            % (
                plan.request_id,
                plan.route_id,
                plan.success,
                len(plan.segments),
                plan.total_length_m,
                plan.failure_reason,
            )
        )
        return message

    def _to_ros_plan(self, plan) -> RosRoutePlan:
        message = RosRoutePlan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.route_id = plan.route_id
        message.request_id = plan.request_id
        message.success = plan.success
        message.failure_reason = plan.failure_reason
        message.map_id = plan.map_id
        message.total_length_m = plan.total_length_m
        message.target_lane = plan.target_lane
        message.segments = [
            RosRouteSegment(
                edge_id=segment.edge_id,
                road_id=segment.road_id,
                lane_id=segment.lane_id,
                lane_index=segment.lane_index,
                maneuver=segment.maneuver,
                length_m=segment.length_m,
                speed_limit_kmh=segment.speed_limit_kmh,
            )
            for segment in plan.segments
        ]
        message.points = [
            PathPoint(x=x, y=y, theta=theta, kappa=kappa)
            for x, y, theta, kappa in plan.points
        ]
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoutingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
