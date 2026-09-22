"""ROS 2 adapter that turns routing topology into continuous reference lines."""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from lightweight_sim_msgs.msg import PathPoint
from lightweight_sim_msgs.msg import ReferenceLine as RosReferenceLine
from lightweight_sim_msgs.msg import RoutePlan as RosRoutePlan
from lightweight_sim_msgs.msg import RouteSegment as RosRouteSegment
from rclpy.node import Node

from ..ros_nodes.qos import latched_path_qos
from ..routing.map_loader import load_map
from ..routing.models import RoutePlan, RouteSegment
from .core import ReferenceLineCore


class ReferenceLineNode(Node):
    """Consume RoutePlan events and publish map-validated reference lines."""

    def __init__(self) -> None:
        super().__init__("reference_line_node")
        default_map_dir = str(
            Path(get_package_share_directory("lightweight_sim")) / "config" / "maps"
        )
        self.declare_parameter("map_file", "")
        self.declare_parameter("map_dir", default_map_dir)
        self.declare_parameter("route_topic", "routing/route")
        self.declare_parameter("reference_topic", "routing/reference_line")
        self.declare_parameter("sample_spacing_m", 1.0)
        self.declare_parameter("max_lateral_deviation_m", 0.15)
        self.declare_parameter("join_tolerance_m", 0.25)
        self.declare_parameter("boundary_margin_m", 0.1)
        self.declare_parameter("max_reference_curvature_1pm", 0.15)
        self.declare_parameter("junction_angle_threshold_rad", 0.7)
        self._next_reference_id = 1
        self.core = ReferenceLineCore(
            self._load_maps(),
            sample_spacing_m=float(self.get_parameter("sample_spacing_m").value),
            max_lateral_deviation_m=float(
                self.get_parameter("max_lateral_deviation_m").value
            ),
            join_tolerance_m=float(self.get_parameter("join_tolerance_m").value),
            boundary_margin_m=float(self.get_parameter("boundary_margin_m").value),
            max_curvature_1pm=float(
                self.get_parameter("max_reference_curvature_1pm").value
            ),
            corner_angle_threshold_rad=float(
                self.get_parameter("junction_angle_threshold_rad").value
            ),
        )
        self.reference_pub = self.create_publisher(
            RosReferenceLine,
            str(self.get_parameter("reference_topic").value),
            latched_path_qos(),
        )
        self.route_sub = self.create_subscription(
            RosRoutePlan,
            str(self.get_parameter("route_topic").value),
            self._on_route,
            latched_path_qos(),
        )
        self.get_logger().info(
            "reference line node ready; maps=%s route_topic=%s"
            % (
                ",".join(self.core.map_ids()),
                str(self.get_parameter("route_topic").value),
            )
        )

    def _load_maps(self):
        map_file = str(self.get_parameter("map_file").value).strip()
        if map_file:
            paths = [self._resolve_path(map_file)]
        else:
            map_dir = Path(self._resolve_path(str(self.get_parameter("map_dir").value)))
            paths = sorted(str(path) for path in map_dir.glob("*.json"))
        if not paths:
            raise RuntimeError("reference line node found no JSON maps")
        return [load_map(path) for path in paths]

    @staticmethod
    def _resolve_path(value: str) -> str:
        path = Path(os.path.expandvars(value)).expanduser()
        if path.is_absolute():
            return str(path)
        return str(Path(get_package_share_directory("lightweight_sim")) / path)

    def _on_route(self, message: RosRoutePlan) -> None:
        route = RoutePlan(
            route_id=int(message.route_id),
            request_id=int(message.request_id),
            map_id=str(message.map_id),
            success=bool(message.success),
            failure_reason=str(message.failure_reason),
            total_length_m=float(message.total_length_m),
            target_lane=int(message.target_lane),
            segments=tuple(
                RouteSegment(
                    edge_id=segment.edge_id,
                    road_id=segment.road_id,
                    lane_id=segment.lane_id,
                    lane_index=segment.lane_index,
                    maneuver=segment.maneuver,
                    length_m=segment.length_m,
                    speed_limit_kmh=segment.speed_limit_kmh,
                )
                for segment in message.segments
            ),
        )
        reference = self.core.build(route, reference_id=self._next_reference_id)
        self._next_reference_id += 1
        output = self._to_ros_reference(reference)
        self.reference_pub.publish(output)
        self.get_logger().info(
            "reference request=%d reference=%d success=%s points=%d length=%.2f reason=%s"
            % (
                reference.request_id,
                reference.reference_id,
                reference.success,
                len(reference.points),
                reference.total_length_m,
                reference.failure_reason,
            )
        )

    def _to_ros_reference(self, reference) -> RosReferenceLine:
        message = RosReferenceLine()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.reference_id = reference.reference_id
        message.route_id = reference.route_id
        message.request_id = reference.request_id
        message.success = reference.success
        message.failure_reason = reference.failure_reason
        message.map_id = reference.map_id
        message.total_length_m = reference.total_length_m
        message.sample_spacing_m = reference.sample_spacing_m
        message.lane_width = reference.lane_width
        message.num_lanes = reference.num_lanes
        message.reference_lane_index = reference.reference_lane_index
        message.target_lane = reference.target_lane
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
            for segment in reference.segments
        ]
        message.points = [
            PathPoint(x=x, y=y, theta=theta, kappa=kappa)
            for x, y, theta, kappa in reference.points
        ]
        message.left_boundary = [
            PathPoint(x=x, y=y, theta=theta, kappa=kappa)
            for x, y, theta, kappa in reference.left_boundary
        ]
        message.right_boundary = [
            PathPoint(x=x, y=y, theta=theta, kappa=kappa)
            for x, y, theta, kappa in reference.right_boundary
        ]
        message.drivable_left_boundary = [
            PathPoint(x=x, y=y, theta=theta, kappa=kappa)
            for x, y, theta, kappa in reference.drivable_left_boundary
        ]
        message.drivable_right_boundary = [
            PathPoint(x=x, y=y, theta=theta, kappa=kappa)
            for x, y, theta, kappa in reference.drivable_right_boundary
        ]
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReferenceLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
