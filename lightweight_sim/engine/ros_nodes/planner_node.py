"""ROS 2 adapter for the latest-only obstacle-aware planner."""

import math
from typing import Optional

import rclpy
from std_msgs.msg import String
from lightweight_sim_msgs.msg import ObstacleArray
from lightweight_sim_msgs.msg import Path as RosPath
from lightweight_sim_msgs.msg import PathPoint
from lightweight_sim_msgs.msg import ReferenceLine as RosReferenceLine
from lightweight_sim_msgs.msg import VehicleState as RosVehicleState
from rclpy.node import Node
from ..algorithms.planner.motion_planner import MotionPlanner
from ..reference_line import RouteAwareMotionPlanner
from ..simulator.data_types import Obstacle, VehicleState
from ..runtime_config import DEFAULT_RUNTIME_CONFIG
from .qos import latched_path_qos, sensor_data_qos
from .route_session import decode_sequence, encode_sequence, parse_context


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
        self.declare_parameter("plan_period", DEFAULT_RUNTIME_CONFIG.plan_period)
        self.declare_parameter("lane_width", DEFAULT_RUNTIME_CONFIG.lane_width)
        self.declare_parameter("num_lanes", DEFAULT_RUNTIME_CONFIG.num_lanes)
        self.declare_parameter("prediction_time", DEFAULT_RUNTIME_CONFIG.prediction_time)
        self.declare_parameter("use_routing_reference", True)
        self.declare_parameter("routing_reference_topic", "routing/reference_line")
        self.path = []
        self.state: Optional[VehicleState] = None
        self.obstacles = []
        self.sequence = 0
        self.planner: Optional[MotionPlanner] = None
        self.plan_pending = False
        self.route_context = None
        self.reference_run = None
        self.active_run = None
        self.active_reference_source = None
        self.routing_reference_path = []
        self.routing_reference_run = None
        self.routing_reference_lane = -1
        self.routing_target_lane = -1
        self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())

        self.path_sub = self.create_subscription(
            RosPath, "reference_path", self._on_path, latched_path_qos()
        )
        self.routing_reference_sub = self.create_subscription(
            RosReferenceLine,
            str(self.get_parameter("routing_reference_topic").value),
            self._on_routing_reference,
            latched_path_qos(),
        )
        sensor_qos = sensor_data_qos()
        self.state_sub = self.create_subscription(
            RosVehicleState, "vehicle/state", self._on_state, sensor_qos
        )
        self.obstacle_sub = self.create_subscription(
            ObstacleArray, "obstacles", self._on_obstacles, sensor_qos
        )
        self.result_pub = self.create_publisher(
            RosPath, "planned_path", latched_path_qos()
        )
        period = float(self.get_parameter("plan_period").value)
        self.plan_timer = self.create_timer(period, self._request_plan)
        self.poll_timer = self.create_timer(0.02, self._poll_result)

    def _on_path(self, message: RosPath) -> None:
        run, version = decode_sequence(message.sequence)
        if version or (self.route_context and run < self.route_context["run_id"]):
            return
        path = path_to_tuples(message)
        if not path:
            return
        self.path = path
        self.reference_run = run
        self._activate_reference()

    def _on_context(self, message):
        context = parse_context(message)
        if self.route_context and context["run_id"] <= self.route_context["run_id"]:
            return
        self.route_context = context
        self.active_run = None
        self.active_reference_source = None
        self.plan_pending = False
        self.state = None
        self.obstacles = []
        if self.routing_reference_run != context["run_id"]:
            self.routing_reference_path = []
            self.routing_reference_run = None
            self.routing_reference_lane = -1
            self.routing_target_lane = -1
        if self.planner is not None:
            self.planner.stop()
            self.planner = None
        self._activate_reference()

    def _on_routing_reference(self, message: RosReferenceLine) -> None:
        if not message.success or len(message.points) < 2:
            return
        run_id = int(message.request_id)
        if self.route_context and run_id != self.route_context["run_id"]:
            return
        self.routing_reference_path = [
            (point.x, point.y, point.theta, point.kappa)
            for point in message.points
        ]
        self.routing_reference_run = run_id
        self.routing_reference_lane = int(message.reference_lane_index)
        self.routing_target_lane = int(message.target_lane)
        self._activate_reference()

    def _activate_reference(self):
        if not self.route_context or self.reference_run != self.route_context["run_id"]:
            return
        routing_ready = (
            bool(self.get_parameter("use_routing_reference").value)
            and self.routing_reference_run == self.route_context["run_id"]
            and len(self.routing_reference_path) >= 2
        )
        source = "routing" if routing_ready else "simulator"
        if (
            self.active_run == self.reference_run
            and self.active_reference_source == source
        ):
            return
        if self.planner is not None:
            self.planner.stop()
        new_run = self.active_run != self.reference_run
        self.active_run = self.reference_run
        self.active_reference_source = source
        self.plan_pending = False
        if new_run:
            self.sequence = 0
        from rclpy.parameter import Parameter
        self.set_parameters([
            Parameter("lane_width", value=float(self.route_context["lane_width"])),
            Parameter("num_lanes", value=int(self.route_context["num_lanes"]))])
        lane_width = float(self.get_parameter("lane_width").value)
        num_lanes = int(self.get_parameter("num_lanes").value)
        if routing_ready:
            self.planner = RouteAwareMotionPlanner(
                self.routing_reference_path,
                lane_width=lane_width,
                num_lanes=num_lanes,
                reference_lane_index=self.routing_reference_lane,
                target_lane=self.routing_target_lane,
            )
        else:
            self.planner = MotionPlanner(
                self.path,
                lane_width=lane_width,
                num_lanes=num_lanes,
            )
        self.planner.start()
        if self.state is not None:
            self._request_plan()
        self.get_logger().info(f"planner reference source={source}")

    def _on_state(self, message: RosVehicleState) -> None:
        self.state = message_to_state(message)
        # Request the first plan as soon as the current route session has both
        # a reference and a state.  Waiting for the periodic timer leaves the
        # controller tracking the road centre line for one planning period.
        if (
            self.active_run == self.reference_run
            and self.planner is not None
            and self.sequence == 0
            and not self.plan_pending
        ):
            self._request_plan()

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
        if self.active_run != self.reference_run or self.planner is None or self.state is None or self.plan_pending:
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
        self._publish_plan(path)
        self.plan_pending = False

    def _publish_plan(self, path):
        self.sequence += 1
        message = RosPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.sequence = encode_sequence(self.active_run, self.sequence)
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
