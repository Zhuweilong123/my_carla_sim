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
from ..algorithms.planner.handover import PathHandover
from ..reference_line.dp_qp_planner import create_local_planner
from ..simulator.data_types import Obstacle, VehicleState, VehicleParams
from ..runtime_config import DEFAULT_RUNTIME_CONFIG
from .qos import latched_path_qos, sensor_data_qos
from .route_session import encode_sequence, parse_context
from .message_conversions import message_to_state


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_node")
        self.declare_parameter("plan_period", DEFAULT_RUNTIME_CONFIG.plan_period)
        self.declare_parameter("result_poll_period", 0.02)
        self.declare_parameter("lane_width", DEFAULT_RUNTIME_CONFIG.lane_width)
        self.declare_parameter("num_lanes", DEFAULT_RUNTIME_CONFIG.num_lanes)
        self.declare_parameter("prediction_time", DEFAULT_RUNTIME_CONFIG.prediction_time)
        self.declare_parameter("routing_reference_topic", "routing/reference_line")
        self.declare_parameter("routing_corridor_margin_m", 1.1)
        self.declare_parameter("local_plan_points", 80)
        self.declare_parameter("local_planner_algorithm", "dp_qp")
        self.declare_parameter("dp_station_step_m", 8.0)
        self.declare_parameter("dp_lateral_step_m", 0.5)
        self.declare_parameter("qp_station_step_m", 4.0)
        self.declare_parameter("qp_time_limit_s", 0.08)
        for name, value in dict(handover_min_time_s=.15, handover_max_time_s=.4,
                                handover_margin_time_s=.05, handover_position_tolerance_m=.35,
                                handover_heading_tolerance_rad=.25, handover_curvature_tolerance_1pm=.12,
                                handover_reuse_time_s=.5).items():
            self.declare_parameter(name, value)
        self.declare_parameter(
            "local_transition_distance_m",
            DEFAULT_RUNTIME_CONFIG.local_transition_distance_m,
        )
        self.declare_parameter(
            "local_path_sampling_resolution_m",
            DEFAULT_RUNTIME_CONFIG.local_path_sampling_resolution_m,
        )
        self.declare_parameter("local_collision_margin_m", 0.25)
        self.declare_parameter("local_obstacle_longitudinal_min_m", -5.0)
        self.declare_parameter("local_obstacle_longitudinal_max_m", 65.0)
        self.declare_parameter("local_obstacle_lateral_clearance_m", 2.2)
        self.declare_parameter("local_vehicle_length_m", VehicleParams().length)
        self.declare_parameter("local_vehicle_width_m", VehicleParams().width)
        self.state: Optional[VehicleState] = None
        self.obstacles = []
        self.sequence = 0
        self.planner: Optional[MotionPlanner] = None
        self.plan_pending = False
        self.handover = self._new_handover()
        self._request_time = 0.
        self._accepted_profile = None
        self.route_context = None
        self.active_run = None
        self.active_reference_source = None
        self.sequence = 0
        self.routing_reference_path = []
        self.routing_reference_run = None
        self._pending_routing_reference = None
        self.routing_reference_lane = -1
        self.routing_target_lane = -1
        self.routing_drivable_left_boundary = []
        self.routing_drivable_right_boundary = []
        self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())

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
        self.poll_timer = self.create_timer(
            float(self.get_parameter("result_poll_period").value),
            self._poll_result,
        )

    def _on_context(self, message):
        context = parse_context(message)
        if self.route_context and context["run_id"] <= self.route_context["run_id"]:
            return
        self.route_context = context
        self._stop_planner()
        self.sequence = 0
        self.state = None
        self.obstacles = []
        if self.routing_reference_run != context["run_id"]:
            self.routing_reference_path = []
            self.routing_reference_run = None
            self.routing_reference_lane = -1
            self.routing_target_lane = -1
            self.routing_drivable_left_boundary = []
            self.routing_drivable_right_boundary = []
        pending_reference = self._pending_routing_reference
        if pending_reference is not None:
            pending_run = int(pending_reference.request_id)
            if pending_run <= context["run_id"]:
                self._pending_routing_reference = None
                if pending_run == context["run_id"]:
                    self._on_routing_reference(pending_reference)
        self._activate_reference()

    def _on_routing_reference(self, message: RosReferenceLine) -> None:
        run_id = int(message.request_id)
        # The simulator publishes the new context and route request in the
        # same callback, but DDS may deliver the derived reference before the
        # context.  Keep it until the matching context arrives instead of
        # activating it and then clearing it during context reset.
        if self.route_context is None:
            if (
                self._pending_routing_reference is None
                or run_id >= int(self._pending_routing_reference.request_id)
            ):
                self._pending_routing_reference = message
            return
        if self.route_context and run_id != self.route_context["run_id"]:
            if run_id > self.route_context["run_id"]:
                self._pending_routing_reference = message
            return
        if not message.success or len(message.points) < 2:
            self.routing_reference_path = []
            self.routing_reference_run = run_id
            self._stop_planner()
            return
        self.routing_reference_path = [
            (point.x, point.y, point.theta, point.kappa)
            for point in message.points
        ]
        self.routing_reference_run = run_id
        self.routing_reference_lane = int(message.reference_lane_index)
        self.routing_target_lane = int(message.target_lane)
        if (
            len(message.drivable_left_boundary) == len(message.points)
            and len(message.drivable_right_boundary) == len(message.points)
        ):
            self.routing_drivable_left_boundary = [
                (point.x, point.y) for point in message.drivable_left_boundary
            ]
            self.routing_drivable_right_boundary = [
                (point.x, point.y) for point in message.drivable_right_boundary
            ]
        else:
            self.routing_drivable_left_boundary = []
            self.routing_drivable_right_boundary = []
        self._activate_reference()

    def _activate_reference(self):
        if not self.route_context:
            return
        if (
            self.routing_reference_run != self.route_context["run_id"]
            or len(self.routing_reference_path) < 2
        ):
            return
        if self.active_run == self.route_context["run_id"]:
            return
        self._stop_planner()
        self.active_run = self.route_context["run_id"]
        self.active_reference_source = "routing"
        self.plan_pending = False
        self.sequence = 0
        from rclpy.parameter import Parameter
        self.set_parameters([
            Parameter("lane_width", value=float(self.route_context["lane_width"])),
            Parameter("num_lanes", value=int(self.route_context["num_lanes"]))])
        lane_width = float(self.get_parameter("lane_width").value)
        num_lanes = int(self.get_parameter("num_lanes").value)
        vehicle = self.route_context.get("vehicle_parameters")
        if vehicle is not None:
            params = VehicleParams(**vehicle)
            vehicle_length, vehicle_width = params.length, params.width
            self.set_parameters([
                Parameter("local_vehicle_length_m", value=vehicle_length),
                Parameter("local_vehicle_width_m", value=vehicle_width)])
        else:
            vehicle_length = float(self.get_parameter("local_vehicle_length_m").value)
            vehicle_width = float(self.get_parameter("local_vehicle_width_m").value)
        self.planner = create_local_planner(
                str(self.get_parameter("local_planner_algorithm").value),
                dp_station_step_m=float(self.get_parameter("dp_station_step_m").value),
                dp_lateral_step_m=float(self.get_parameter("dp_lateral_step_m").value),
                qp_station_step_m=float(self.get_parameter("qp_station_step_m").value),
                qp_time_limit_s=float(self.get_parameter("qp_time_limit_s").value),
                global_frenet_path=self.routing_reference_path,
                lane_width=lane_width,
                num_lanes=num_lanes,
                reference_lane_index=self.routing_reference_lane,
                target_lane=self.routing_target_lane,
                drivable_left_boundary=self.routing_drivable_left_boundary,
                drivable_right_boundary=self.routing_drivable_right_boundary,
                corridor_margin_m=float(
                    self.get_parameter("routing_corridor_margin_m").value
                ),
                horizon_points=int(self.get_parameter("local_plan_points").value),
                transition_distance_m=float(
                    self.get_parameter("local_transition_distance_m").value
                ),
                sampling_resolution_m=float(
                    self.get_parameter("local_path_sampling_resolution_m").value
                ),
                collision_margin_m=float(
                    self.get_parameter("local_collision_margin_m").value
                ),
                obstacle_longitudinal_min_m=float(
                    self.get_parameter("local_obstacle_longitudinal_min_m").value
                ),
                obstacle_longitudinal_max_m=float(
                    self.get_parameter("local_obstacle_longitudinal_max_m").value
                ),
                obstacle_lateral_clearance_m=float(
                    self.get_parameter("local_obstacle_lateral_clearance_m").value
                ),
                vehicle_length_m=vehicle_length,
                vehicle_width_m=vehicle_width,
            )
        self.planner.start()
        if self.state is not None:
            self._request_plan()
        self.get_logger().info(f"planner reference source=routing algorithm={type(self.planner).__name__}")

    def _new_handover(self):
        options = dict(min_time="handover_min_time_s", max_time="handover_max_time_s",
                       margin_time="handover_margin_time_s", position_tolerance="handover_position_tolerance_m",
                       heading_tolerance="handover_heading_tolerance_rad",
                       curvature_tolerance="handover_curvature_tolerance_1pm", reuse_time="handover_reuse_time_s")
        return PathHandover(**{name: float(self.get_parameter(parameter).value) for name, parameter in options.items()})

    def _stop_planner(self):
        if self.planner is not None:
            self.planner.stop()
            self.planner = None
        self.active_run = None
        self.active_reference_source = None
        self.plan_pending = False
        self.handover = self._new_handover()
        self._accepted_profile = None

    def _on_state(self, message: RosVehicleState) -> None:
        self.state = message_to_state(message)
        # Request the first plan as soon as the current route session has both
        # a reference and a state.  Waiting for the periodic timer leaves the
        # controller tracking the road centre line for one planning period.
        if (
            self.route_context
            and self.active_run == self.route_context["run_id"]
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
        if self.route_context is None or self.active_run != self.route_context["run_id"] or self.planner is None or self.state is None or self.plan_pending:
            return
        prediction_time = float(self.get_parameter("prediction_time").value)
        state = self.state
        self._request_time = self.get_clock().now().nanoseconds*1e-9
        if str(self.get_parameter("local_planner_algorithm").value) == "dp_qp":
            state = self.handover.prepare(state, self._request_time)
            if state is None:
                self.get_logger().warning(f"handover rejected before planning: {self.handover.status} errors={self.handover.tracking_errors}")
                old = self._safe_recovery_path(self._request_time)
                if old:
                    # Keep controlling the safe prefix while recovering from
                    # measured state, without a brake/resume history gap.
                    state = self.state
                    self.planner._previous_profile = None
                    self._accepted_profile = None
                else:
                    self._publish_plan([])
                    return
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
            handover_state=self.handover.request is not None,
        )

    def _safe_recovery_path(self, now):
        old = self.handover.reusable(self.state, now, recovery=True)
        state = self.state
        # Check the actual body as well as the nominal retained trajectory.
        actual = [(state.x, state.y, state.phi, 0.),
                  (state.x+.1*math.cos(state.phi), state.y+.1*math.sin(state.phi), state.phi, 0.)]
        if (old and self.planner.validate_path(old, self.obstacles)
                and self.planner.validate_path(actual, self.obstacles)):
            return old
        return []

    def _poll_result(self) -> None:
        if self.planner is None or not self.plan_pending or not self.planner.poll_result():
            return
        path = self.planner.get_result() or []
        now = self.get_clock().now().nanoseconds*1e-9
        self.handover.observe_latency(now, self._request_time)
        if path and str(self.get_parameter("local_planner_algorithm").value) == "dp_qp":
            candidate = self.handover.splice(path, self.state, now)
            if candidate is not None and not self.planner.validate_path(candidate, self.obstacles):
                self.handover.status = 'unsafe_splice'
                candidate = None
            if candidate is None:
                self.get_logger().warning(f"handover result rejected: {self.handover.status} errors={self.handover.tracking_errors}")
                self.planner._previous_profile = self._accepted_profile
                old = self.handover.reusable(self.state, now)
                if not old and self.handover.status == 'tracking_mismatch':
                    old = self._safe_recovery_path(now)
                if old and self.planner.validate_path(old, self.obstacles):
                    # Do not extend the old path's acceptance age on reuse.
                    self._publish_plan(old, accepted=False)
                    return
                path = []
            else:
                path = candidate
                self._accepted_profile = self.planner._previous_profile
        self._publish_plan(path)
        self.plan_pending = False

    def _publish_plan(self, path, *, accepted=True):
        if accepted:
            self.handover.accept(path, self.get_clock().now().nanoseconds*1e-9)
            if not path and self.planner is not None:
                self.planner._previous_profile = None
                self._accepted_profile = None
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
