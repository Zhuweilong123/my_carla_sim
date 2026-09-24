"""ROS 2 adapter for the lateral/longitudinal vehicle controller."""

from typing import Optional
import json
import math

import rclpy
from std_msgs.msg import String
from lightweight_sim_msgs.msg import ControlCommand, Path as RosPath
from lightweight_sim_msgs.msg import ReferenceLine as RosReferenceLine
from lightweight_sim_msgs.msg import VehicleState as RosVehicleState
from rclpy.node import Node
from ..algorithms.controller.combined import VehicleController
from .planner_node import message_to_state, path_to_tuples
from .qos import command_qos, latched_path_qos, sensor_data_qos
from .route_session import decode_sequence, parse_context
from ..analysis.tracking import TrackingMonitor
from ..simulator.data_types import VehicleParams
from ..simulator.steering import SteeringParams
from ..runtime_config import DEFAULT_RUNTIME_CONFIG


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_node")
        self.declare_parameter("controller", DEFAULT_RUNTIME_CONFIG.controller)
        self.declare_parameter("physics_dt", DEFAULT_RUNTIME_CONFIG.physics_dt)
        self.declare_parameter("target_speed_kmh", DEFAULT_RUNTIME_CONFIG.target_speed_kmh)
        self.declare_parameter("control_period", DEFAULT_RUNTIME_CONFIG.control_period)
        self.declare_parameter("state_timeout", DEFAULT_RUNTIME_CONFIG.state_timeout)
        self.declare_parameter("plan_timeout", DEFAULT_RUNTIME_CONFIG.plan_timeout)
        self.declare_parameter("routing_reference_topic", "routing/reference_line")
        self.declare_parameter(
            "speed_profile_lookahead_m", DEFAULT_RUNTIME_CONFIG.speed_profile_lookahead_m
        )
        self.declare_parameter("output_topic", "control_command")
        self.declare_parameter("actuator_compensation", True)
        self.declare_parameter("lateral_q_weights", [200.0, 1.0, 50.0, 1.0])
        self.declare_parameter("lateral_r", 100.0)
        self.declare_parameter("dynamic_lateral_r", 300.0)
        self.declare_parameter("lateral_feedback_horizon_s", 0.0)
        self.declare_parameter("lateral_smooth_reference_heading", True)
        self.declare_parameter("lateral_discretization", "plant")
        self.declare_parameter("longitudinal_kp", 1.15)
        self.declare_parameter("longitudinal_ki", 0.0)
        self.declare_parameter("longitudinal_kd", 0.55)
        self.declare_parameter("longitudinal_error_threshold_kmh", 1.0)
        self.declare_parameter("longitudinal_max_jerk_mps3", 40.0)
        self.declare_parameter("longitudinal_coupling_gain", 0.5)
        self.controller = VehicleController(
            vehicle_params=VehicleParams(),
            steering_params=SteeringParams(),
            controller_type=str(self.get_parameter("controller").value),
            target_speed_kmh=float(self.get_parameter("target_speed_kmh").value),
            dt=float(self.get_parameter("physics_dt").value),
            **self._controller_options(),
        )
        self.state: Optional[object] = None
        self.state_time = None
        self.reference_path = []
        self.planned_path = []
        self.last_sequence = -1
        self.route_context = None
        self.reference_run = None
        self.active_run = None
        self.target_speed_ratio = 1.0
        self.max_lateral_accel_mps2 = DEFAULT_RUNTIME_CONFIG.max_lateral_accel_mps2
        self.speed_profile_lookahead_m = float(
            self.get_parameter("speed_profile_lookahead_m").value
        )
        self.routing_speed_points = []
        self.routing_speed_s = []
        self.routing_speed_profile = []
        self.plan_time = None
        self.plan_ready = False
        self.last_control_stamp = None
        self.actuator_timing_fault = False
        self.tracking_monitor = None
        self.measurement_path = None
        self.tracking_pub = self.create_publisher(String, "tracking/metrics", sensor_data_qos())
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
        self.reference_sub = self.create_subscription(
            RosPath, "reference_path", self._on_reference, latched_path_qos()
        )
        self.planned_sub = self.create_subscription(
            RosPath, "planned_path", self._on_planned, latched_path_qos()
        )
        self.command_pub = self.create_publisher(
            ControlCommand,
            str(self.get_parameter("output_topic").value),
            command_qos(),
        )
        period = float(self.get_parameter("control_period").value)
        self.timer = self.create_timer(period, self._on_timer)

    def _controller_options(self):
        return dict(
            actuator_compensation=bool(
                self.get_parameter("actuator_compensation").value
            ),
            lateral_q=list(self.get_parameter("lateral_q_weights").value),
            lateral_r=float(self.get_parameter("lateral_r").value),
            dynamic_lateral_r=float(
                self.get_parameter("dynamic_lateral_r").value
            ),
            feedback_horizon_s=float(
                self.get_parameter("lateral_feedback_horizon_s").value
            ),
            smooth_reference_heading=bool(
                self.get_parameter("lateral_smooth_reference_heading").value
            ),
            discretization=str(
                self.get_parameter("lateral_discretization").value
            ),
            longitudinal_params=dict(
                K_P=float(self.get_parameter("longitudinal_kp").value),
                K_I=float(self.get_parameter("longitudinal_ki").value),
                K_D=float(self.get_parameter("longitudinal_kd").value),
                error_threshold=float(
                    self.get_parameter("longitudinal_error_threshold_kmh").value
                ),
                max_jerk=float(
                    self.get_parameter("longitudinal_max_jerk_mps3").value
                ),
                coupling_gain=float(
                    self.get_parameter("longitudinal_coupling_gain").value
                ),
            ),
        )

    def _on_state(self, message: RosVehicleState) -> None:
        self.state = message_to_state(message)
        self.state_time = self.get_clock().now()
        # One control calculation per state; a backward /clock jump on reset
        # must not stall control until an old timer deadline is reached.
        self._on_timer()

    def _on_context(self, message):
        context = parse_context(message)
        if self.route_context and context["run_id"] <= self.route_context["run_id"]:
            return
        self.route_context = context
        self.target_speed_ratio = float(context.get("target_speed_ratio", 1.0))
        self.max_lateral_accel_mps2 = float(
            context.get(
                "max_lateral_accel_mps2",
                DEFAULT_RUNTIME_CONFIG.max_lateral_accel_mps2,
            )
        )
        self.actuator_timing_fault = False
        self.active_run = None
        self.routing_speed_points = []
        self.routing_speed_s = []
        self.routing_speed_profile = []
        self.planned_path = []
        self.plan_time = None
        self.plan_ready = False
        self.state = None
        self.last_control_stamp = None
        self.last_sequence = -1
        self._activate_reference()

    def _on_routing_reference(self, message: RosReferenceLine) -> None:
        """Cache route segment limits for ratio-based longitudinal control."""

        if not message.success or len(message.points) < 2:
            return
        run_id = int(message.request_id)
        if self.route_context and run_id != self.route_context["run_id"]:
            return
        points = [
            (float(point.x), float(point.y), float(point.kappa))
            for point in message.points
        ]
        cumulative = [0.0]
        for first, second in zip(points[:-1], points[1:]):
            cumulative.append(
                cumulative[-1]
                + math.hypot(second[0] - first[0], second[1] - first[1])
            )
        ratio = self.target_speed_ratio
        configured_limits = (
            self.route_context.get("speed_limits", {}) if self.route_context else {}
        )
        profile = []
        end_s = 0.0
        for segment in message.segments:
            end_s += max(0.0, float(segment.length_m))
            maneuver = str(segment.maneuver).lower()
            if maneuver in {"left", "right", "u_turn", "curve"}:
                speed_type = "curve"
            elif maneuver in {"lane_change", "merge"}:
                speed_type = "lane_change"
            elif maneuver in {"intersection", "junction"}:
                speed_type = "intersection"
            else:
                speed_type = "straight"
            speed_limit = configured_limits.get(
                speed_type, float(segment.speed_limit_kmh)
            )
            profile.append((end_s, max(0.0, float(speed_limit)) * ratio))
        if not profile:
            return
        self.routing_speed_points = points
        self.routing_speed_s = cumulative
        self.routing_speed_profile = profile

    def _target_speed_at(self, x: float, y: float) -> float:
        """Return the current route-limit target, or the scene fallback."""

        if not self.routing_speed_points or not self.routing_speed_profile:
            return float(
                self.route_context.get("target_speed_kmh", self.get_parameter("target_speed_kmh").value)
                if self.route_context
                else self.get_parameter("target_speed_kmh").value
            )
        nearest_index = min(
            range(len(self.routing_speed_points)),
            key=lambda index: (
                self.routing_speed_points[index][0] - x
            ) ** 2 + (self.routing_speed_points[index][1] - y) ** 2,
        )
        current_s = self.routing_speed_s[nearest_index]
        route_s = current_s + self.speed_profile_lookahead_m
        target_speed = self.routing_speed_profile[-1][1]
        for end_s, target_speed in self.routing_speed_profile:
            if route_s <= end_s:
                break

        end_index = nearest_index
        while (
            end_index + 1 < len(self.routing_speed_s)
            and self.routing_speed_s[end_index + 1] <= route_s
        ):
            end_index += 1
        max_abs_curvature = max(
            abs(self.routing_speed_points[index][2])
            for index in range(nearest_index, end_index + 1)
        )
        if max_abs_curvature > 1e-6:
            curve_speed_mps = math.sqrt(
                self.max_lateral_accel_mps2 / max_abs_curvature
            )
            target_speed = min(target_speed, curve_speed_mps * 3.6)
        return target_speed

    def _activate_reference(self):
        if not self.route_context or self.reference_run != self.route_context["run_id"]:
            return
        if self.active_run == self.reference_run:
            return
        self.active_run = self.reference_run
        self.controller = VehicleController(
            controller_type=str(self.get_parameter("controller").value),
            target_speed_kmh=self.route_context["target_speed_kmh"],
            vehicle_params=VehicleParams(**self.route_context.get("vehicle_parameters", {})),
            steering_params=SteeringParams(**self.route_context.get("steering_parameters", {})),
            dt=float(self.route_context["physics_dt"]),
            **self._controller_options(),
        )
        self.controller.update_ref_path(self.reference_path, reset=True)
        self.tracking_monitor = TrackingMonitor(self.reference_path)
        self.measurement_path = self.controller.ref_path
        self.controller.set_target_speed(self.route_context["target_speed_kmh"])
        self.controller.lat.ts = float(self.route_context["physics_dt"])
        self.controller.lon.dt = float(self.route_context["physics_dt"])
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter("target_speed_kmh", value=float(self.route_context["target_speed_kmh"]))])

    def _on_reference(self, message: RosPath) -> None:
        run, version = decode_sequence(message.sequence)
        if version or (self.route_context and run < self.route_context["run_id"]):
            return
        path = path_to_tuples(message)
        if path:
            self.reference_path = path
            self.reference_run = run
            if run != self.active_run:
                self.active_run = None
                self.planned_path = []
                self.plan_time = None
                self.plan_ready = False
            self._activate_reference()

    def _on_planned(self, message: RosPath) -> None:
        run, version = decode_sequence(message.sequence)
        if run != self.active_run or not version or int(message.sequence) <= self.last_sequence:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9 - stamp
        if age < -0.1 or age > float(self.get_parameter("plan_timeout").value):
            return
        self.last_sequence = int(message.sequence)
        self.planned_path = path_to_tuples(message)
        self.plan_time = stamp
        self.plan_ready = bool(self.planned_path)
        self.controller.update_ref_path(self.planned_path or self.reference_path, reset=False)

    def _publish_command(self, steer: float, throttle: float, brake: float) -> None:
        message = ControlCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.steering_angle = float(steer)
        message.throttle = float(throttle)
        message.brake = float(brake)
        message.gear = 1
        self.command_pub.publish(message)

    def _on_timer(self) -> None:
        if self.active_run is None or self.state is None or self.state_time is None:
            self._publish_command(0.0, 0.0, 1.0)
            return
        if self.route_context and self.route_context.get("maneuver") == "reverse_parking":
            # Reverse parking has a separate controller candidate.  The cruise
            # controller must stay neutral even if it was launched directly.
            self._publish_command(0.0, 0.0, 1.0)
            return
        age = (self.get_clock().now() - self.state_time).nanoseconds / 1e9
        timeout = float(self.get_parameter("state_timeout").value)
        if age < 0 or age > timeout or not self.reference_path:
            self._publish_command(0.0, 0.0, 1.0)
            return
        if not self.plan_ready:
            # Do not steer from the road-centre reference while the current
            # route session is still waiting for its first local plan.
            self._publish_command(0.0, 0.0, 1.0)
            return
        if self.planned_path and self.plan_time is not None:
            plan_age = self.get_clock().now().nanoseconds/1e9 - self.plan_time
            if plan_age < -0.1 or plan_age > float(self.get_parameter("plan_timeout").value):
                # A stale avoidance path is not permission to drive through
                # obstacles on the global centreline. Wait stopped for a plan.
                self._publish_command(0.0, 0.0, 1.0)
                return
        if self.last_control_stamp == self.state.timestamp:
            return
        if self.controller.lat.actuator_params.mode == "dynamic":
            if self.last_control_stamp is not None and not math.isclose(
                    self.state.timestamp-self.last_control_stamp, self.controller.lat.ts, abs_tol=1e-6):
                if not self.actuator_timing_fault:
                    self.get_logger().error("actuator command history lost time alignment; reset required")
                self.actuator_timing_fault = True
            if self.actuator_timing_fault:
                # Do not extrapolate a delay FIFO through missing physics ticks.
                # Latch a bounded hold-angle/brake request until a new run/reset.
                self._publish_command(self.state.steer, 0.0, 1.0)
                return
        self.last_control_stamp = self.state.timestamp
        self.controller.set_target_speed(
            self._target_speed_at(self.state.x, self.state.y)
        )
        steer, throttle, brake = self.controller.step(
            self.state.x,
            self.state.y,
            self.state.phi,
            self.state.vx,
            self.state.vy,
            self.state.r,
            actual_steer=self.state.steer,
        )
        if self.measurement_path is not self.controller.ref_path:
            # Transfer measurement state across local-plan updates separately
            # from the controller's predicted reference state.
            previous = self.tracking_monitor.tracker.projection
            self.tracking_monitor = TrackingMonitor(self.controller.ref_path)
            if previous is not None:
                self.tracking_monitor.tracker.s = self.tracking_monitor.tracker.geometry.project(
                    previous.x, previous.y, previous.theta).s
            self.measurement_path = self.controller.ref_path
        measured = self.tracking_monitor.update(self.state)
        measured.update(run_id=self.active_run, path_sequence=self.last_sequence,
                        reference_kind="planned" if self.planned_path else "global")
        self.tracking_pub.publish(String(data=json.dumps(measured)))
        self._publish_command(steer, throttle, brake)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
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
