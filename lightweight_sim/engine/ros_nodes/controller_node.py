"""ROS 2 adapter for the lateral/longitudinal vehicle controller."""

from typing import Optional
import json
import math

import rclpy
from std_msgs.msg import String
from lightweight_sim_msgs.msg import ControlCommand, Path as RosPath
from lightweight_sim_msgs.msg import ReferenceLine as RosReferenceLine
from lightweight_sim_msgs.msg import SpeedProfile
import numpy as np
from ..algorithms.planner.st_speed import SpeedPlan
from lightweight_sim_msgs.msg import VehicleState as RosVehicleState
from rclpy.node import Node
from ..algorithms.controller.combined import VehicleController
from .message_conversions import message_to_state, path_to_tuples
from .qos import command_qos, latched_path_qos, sensor_data_qos
from .route_session import decode_sequence, parse_context
from ..analysis.tracking import TrackingMonitor
from ..simulator.data_types import VehicleParams
from ..simulator.steering import SteeringParams
from ..runtime_config import DEFAULT_RUNTIME_CONFIG


def maximum_path_curvature(path, x: float, y: float, lookahead_m: float) -> float:
    """Return the greatest absolute curvature near the pose and ahead on a path."""

    if not path:
        return 0.0
    nearest = min(
        range(len(path)),
        key=lambda index: (path[index][0] - x) ** 2 + (path[index][1] - y) ** 2,
    )
    maximum = abs(float(path[nearest][3]))
    distance = 0.0
    for index in range(nearest + 1, len(path)):
        previous, current = path[index - 1], path[index]
        distance += math.hypot(current[0] - previous[0], current[1] - previous[1])
        if distance > lookahead_m:
            break
        maximum = max(maximum, abs(float(current[3])))
    return maximum


def curvature_limited_speed_kmh(
    target_speed_kmh: float,
    maximum_curvature_1pm: float,
    maximum_lateral_accel_mps2: float,
) -> float:
    """Limit the route speed so local path curvature respects lateral acceleration."""

    if maximum_curvature_1pm <= 1e-6:
        return target_speed_kmh
    curve_speed_kmh = math.sqrt(
        maximum_lateral_accel_mps2 / maximum_curvature_1pm
    ) * 3.6
    return min(target_speed_kmh, curve_speed_kmh)


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_node")
        self.declare_parameter("controller", DEFAULT_RUNTIME_CONFIG.controller)
        self.declare_parameter("speed_planning_enabled", False)
        self.declare_parameter("speed_profile_timeout_s", .6)
        self.declare_parameter("physics_dt", DEFAULT_RUNTIME_CONFIG.physics_dt)
        self.declare_parameter("target_speed_kmh", DEFAULT_RUNTIME_CONFIG.target_speed_kmh)
        self.declare_parameter("control_period", DEFAULT_RUNTIME_CONFIG.control_period)
        self.declare_parameter("state_timeout", DEFAULT_RUNTIME_CONFIG.state_timeout)
        self.declare_parameter("plan_timeout", DEFAULT_RUNTIME_CONFIG.plan_timeout)
        self.declare_parameter("routing_reference_topic", "routing/reference_line")
        self.declare_parameter(
            "speed_profile_lookahead_m", DEFAULT_RUNTIME_CONFIG.speed_profile_lookahead_m
        )
        self.declare_parameter("output_topic", "control_command/cruise")
        self.declare_parameter("actuator_compensation", True)
        self.declare_parameter("lateral_q_weights", [200.0, 1.0, 50.0, 1.0])
        self.declare_parameter("lateral_r", 100.0)
        self.declare_parameter("dynamic_lateral_r", 300.0)
        self.declare_parameter("lateral_feedback_horizon_s", 0.0)
        self.declare_parameter("lateral_smooth_reference_heading", True)
        self.declare_parameter("lateral_discretization", "plant")
        self.declare_parameter("mpc_prediction_steps", 6)
        self.declare_parameter("mpc_control_steps", 2)
        self.declare_parameter("mpc_solver", "auto")
        self.declare_parameter("longitudinal_kp", 1.15)
        self.declare_parameter("longitudinal_ki", 0.0)
        self.declare_parameter("longitudinal_kd", 0.55)
        self.declare_parameter("longitudinal_error_threshold_kmh", 1.0)
        self.declare_parameter("longitudinal_max_jerk_mps3", 6.0)
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
        self._path_candidates = {}
        self._speed_candidates = {}
        self.speed_reference = None
        self.speed_stamp = None
        self._invalid_path_sequence = -1
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
        self._history_command_stamp = None
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
        self._pending_routing_reference = None

        sensor_qos = sensor_data_qos()
        self.state_sub = self.create_subscription(
            RosVehicleState, "vehicle/state", self._on_state, sensor_qos
        )
        self.planned_sub = self.create_subscription(
            RosPath, "planned_path", self._on_planned, latched_path_qos()
        )
        self.create_subscription(SpeedProfile, "speed_profile", self._on_speed, latched_path_qos())
        self.speed_tracking_pub = self.create_publisher(String, "speed/tracking", sensor_data_qos())
        self.command_pub = self.create_publisher(
            ControlCommand,
            str(self.get_parameter("output_topic").value),
            command_qos(),
        )
        period = float(self.get_parameter("control_period").value)
        self.timer = self.create_timer(period, self._on_timer)

    def _controller_options(self):
        context = getattr(self, "route_context", None) or {}
        return dict(
            dynamic_max_substep_s=float(context.get(
                "dynamic_max_substep_s", DEFAULT_RUNTIME_CONFIG.dynamic_max_substep_s)),
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
            mpc_params=dict(
                N=int(self.get_parameter("mpc_prediction_steps").value),
                P=int(self.get_parameter("mpc_control_steps").value),
                solver=str(self.get_parameter("mpc_solver").value),
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
        if self.reference_run != context["run_id"]:
            self.reference_path = []
            self.reference_run = None
        self.routing_speed_points = []
        self.routing_speed_s = []
        self.routing_speed_profile = []
        self.planned_path = []
        self.plan_time = None
        self.plan_ready = False
        self.state = None
        self.last_control_stamp = None
        self._history_command_stamp = None
        self.last_sequence = -1
        self._path_candidates.clear()
        self._speed_candidates.clear()
        self.speed_reference = self.speed_stamp = None
        self._invalid_path_sequence = -1
        pending_reference = self._pending_routing_reference
        if (
            pending_reference is not None
            and int(pending_reference.request_id) == context["run_id"]
        ):
            self._pending_routing_reference = None
            self._on_routing_reference(pending_reference)
        self._activate_reference()

    def _on_routing_reference(self, message: RosReferenceLine) -> None:
        """Use only the routing reference matching the active simulation run."""
        run_id = int(message.request_id)
        if self.route_context is None:
            self._pending_routing_reference = message
            return
        if self.route_context and run_id != self.route_context["run_id"]:
            if run_id > self.route_context["run_id"]:
                self._pending_routing_reference = message
            return
        if not message.success or len(message.points) < 2:
            self.reference_path = []
            self.reference_run = None
            self.active_run = None
            self.planned_path = []
            self.plan_ready = False
            self.routing_speed_points = []
            self.routing_speed_s = []
            self.routing_speed_profile = []
            return
        self.reference_path = [
            (float(point.x), float(point.y), float(point.theta), float(point.kappa))
            for point in message.points
        ]
        self.reference_run = run_id
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
        self._activate_reference()

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
        max_abs_curvature = max(
            max_abs_curvature,
            maximum_path_curvature(
                self.planned_path,
                x,
                y,
                self.speed_profile_lookahead_m,
            )
        )
        return curvature_limited_speed_kmh(
            target_speed,
            max_abs_curvature,
            self.max_lateral_accel_mps2,
        )

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

    def _on_planned(self, message: RosPath) -> None:
        run, version = decode_sequence(message.sequence)
        if run != self.active_run or not version or int(message.sequence) <= self.last_sequence:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9 - stamp
        if age < -0.1 or age > float(self.get_parameter("plan_timeout").value):
            return
        if bool(self.get_parameter("speed_planning_enabled").value):
            sequence = int(message.sequence)
            if sequence <= self._invalid_path_sequence:
                return
            if len(message.points) < 2:
                self._invalid_path_sequence = sequence
                self._path_candidates.clear()
                self._speed_candidates.clear()
                self.speed_reference = self.speed_stamp = None
                self.plan_ready = False
                return
            self._path_candidates[sequence] = (path_to_tuples(message), stamp)
            while len(self._path_candidates) > 16:
                del self._path_candidates[min(self._path_candidates)]
            pending = self._speed_candidates.pop(sequence, None)
            if pending is not None:
                self._on_speed(pending)
            return
        self.last_sequence = int(message.sequence)
        self.planned_path = path_to_tuples(message)
        self.plan_time = stamp
        self.plan_ready = bool(self.planned_path)
        self.controller.update_ref_path(self.planned_path or self.reference_path, reset=False)

    def _on_speed(self, message):
        if not bool(self.get_parameter("speed_planning_enabled").value):
            return
        run, version = decode_sequence(message.path_sequence)
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        if (run != self.active_run or message.run_id != run or not version
                or message.path_sequence <= self._invalid_path_sequence
                or message.path_sequence < self.last_sequence
                or not -.001 <= age <= float(self.get_parameter("speed_profile_timeout_s").value)
                or (self.speed_stamp is not None and stamp < self.speed_stamp)):
            return
        if not message.valid:
            self.speed_reference = None
            self.plan_ready = False
            self.controller.lon.reset()
            return
        points = message.points
        if len(points) < 2:
            return
        data = np.array([[p.time_from_start, p.s, p.speed, p.acceleration, p.jerk] for p in points])
        if (not np.isfinite(data).all() or not math.isfinite(message.origin_s) or abs(data[0, 0]) > 1e-8
                or np.any(np.diff(data[:, 0]) <= 0) or np.any(data[:, 2] < 0)
                or np.any(np.diff(data[:, 1]) < -.002)):
            return
        # Verify the advertised interpolation follows its knot states.
        for first, second in zip(data[:-1], data[1:]):
            dt = second[0]-first[0]
            s, v, a, j = first[1:]
            expected = [s+v*dt+.5*a*dt**2+j*dt**3/6, v+a*dt+.5*j*dt**2, a+j*dt]
            if np.max(np.abs(np.array(expected)-second[1:4])) > .01:
                return
        if message.path_sequence not in self._path_candidates:
            self._speed_candidates[message.path_sequence] = message
            while len(self._speed_candidates) > 16:
                del self._speed_candidates[min(self._speed_candidates)]
            return
        path, path_stamp = self._path_candidates[message.path_sequence]
        if self.get_clock().now().nanoseconds/1e9-path_stamp > float(self.get_parameter("plan_timeout").value):
            return
        reference = SpeedPlan(*[data[:, i].copy() for i in range(4)], data[:-1, 4].copy(),
                              message.origin_s, None, 'solved')
        try:
            _, v, _ = reference.sample(age)
        except ValueError:
            return
        if self.state is not None and abs(v-self.state.speed) > 1.5:
            return
        self.speed_reference, self.speed_stamp = reference, stamp
        self.last_sequence = int(message.path_sequence)
        self.planned_path, self.plan_time, self.plan_ready = path, path_stamp, True
        self.controller.update_ref_path(path, reset=False)

    def _publish_command(self, steer: float, throttle: float, brake: float,
                         reset_longitudinal: bool = True) -> None:
        # Legacy/direct-writer inputs without plant FIFO feedback still account
        # for a brake/hold command once per physics state, including early exits.
        if (self.active_run is not None and self.state is not None
                and self.controller.lat.actuator_params.mode == "dynamic"
                and self.state.steering_delay_queue is None
                and self._history_command_stamp != self.state.timestamp):
            history = self.controller.lat.command_history
            if history:
                self.controller.lat.command_history = history[1:]+[float(steer)]
            self._history_command_stamp = self.state.timestamp
        message = ControlCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.steering_angle = float(steer)
        message.throttle = float(throttle)
        message.brake = float(brake)
        message.gear = 1
        self.command_pub.publish(message)
        if reset_longitudinal and brake >= .999:
            self.controller.lon.reset()

    def _on_timer(self) -> None:
        if self.active_run is None or self.state is None or self.state_time is None:
            self._publish_command(0.0, 0.0, 1.0)
            return
        age = (self.get_clock().now() - self.state_time).nanoseconds / 1e9
        timeout = float(self.get_parameter("state_timeout").value)
        if age < 0 or age > timeout:
            self._publish_command(0.0, 0.0, 1.0)
            return
        if self.last_control_stamp == self.state.timestamp:
            return
        if self.controller.lat.actuator_params.mode == "dynamic":
            elapsed = (None if self.last_control_stamp is None
                       else self.state.timestamp-self.last_control_stamp)
            try:
                if not math.isfinite(self.state.timestamp) or (elapsed is not None and elapsed <= 0):
                    raise ValueError('actuator state timestamp is invalid or moved backwards')
                feedback = self.state.steering_delay_queue
                if feedback is not None:
                    # Same timestamp as measured angle: actual plant inputs after
                    # arbitration, safe-stop and manual/parking overrides.
                    self.controller.lat.synchronize_actuator_state(self.state.steer, feedback)
                elif elapsed is not None and not math.isclose(elapsed, self.controller.lat.ts, abs_tol=1e-6):
                    raise ValueError('actuator command history lost time alignment without plant feedback')
            except ValueError as exc:
                if not self.actuator_timing_fault:
                    self.get_logger().error(f'{exc}; reset required')
                self.actuator_timing_fault = True
            if self.actuator_timing_fault:
                self._publish_command(self.state.steer, 0.0, 1.0)
                return
        # Account for every fresh physics state, even when no path is ready.
        self.last_control_stamp = self.state.timestamp
        if self.route_context and self.route_context.get("maneuver") == "reverse_parking":
            # Reverse parking has a separate controller candidate.  The cruise
            # controller must stay neutral even if it was launched directly.
            self._publish_command(0.0, 0.0, 1.0)
            return
        if not self.reference_path:
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
        reference_accel = None
        if bool(self.get_parameter("speed_planning_enabled").value):
            elapsed = self.state.timestamp-(self.speed_stamp or 0.)
            if (self.speed_reference is None or self.speed_stamp is None
                    or not 0 <= elapsed <= float(self.get_parameter("speed_profile_timeout_s").value)):
                self._publish_command(self.state.steer, 0., 1.)
                return
            try:
                reference_s, reference_speed, reference_accel = self.speed_reference.sample(elapsed)
            except ValueError:
                self._publish_command(self.state.steer, 0., 1.)
                return
            if reference_speed < .05 and reference_accel <= .01 and self.state.speed < .15:
                self._publish_command(self.state.steer, 0., 1.)
                return
            self.controller.set_target_speed(reference_speed*3.6)
            self.speed_tracking_pub.publish(String(data=json.dumps(dict(run_id=self.active_run,
                path_sequence=self.last_sequence, reference_speed_mps=reference_speed,
                reference_accel_mps2=reference_accel, measured_speed_mps=self.state.speed,
                reference_s=reference_s))))
        else:
            reference_speed = self._target_speed_at(self.state.x, self.state.y) / 3.6
            self.controller.set_target_speed(reference_speed * 3.6)
        try:
            steer, throttle, brake = self.controller.step(
                self.state.x,
                self.state.y,
                self.state.phi,
                self.state.vx,
                self.state.vy,
                self.state.r,
                actual_steer=self.state.steer,
                reference_accel=reference_accel,
            )
            self._history_command_stamp = self.state.timestamp
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(f"controller failed; braking: {exc}")
            self.actuator_timing_fault = self.controller.lat.actuator_params.mode == "dynamic"
            self._publish_command(self.state.steer, 0.0, 1.0)
            return
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
                        reference_kind="planned" if self.planned_path else "global",
                        reference_speed_mps=reference_speed,
                        measured_speed_mps=self.state.speed,
                        speed_error_kmh=(reference_speed-self.state.speed)*3.6)
        self.tracking_pub.publish(String(data=json.dumps(measured)))
        # Saturation is a normal PID output, not a protective stop. Clearing
        # its memory at full braking restarts the jerk ramp from zero and
        # creates repeated -6/-2/-4 acceleration pulses.
        self._publish_command(steer, throttle, brake, False)


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
