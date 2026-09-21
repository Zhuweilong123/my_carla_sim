"""ROS 2 adapter for the lateral/longitudinal vehicle controller."""

from typing import Optional
import json
import math

import rclpy
from std_msgs.msg import String
from lightweight_sim_msgs.msg import ControlCommand, Path as RosPath
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
        self.declare_parameter("target_speed_kmh", DEFAULT_RUNTIME_CONFIG.target_speed_kmh)
        self.declare_parameter("control_period", DEFAULT_RUNTIME_CONFIG.control_period)
        self.declare_parameter("state_timeout", DEFAULT_RUNTIME_CONFIG.state_timeout)
        self.declare_parameter("plan_timeout", DEFAULT_RUNTIME_CONFIG.plan_timeout)
        self.controller = VehicleController(
            vehicle_params=VehicleParams(),
            steering_params=SteeringParams(),
            controller_type=str(self.get_parameter("controller").value),
            target_speed_kmh=float(self.get_parameter("target_speed_kmh").value),
            dt=DEFAULT_RUNTIME_CONFIG.physics_dt,
        )
        self.state: Optional[object] = None
        self.state_time = None
        self.reference_path = []
        self.planned_path = []
        self.last_sequence = -1
        self.route_context = None
        self.reference_run = None
        self.active_run = None
        self.plan_time = None
        self.last_control_stamp = None
        self.actuator_timing_fault = False
        self.tracking_monitor = None
        self.measurement_path = None
        self.tracking_pub = self.create_publisher(String, "tracking/metrics", sensor_data_qos())
        self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())

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
            ControlCommand, "control_command", command_qos()
        )
        period = float(self.get_parameter("control_period").value)
        self.timer = self.create_timer(period, self._on_timer)

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
        self.actuator_timing_fault = False
        self.active_run = None
        self.planned_path = []
        self.plan_time = None
        self.state = None
        self.last_control_stamp = None
        self.last_sequence = -1
        self._activate_reference()

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
            dt=float(self.route_context["physics_dt"]))
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
        age = (self.get_clock().now() - self.state_time).nanoseconds / 1e9
        timeout = float(self.get_parameter("state_timeout").value)
        if age < 0 or age > timeout or not self.reference_path:
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
            float(self.get_parameter("target_speed_kmh").value)
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
