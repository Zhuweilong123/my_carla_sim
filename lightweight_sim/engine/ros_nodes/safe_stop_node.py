"""Fail-closed planning readiness supervisor for actuator arbitration."""

import json
import time

import rclpy
from lightweight_sim_msgs.msg import Path as RosPath
from lightweight_sim_msgs.msg import ReferenceLine as RosReferenceLine
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .qos import command_qos, latched_path_qos
from .route_session import decode_sequence


class SafeStopNode(Node):
    """Assert a brake request unless this run has a fresh usable plan.

    This node does not publish actuator commands.  The controller manager is
    the sole publisher to ``control_command`` and treats a missing heartbeat
    from this supervisor as a stop request.
    """

    def __init__(self) -> None:
        super().__init__("safe_stop_node")
        self.declare_parameter("planned_path_timeout_s", 0.25)
        self.declare_parameter("update_period_s", 0.05)
        self.declare_parameter("reference_topic", "routing/reference_line")
        self.declare_parameter("stop_topic", "safety/stop_request")
        self._context = None
        self._reference_run = None
        self._reference_ready = False
        self._plan_run = None
        self._plan_received_at = None
        self._last_reason = None
        self._stop_pub = self.create_publisher(
            Bool, str(self.get_parameter("stop_topic").value), command_qos()
        )
        self.create_subscription(String, "sim/context", self._on_context, latched_path_qos())
        self.create_subscription(
            RosReferenceLine,
            str(self.get_parameter("reference_topic").value),
            self._on_reference,
            latched_path_qos(),
        )
        self.create_subscription(RosPath, "planned_path", self._on_plan, latched_path_qos())
        self._timer = self.create_timer(
            float(self.get_parameter("update_period_s").value), self._publish_status
        )
        self._publish_status()

    def _on_context(self, message: String) -> None:
        try:
            context = json.loads(message.data)
            run_id = int(context["run_id"])
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return
        if self._context and run_id <= int(self._context["run_id"]):
            return
        self._context = context
        if self._reference_run != run_id:
            self._reference_run = None
            self._reference_ready = False
        if self._plan_run != run_id:
            self._plan_run = None
            self._plan_received_at = None

    def _on_reference(self, message: RosReferenceLine) -> None:
        run_id = int(message.request_id)
        if self._context is not None and run_id != int(self._context["run_id"]):
            if run_id < int(self._context["run_id"]):
                return
        self._reference_run = run_id
        self._reference_ready = bool(message.success and len(message.points) >= 2)

    def _on_plan(self, message: RosPath) -> None:
        run_id, version = decode_sequence(message.sequence)
        if version <= 0:
            return
        if self._context is not None and run_id != int(self._context["run_id"]):
            if run_id < int(self._context["run_id"]):
                return
        self._plan_run = run_id
        self._plan_received_at = (
            time.monotonic() if len(message.points) >= 2 else None
        )

    def _stop_reason(self) -> str:
        if self._context is None:
            return "waiting for simulation context"
        if self._context.get("routing_map_id"):
            if self._reference_run != int(self._context["run_id"]):
                return "waiting for matching routing reference line"
            if not self._reference_ready:
                return "routing reference line failed"
        if (
            self._plan_run != int(self._context["run_id"])
            or self._plan_received_at is None
        ):
            return "waiting for matching local plan"
        if time.monotonic() - self._plan_received_at > float(
            self.get_parameter("planned_path_timeout_s").value
        ):
            return "local plan heartbeat stale"
        return ""

    def _publish_status(self) -> None:
        reason = self._stop_reason()
        stop = Bool(data=bool(reason))
        self._stop_pub.publish(stop)
        if reason != self._last_reason:
            if reason:
                self.get_logger().warning(f"safety stop asserted: {reason}")
            else:
                self.get_logger().info("safety stop released: current route and plan are ready")
            self._last_reason = reason


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
