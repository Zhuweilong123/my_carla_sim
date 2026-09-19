"""ROS 2 simulator node with runtime scenario switching."""

from typing import Optional
import json
import time
from dataclasses import asdict

import rclpy
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

from ._simulator_node_impl import *  # noqa: F401,F403
from ..simulator.data_types import ControlCommand
from ..simulator.engine import SimulationEngine
from ..simulator.scenarios import make_scenario
from ..simulator.steering import steering_profile
from .route_session import encode_sequence


_LegacySimulatorNode = SimulatorNode


class SimulatorNode(_LegacySimulatorNode):
    """Add a parameter callback that reloads the selected scenario."""

    def __init__(self) -> None:
        super().__init__()
        self.add_on_set_parameters_callback(self._on_parameters)

    def _publish_reference(self, sequence=0):
        # Called at startup, reset and scenario switch, including by the base
        # constructor. The context and reference are paired by this run ID.
        if not hasattr(self, "context_pub"):
            self.context_pub = self.create_publisher(String, "sim/context", latched_path_qos())
        self.run_id = max(int(time.time()*1000), getattr(self, "run_id", 0)+1)
        config = self.engine.config
        context = dict(schema_version=1, run_id=self.run_id,
                       route_id=config.name, scenario=config.name,
                       target_speed_kmh=config.target_speed,
                       vehicle_model=config.vehicle_model,
                       vehicle_parameters=asdict(config.vehicle_params),
                       steering_parameters=asdict(config.steering),
                       lane_width=config.road.lane_width,
                       num_lanes=config.road.num_lanes, physics_dt=self.physics_dt)
        self.context_pub.publish(String(data=json.dumps(context)))
        super()._publish_reference(encode_sequence(self.run_id))

    def _on_parameters(self, parameters):
        requested = None
        for parameter in parameters:
            if parameter.name == "steering_profile":
                return SetParametersResult(successful=False,
                    reason="steering_profile is startup-only; restart to change the actuator")
            if parameter.name == "scenario":
                if not isinstance(parameter.value, str):
                    return SetParametersResult(
                        successful=False,
                        reason="scenario must be a string",
                    )
                requested = parameter.value

        if requested is None:
            return SetParametersResult(successful=True)

        try:
            config = make_scenario(requested)
            config.steering = steering_profile(str(self.get_parameter("steering_profile").value))
            config.steering.delay_steps(self.physics_dt)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        self.engine = SimulationEngine(config)
        self.command = ControlCommand()
        self.last_command_time = self.get_clock().now()
        self.paused = False
        self._publish_reference(sequence=0)
        self._publish_state()
        self._publish_status()
        self.get_logger().info(
            f"scenario switched to {config.name} (key: {requested})"
        )
        return SetParametersResult(successful=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[SimulatorNode] = None
    try:
        node = SimulatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass
