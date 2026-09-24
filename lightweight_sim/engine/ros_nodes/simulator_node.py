"""ROS 2 simulator node with runtime scenario switching."""

from typing import Optional
import json
import time
from dataclasses import asdict

import rclpy
from std_msgs.msg import String
from lightweight_sim_msgs.msg import RouteRequest
from rcl_interfaces.msg import SetParametersResult

from ._simulator_node_impl import *  # noqa: F401,F403
from ..simulator.data_types import ControlCommand
from ..simulator.engine import SimulationEngine
from ..simulator.scenarios import make_scenario


_LegacySimulatorNode = SimulatorNode


class SimulatorNode(_LegacySimulatorNode):
    """Add a parameter callback that reloads the selected scenario."""

    def __init__(self) -> None:
        super().__init__()
        self.add_on_set_parameters_callback(self._on_parameters)

    def _publish_run_context(self, sequence=0):
        # Called at startup, reset and scenario switch. The context and route
        # request are paired by this run ID.
        if not hasattr(self, "context_pub"):
            self.context_pub = self.create_publisher(String, "sim/context", latched_path_qos())
        self.run_id = max(int(time.time()*1000), getattr(self, "run_id", 0)+1)
        config = self.engine.config
        reference_lane_index = int(getattr(config, "routing_start_lane", -1))
        if not 0 <= reference_lane_index < config.road.num_lanes:
            lane_centers = [
                (-config.road.num_lanes / 2.0 + lane + 0.5)
                * config.road.lane_width
                for lane in range(config.road.num_lanes)
            ]
            distances = sorted(
                (abs(float(config.ego_start_y) - center), lane)
                for lane, center in enumerate(lane_centers)
            )
            if (
                distances
                and distances[0][0] < config.road.lane_width / 2.0
                and (
                    len(distances) == 1
                    or distances[0][0] + 1e-6 < distances[1][0]
                )
            ):
                reference_lane_index = distances[0][1]
        context = dict(schema_version=1, run_id=self.run_id,
                       route_id=config.name, scenario=config.name,
                       target_speed_kmh=config.target_speed,
                       speed_limit_kmh=config.speed_limit_kmh,
                       target_speed_ratio=config.target_speed_ratio,
                       speed_limits=config.speed_limits,
                       max_lateral_accel_mps2=config.max_lateral_accel_mps2,
                       speed_limit_type=getattr(config, "speed_limit_type", "default"),
                       vehicle_model=config.vehicle_model,
                       vehicle_parameters=asdict(config.vehicle_params),
                       steering_parameters=asdict(config.steering),
                       maneuver=getattr(config, "maneuver", "cruise"),
                       parking_goal=getattr(config, "parking_goal", None),
                       routing_map_id=getattr(config, "routing_map_id", None),
                       routing_start_lane=getattr(config, "routing_start_lane", -1),
                       routing_goal_lane=getattr(config, "routing_goal_lane", -1),
                       routing_closed_loop=getattr(config, "routing_closed_loop", False),
                       reference_lane_index=reference_lane_index,
                       lane_width=config.road.lane_width,
                       num_lanes=config.road.num_lanes,
                       road_network=config.road.road_network,
                       road_network_num_lanes=(
                           config.road.road_network_num_lanes
                           or config.road.num_lanes
                       ),
                       physics_dt=self.physics_dt)
        self.context_pub.publish(String(data=json.dumps(context)))
        self._publish_route_request(config)

    def _publish_route_request(self, config) -> None:
        """Publish the scenario mission once per run/reset for the routing node."""

        if not hasattr(self, "route_request_pub"):
            self.route_request_pub = self.create_publisher(
                RouteRequest, "routing/request", latched_path_qos()
            )
        destination = getattr(config, "destination", None)
        map_id = getattr(config, "routing_map_id", None)
        closed_loop = bool(getattr(config, "routing_closed_loop", False))
        if not map_id or (destination is None and not closed_loop):
            return

        message = RouteRequest()
        message.header.stamp = seconds_to_time(self.engine.sim_time)
        message.header.frame_id = self.frame_id
        message.request_id = self.run_id
        message.map_id = str(map_id)
        message.start_x = float(config.ego_start_x)
        message.start_y = float(config.ego_start_y)
        message.start_yaw = float(config.ego_start_phi)
        message.goal_x = float(destination[0] if destination else config.ego_start_x)
        message.goal_y = float(destination[1] if destination else config.ego_start_y)
        message.goal_yaw = float(config.ego_start_phi)
        message.start_lane = int(getattr(config, "routing_start_lane", -1))
        message.goal_lane = int(getattr(config, "routing_goal_lane", -1))
        message.route_policy = "fastest"
        message.allow_u_turn = False
        message.closed_loop = closed_loop
        self.route_request_pub.publish(message)

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
            attach_map_road_network(config)
            config.physics_dt = self.physics_dt
            self._apply_runtime_parameters(config)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        self.engine = SimulationEngine(config)
        self.command = ControlCommand()
        self.last_command_time = self.get_clock().now()
        self.paused = False
        self._publish_run_context(sequence=0)
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
