"""ROS 2 simulator node with runtime scenario switching."""

from typing import Optional

import rclpy
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

    def _on_parameters(self, parameters):
        requested = None
        for parameter in parameters:
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
