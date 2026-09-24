import json

import rclpy
import pytest
from std_msgs.msg import String

from lightweight_sim.engine.ros_nodes import controller_manager as controller_manager_module
from lightweight_sim.engine.ros_nodes.controller_manager import ControllerManager


def test_scenario_reset_hold_uses_non_resetting_wall_clock(monkeypatch):
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = ControllerManager()
    wall_time = [100.0]
    monkeypatch.setattr(controller_manager_module.time, "monotonic", lambda: wall_time[0])
    try:
        node._timer.cancel()
        node._on_context(
            String(data=json.dumps({"schema_version": 1, "run_id": 1}))
        )

        assert node._hold_until == pytest.approx(100.0 + node.get_parameter("switch_hold_s").value)
        wall_time[0] += node.get_parameter("switch_hold_s").value + 0.01
        assert wall_time[0] >= node._hold_until
    finally:
        node.destroy_node()
        rclpy.shutdown()
