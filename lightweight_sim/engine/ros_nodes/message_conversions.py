"""Conversions shared by ROS nodes at the message/domain boundary."""

from lightweight_sim_msgs.msg import Path as RosPath
from lightweight_sim_msgs.msg import VehicleState as RosVehicleState

from ..simulator.data_types import VehicleState


def message_to_state(message: RosVehicleState) -> VehicleState:
    """Convert the ROS vehicle-state message to the simulator domain model."""

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


def path_to_tuples(message: RosPath) -> list[tuple[float, float, float, float]]:
    """Convert a ROS path to internal (x, y, heading, curvature) tuples."""

    return [(point.x, point.y, point.theta, point.kappa) for point in message.points]
