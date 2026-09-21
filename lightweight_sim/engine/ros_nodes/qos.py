"""Shared ROS 2 QoS profiles for the simulator graph."""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def sensor_data_qos() -> QoSProfile:
    """Keep only the newest high-rate state or obstacle sample."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def command_qos() -> QoSProfile:
    """Deliver the latest control command reliably without queue buildup."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def latched_path_qos() -> QoSProfile:
    """Retain the latest path for nodes that start after the publisher."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def clock_qos() -> QoSProfile:
    """Match the standard ROS 2 /clock best-effort volatile policy."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos() -> QoSProfile:
    """Retain the latest low-rate simulator status for late subscribers."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
