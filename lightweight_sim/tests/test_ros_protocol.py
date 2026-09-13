import pytest

from lightweight_sim.ros_nodes.protocol import (
    decode_obstacles,
    decode_path,
    encode_obstacles,
    encode_path,
)
from lightweight_sim.simulator.data_types import Obstacle


def test_path_protocol_preserves_sequence_and_curvature():
    path = [(1.0, 2.0, 0.3, 0.01), (3.0, 4.0, 0.4, 0.02)]
    sequence, decoded = decode_path(encode_path(path, sequence=7))
    assert sequence == 7
    assert decoded == pytest.approx(path)


def test_obstacle_protocol_round_trip():
    obstacles = [Obstacle(id=3, x=10.0, y=-1.0, speed=2.0, heading=0.1)]
    decoded = decode_obstacles(encode_obstacles(obstacles))
    assert decoded == [(3, 10.0, -1.0, 4.5, 2.0, 2.0, 0.1)]


def test_path_protocol_rejects_truncated_message():
    with pytest.raises(ValueError, match="truncated"):
        decode_path([1.0, 1.0, 1.0])
