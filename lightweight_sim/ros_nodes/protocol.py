"""Small, versioned transport formats used before custom ROS messages land.

Path format: [version, sequence, point_count, x, y, theta, kappa, ...]
Obstacle format: [id, x, y, length, width, speed, heading, ...]
All values are SI units except where explicitly documented by a node parameter.
"""

from typing import Iterable, List, Sequence, Tuple

PATH_VERSION = 1.0
PATH_HEADER_SIZE = 3
PATH_POINT_SIZE = 4
OBSTACLE_SIZE = 7


def encode_path(path: Iterable[Sequence[float]], sequence: int = 0) -> List[float]:
    points = [tuple(float(value) for value in point[:4]) for point in path]
    return [PATH_VERSION, float(sequence), float(len(points))] + [
        value for point in points for value in point
    ]


def decode_path(data: Sequence[float]) -> Tuple[int, List[Tuple[float, float, float, float]]]:
    if len(data) < PATH_HEADER_SIZE:
        return 0, []
    version = float(data[0])
    if version != PATH_VERSION:
        raise ValueError(f"unsupported path protocol version: {version}")
    sequence = int(data[1])
    count = int(data[2])
    expected = PATH_HEADER_SIZE + count * PATH_POINT_SIZE
    if count < 0 or len(data) < expected:
        raise ValueError("truncated path message")
    points = []
    for offset in range(PATH_HEADER_SIZE, expected, PATH_POINT_SIZE):
        points.append(tuple(float(value) for value in data[offset : offset + PATH_POINT_SIZE]))
    return sequence, points


def encode_obstacles(obstacles: Iterable[object]) -> List[float]:
    values: List[float] = []
    for obstacle in obstacles:
        values.extend(
            (
                float(obstacle.id),
                float(obstacle.x),
                float(obstacle.y),
                float(obstacle.length),
                float(obstacle.width),
                float(obstacle.speed),
                float(obstacle.heading),
            )
        )
    return values


def decode_obstacles(data: Sequence[float]) -> List[Tuple[int, float, float, float, float, float, float]]:
    if len(data) % OBSTACLE_SIZE:
        raise ValueError("truncated obstacle message")
    result = []
    for offset in range(0, len(data), OBSTACLE_SIZE):
        values = data[offset : offset + OBSTACLE_SIZE]
        result.append((int(values[0]), *(float(value) for value in values[1:])))
    return result
