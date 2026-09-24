"""Small geometry helpers with no external dependencies."""

import math
from typing import Iterable, List, Sequence, Tuple

from .types import BoxObstacle, Pose2D, wrap_angle

Point = Tuple[float, float]


def rotate(point: Point, angle: float) -> Point:
    c, s = math.cos(angle), math.sin(angle)
    return point[0] * c - point[1] * s, point[0] * s + point[1] * c


def rectangle_corners(x: float, y: float, length: float, width: float, heading: float) -> List[Point]:
    half_l, half_w = length / 2.0, width / 2.0
    local = [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]
    return [(x + px, y + py) for px, py in (rotate(point, heading) for point in local)]


def project(points: Sequence[Point], axis: Point) -> Tuple[float, float]:
    values = [point[0] * axis[0] + point[1] * axis[1] for point in points]
    return min(values), max(values)


def boxes_overlap(first: Sequence[Point], second: Sequence[Point], margin: float = 0.0) -> bool:
    axes: List[Point] = []
    for polygon in (first, second):
        for p0, p1 in zip(polygon, polygon[1:] + polygon[:1]):
            edge = (p1[0] - p0[0], p1[1] - p0[1])
            length = math.hypot(edge[0], edge[1])
            if length > 1e-9:
                axes.append((-edge[1] / length, edge[0] / length))
    for axis in axes:
        a0, a1 = project(first, axis)
        b0, b1 = project(second, axis)
        if a1 + margin < b0 or b1 + margin < a0:
            return False
    return True


def vehicle_corners(pose: Pose2D, length: float, width: float) -> List[Point]:
    return rectangle_corners(pose.x, pose.y, length, width, pose.yaw)


def obstacle_corners(obstacle: BoxObstacle, margin: float = 0.0) -> List[Point]:
    return rectangle_corners(
        obstacle.x,
        obstacle.y,
        obstacle.length + 2.0 * margin,
        obstacle.width + 2.0 * margin,
        obstacle.heading,
    )


def collides(pose: Pose2D, obstacles: Iterable[BoxObstacle], length: float, width: float, margin: float) -> bool:
    body = vehicle_corners(pose, length, width)
    return any(boxes_overlap(body, obstacle_corners(obstacle), margin=margin) for obstacle in obstacles)


def hermite_point(p0: Point, p1: Point, m0: Point, m1: Point, t: float) -> Point:
    t2, t3 = t * t, t * t * t
    h00, h10 = 2 * t3 - 3 * t2 + 1, t3 - 2 * t2 + t
    h01, h11 = -2 * t3 + 3 * t2, t3 - t2
    return (
        h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0],
        h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1],
    )


def hermite_derivative(p0: Point, p1: Point, m0: Point, m1: Point, t: float) -> Point:
    t2 = t * t
    h00, h10 = 6 * t2 - 6 * t, 3 * t2 - 4 * t + 1
    h01, h11 = -6 * t2 + 6 * t, 3 * t2 - 2 * t
    return (
        h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0],
        h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1],
    )


def derivative_heading(derivative: Point, reverse: bool) -> float:
    tangent = math.atan2(derivative[1], derivative[0])
    return wrap_angle(tangent + math.pi if reverse else tangent)
