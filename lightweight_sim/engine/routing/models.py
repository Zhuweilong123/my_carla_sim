"""Data model for lane-level road routing.

The routing core deliberately does not depend on ROS messages.  ROS adapters
convert these immutable objects to and from ``lightweight_sim_msgs``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point2D = Tuple[float, float]
PathTuple = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class RouteRequest:
    map_id: str
    start: Pose2D
    goal: Pose2D
    start_lane: int = -1
    goal_lane: int = -1
    route_policy: str = "fastest"
    allow_u_turn: bool = False


@dataclass(frozen=True)
class MapNode:
    node_id: str
    x: float
    y: float


@dataclass(frozen=True)
class LaneEdge:
    edge_id: str
    road_id: str
    lane_id: str
    from_node: str
    to_node: str
    lane_index: int
    centerline: Tuple[Point2D, ...]
    left_boundary: Tuple[Point2D, ...] = ()
    right_boundary: Tuple[Point2D, ...] = ()
    speed_limit_kmh: float = 40.0
    successors: Tuple[str, ...] = ()
    maneuver: str = "straight"
    bidirectional: bool = False

    @property
    def length(self) -> float:
        total = 0.0
        for first, second in zip(self.centerline[:-1], self.centerline[1:]):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            total += (dx * dx + dy * dy) ** 0.5
        return total

    @property
    def start(self) -> Point2D:
        return self.centerline[0]

    @property
    def end(self) -> Point2D:
        return self.centerline[-1]


@dataclass(frozen=True)
class RoadMap:
    map_id: str
    nodes: Dict[str, MapNode]
    edges: Dict[str, LaneEdge]
    lane_width: float = 3.5
    num_lanes: int = 1

    def outgoing(self, edge_id: str) -> Tuple[LaneEdge, ...]:
        edge = self.edges[edge_id]
        return tuple(self.edges[successor] for successor in edge.successors)

    def edges_for_lane(self, lane_index: int) -> Iterable[LaneEdge]:
        return (edge for edge in self.edges.values() if edge.lane_index == lane_index)

    def nearest_edge(self, pose: Pose2D, lane_index: int = -1) -> LaneEdge:
        candidates = list(self.edges.values())
        if lane_index >= 0:
            lane_candidates = [edge for edge in candidates if edge.lane_index == lane_index]
            if lane_candidates:
                candidates = lane_candidates
        if not candidates:
            raise ValueError("road map contains no routable lane edges")

        def score(edge: LaneEdge) -> float:
            distance = min(
                (pose.x - point[0]) ** 2 + (pose.y - point[1]) ** 2
                for point in edge.centerline
            )
            return distance

        return min(candidates, key=score)


@dataclass(frozen=True)
class RouteSegment:
    edge_id: str
    road_id: str
    lane_id: str
    lane_index: int
    maneuver: str
    length_m: float
    speed_limit_kmh: float


@dataclass(frozen=True)
class RoutePlan:
    route_id: int
    request_id: int
    map_id: str
    success: bool
    failure_reason: str = ""
    segments: Tuple[RouteSegment, ...] = ()
    points: Tuple[PathTuple, ...] = ()
    total_length_m: float = 0.0
    target_lane: int = -1
    metadata: Dict[str, str] = field(default_factory=dict)

    @classmethod
    def failure(cls, route_id: int, request_id: int, map_id: str, reason: str):
        return cls(
            route_id=route_id,
            request_id=request_id,
            map_id=map_id,
            success=False,
            failure_reason=reason,
        )
