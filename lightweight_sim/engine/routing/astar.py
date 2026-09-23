"""Lane-level A* search and route geometry construction."""

from __future__ import annotations

import heapq
import itertools
import math
from typing import Dict, Iterable, List, Optional, Tuple

from .models import LaneEdge, PathTuple, RoutePlan, RouteRequest, RouteSegment, RoadMap


class AStarRouter:
    """Search a validated directed lane graph.

    Edge length is the primary cost.  ``fastest`` additionally uses the edge
    speed limit, while all policies apply configurable turn and lane-change
    penalties.  Dynamic obstacles intentionally stay outside this module.
    """

    def __init__(
        self,
        road_map: RoadMap,
        *,
        turn_penalty_s: float = 2.0,
        lane_change_penalty_s: float = 1.0,
        u_turn_penalty_s: float = 30.0,
        sample_spacing_m: float = 1.0,
    ):
        self.road_map = road_map
        self.turn_penalty_s = float(turn_penalty_s)
        self.lane_change_penalty_s = float(lane_change_penalty_s)
        self.u_turn_penalty_s = float(u_turn_penalty_s)
        self.sample_spacing_m = float(sample_spacing_m)
        if self.sample_spacing_m <= 0.0:
            raise ValueError("sample spacing must be positive")

    def search(
        self,
        request: RouteRequest,
        *,
        route_id: int,
        request_id: int,
    ) -> RoutePlan:
        if request.map_id and request.map_id != self.road_map.map_id:
            return RoutePlan.failure(
                route_id, request_id, request.map_id,
                f"map mismatch: requested {request.map_id}, loaded {self.road_map.map_id}",
            )

        try:
            start_edge = self.road_map.nearest_edge(request.start, request.start_lane)
            goal_edge = self.road_map.nearest_edge(request.goal, request.goal_lane)
        except ValueError as exc:
            return RoutePlan.failure(route_id, request_id, self.road_map.map_id, str(exc))

        open_set: List[Tuple[float, int, str]] = []
        sequence = itertools.count()
        heapq.heappush(open_set, (0.0, next(sequence), start_edge.edge_id))
        costs: Dict[str, float] = {start_edge.edge_id: 0.0}
        previous: Dict[str, Optional[str]] = {start_edge.edge_id: None}
        visited = set()

        while open_set:
            _, _, edge_id = heapq.heappop(open_set)
            if edge_id in visited:
                continue
            visited.add(edge_id)
            if edge_id == goal_edge.edge_id:
                edge_ids = self._reconstruct(previous, edge_id)
                return self._build_plan(
                    request, route_id, request_id, edge_ids, goal_edge.lane_index
                )

            current = self.road_map.edges[edge_id]
            for successor in self.road_map.outgoing(edge_id):
                if not request.allow_u_turn and successor.maneuver.lower() in {
                    "u_turn", "uturn", "u-turn"
                }:
                    continue
                candidate = costs[edge_id] + self._transition_cost(current, successor, request)
                if candidate >= costs.get(successor.edge_id, float("inf")):
                    continue
                costs[successor.edge_id] = candidate
                previous[successor.edge_id] = edge_id
                priority = candidate + self._heuristic(successor, request.goal, request.route_policy)
                heapq.heappush(open_set, (priority, next(sequence), successor.edge_id))

        return RoutePlan.failure(
            route_id,
            request_id,
            self.road_map.map_id,
            f"no route from {start_edge.edge_id} to {goal_edge.edge_id}",
        )

    def _transition_cost(self, current: LaneEdge, successor: LaneEdge, request: RouteRequest) -> float:
        speed_mps = max(1.0, successor.speed_limit_kmh / 3.6)
        if request.route_policy.lower() == "shortest":
            cost = successor.length
        else:
            cost = successor.length / speed_mps
        maneuver = successor.maneuver.lower()
        if maneuver not in {"", "straight"}:
            cost += self.turn_penalty_s
        if current.road_id == successor.road_id and current.lane_index != successor.lane_index:
            cost += self.lane_change_penalty_s
        if maneuver in {"u_turn", "uturn", "u-turn"}:
            cost += self.u_turn_penalty_s
        return cost

    @staticmethod
    def _heuristic(edge: LaneEdge, goal, policy: str) -> float:
        distance = math.hypot(edge.end[0] - goal.x, edge.end[1] - goal.y)
        if policy.lower() == "shortest":
            return distance
        return distance / max(1.0, edge.speed_limit_kmh / 3.6)

    @staticmethod
    def _reconstruct(previous: Dict[str, Optional[str]], edge_id: str) -> Tuple[str, ...]:
        result: List[str] = []
        current: Optional[str] = edge_id
        while current is not None:
            result.append(current)
            current = previous[current]
        return tuple(reversed(result))

    def _build_plan(
        self,
        request: RouteRequest,
        route_id: int,
        request_id: int,
        edge_ids: Iterable[str],
        target_lane: int,
    ) -> RoutePlan:
        edges = [self.road_map.edges[edge_id] for edge_id in edge_ids]
        segments = tuple(
            RouteSegment(
                edge_id=edge.edge_id,
                road_id=edge.road_id,
                lane_id=edge.lane_id,
                lane_index=edge.lane_index,
                maneuver=edge.maneuver,
                length_m=edge.length,
                speed_limit_kmh=edge.speed_limit_kmh,
            )
            for edge in edges
        )
        points = _build_reference_path(
            edges,
            sample_spacing_m=self.sample_spacing_m,
        )
        return RoutePlan(
            route_id=route_id,
            request_id=request_id,
            map_id=self.road_map.map_id,
            success=True,
            segments=segments,
            points=points,
            total_length_m=sum(segment.length_m for segment in segments),
            target_lane=target_lane,
            metadata={
                "route_policy": request.route_policy,
                "start_edge": edges[0].edge_id,
                "goal_edge": edges[-1].edge_id,
            },
        )


def _build_reference_path(
    edges: Iterable[LaneEdge], *, sample_spacing_m: float = 1.0
) -> Tuple[PathTuple, ...]:
    raw: List[Tuple[float, float]] = []
    for edge in edges:
        for point in edge.centerline:
            if raw and math.hypot(point[0] - raw[-1][0], point[1] - raw[-1][1]) < 1e-6:
                continue
            raw.append(point)
    if len(raw) < 2:
        return ()

    dense: List[Tuple[float, float]] = [raw[0]]
    spacing = max(0.1, float(sample_spacing_m))
    for first, second in zip(raw[:-1], raw[1:]):
        distance = math.hypot(second[0] - first[0], second[1] - first[1])
        steps = max(1, int(math.ceil(distance / spacing)))
        for step in range(1, steps + 1):
            ratio = step / steps
            point = (
                first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio,
            )
            if math.hypot(point[0] - dense[-1][0], point[1] - dense[-1][1]) > 1e-9:
                dense.append(point)
    raw = dense

    result: List[PathTuple] = []
    for index, (x, y) in enumerate(raw):
        if index == 0:
            next_point = raw[1]
            theta = math.atan2(next_point[1] - y, next_point[0] - x)
        elif index == len(raw) - 1:
            previous = raw[index - 1]
            theta = math.atan2(y - previous[1], x - previous[0])
        else:
            previous, next_point = raw[index - 1], raw[index + 1]
            theta = math.atan2(next_point[1] - previous[1], next_point[0] - previous[0])
        if index == 0 or index == len(raw) - 1:
            kappa = 0.0
        else:
            previous = raw[index - 1]
            next_point = raw[index + 1]
            ab = math.hypot(x - previous[0], y - previous[1])
            bc = math.hypot(next_point[0] - x, next_point[1] - y)
            ac = math.hypot(next_point[0] - previous[0], next_point[1] - previous[1])
            denominator = ab * bc * ac
            if denominator <= 1e-9:
                kappa = 0.0
            else:
                cross = (
                    (x - previous[0]) * (next_point[1] - y)
                    - (y - previous[1]) * (next_point[0] - x)
                )
                kappa = 2.0 * cross / denominator
        result.append((x, y, theta, kappa))
    return tuple(result)
