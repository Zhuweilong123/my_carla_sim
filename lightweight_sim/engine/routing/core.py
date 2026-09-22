"""Public, transport-independent Routing service."""

from __future__ import annotations

import itertools
import threading
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional

from .astar import AStarRouter
from .map_loader import load_map
from .models import RoadMap, RoutePlan, RouteRequest


class RoutingCore:
    """Manage map versions and provide deterministic A* route requests."""

    def __init__(self, maps: Optional[Iterable[RoadMap]] = None):
        self._maps: Dict[str, RoadMap] = {}
        self._routers: Dict[str, AStarRouter] = {}
        self._route_ids = itertools.count(1)
        self._request_ids = itertools.count(1)
        self._lock = threading.RLock()
        for road_map in maps or ():
            self.register_map(road_map)

    @classmethod
    def from_paths(cls, paths: Iterable[str | Path]) -> "RoutingCore":
        return cls(load_map(path) for path in paths)

    def register_map(self, road_map: RoadMap) -> None:
        with self._lock:
            self._maps[road_map.map_id] = road_map
            self._routers[road_map.map_id] = AStarRouter(road_map)

    def map_ids(self):
        with self._lock:
            return tuple(sorted(self._maps))

    def route(self, request: RouteRequest, request_id: Optional[int] = None) -> RoutePlan:
        with self._lock:
            request_id = int(request_id) if request_id and request_id > 0 else next(self._request_ids)
            route_id = next(self._route_ids)
            router = self._routers.get(request.map_id)
            if router is None:
                return RoutePlan.failure(
                    route_id,
                    request_id,
                    request.map_id,
                    f"unknown map: {request.map_id}",
                )
            return router.search(request, route_id=route_id, request_id=request_id)

    def route_from_values(
        self,
        map_id: str,
        start_x: float,
        start_y: float,
        start_yaw: float,
        goal_x: float,
        goal_y: float,
        goal_yaw: float = 0.0,
        *,
        start_lane: int = -1,
        goal_lane: int = -1,
        route_policy: str = "fastest",
        allow_u_turn: bool = False,
    ) -> RoutePlan:
        from .models import Pose2D

        return self.route(
            RouteRequest(
                map_id=map_id,
                start=Pose2D(start_x, start_y, start_yaw),
                goal=Pose2D(goal_x, goal_y, goal_yaw),
                start_lane=start_lane,
                goal_lane=goal_lane,
                route_policy=route_policy,
                allow_u_turn=allow_u_turn,
            )
        )
