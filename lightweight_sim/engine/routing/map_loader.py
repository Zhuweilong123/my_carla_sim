"""Road-map loading and structural validation for the routing core."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence

from .models import LaneEdge, MapNode, Point2D, RoadMap


def _point(value: Sequence[float]) -> Point2D:
    if len(value) < 2:
        raise ValueError("centerline point must contain x and y")
    x, y = float(value[0]), float(value[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("centerline coordinates must be finite")
    return x, y


def _centerline(value: Iterable[Sequence[float]]) -> tuple[Point2D, ...]:
    points = tuple(_point(point) for point in value)
    if len(points) < 2:
        raise ValueError("lane edge centerline needs at least two points")
    if all(first == second for first, second in zip(points[:-1], points[1:])):
        raise ValueError("lane edge centerline must have nonzero length")
    return points


def map_from_dict(data: Mapping[str, Any]) -> RoadMap:
    """Build and validate a :class:`RoadMap` from a JSON-compatible object.

    The compact schema is intentionally explicit at junctions.  Every lane
    edge lists its legal successor edge IDs; lane changes can be represented
    by short connector edges in exactly the same graph.
    """

    map_id = str(data.get("map_id", "unnamed_map"))
    raw_nodes = data.get("nodes", [])
    raw_edges = data.get("edges", data.get("lanes", []))
    nodes: Dict[str, MapNode] = {}
    for raw in raw_nodes:
        node_id = str(raw["id"])
        if node_id in nodes:
            raise ValueError(f"duplicate map node: {node_id}")
        nodes[node_id] = MapNode(node_id, float(raw["x"]), float(raw["y"]))

    edges: Dict[str, LaneEdge] = {}
    for raw in raw_edges:
        edge_id = str(raw["id"])
        if edge_id in edges:
            raise ValueError(f"duplicate lane edge: {edge_id}")
        centerline = _centerline(raw["centerline"])
        speed = float(raw.get("speed_limit_kmh", 40.0))
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(f"invalid speed limit for edge {edge_id}")
        edge = LaneEdge(
            edge_id=edge_id,
            road_id=str(raw.get("road_id", edge_id)),
            lane_id=str(raw.get("lane_id", edge_id)),
            from_node=str(raw["from"]),
            to_node=str(raw["to"]),
            lane_index=int(raw.get("lane_index", 0)),
            centerline=centerline,
            speed_limit_kmh=speed,
            successors=tuple(str(item) for item in raw.get("successors", [])),
            maneuver=str(raw.get("maneuver", "straight")),
            bidirectional=bool(raw.get("bidirectional", False)),
        )
        if edge.length <= 1e-6:
            raise ValueError(f"lane edge has zero length: {edge_id}")
        edges[edge_id] = edge

    if not nodes:
        raise ValueError("road map must contain at least one node")
    if not edges:
        raise ValueError("road map must contain at least one lane edge")

    # Fill omitted successors from the node topology.  Production maps should
    # normally be explicit; this fallback keeps simple hand-authored maps
    # usable while still validating every referenced successor.
    outgoing: Dict[str, list[str]] = {}
    for edge in edges.values():
        outgoing.setdefault(edge.from_node, []).append(edge.edge_id)
    normalized: Dict[str, LaneEdge] = {}
    for edge in edges.values():
        successors = edge.successors
        if not successors:
            successors = tuple(
                candidate.edge_id
                for candidate in edges.values()
                if candidate.from_node == edge.to_node
            )
        normalized[edge.edge_id] = LaneEdge(
            edge_id=edge.edge_id,
            road_id=edge.road_id,
            lane_id=edge.lane_id,
            from_node=edge.from_node,
            to_node=edge.to_node,
            lane_index=edge.lane_index,
            centerline=edge.centerline,
            speed_limit_kmh=edge.speed_limit_kmh,
            successors=successors,
            maneuver=edge.maneuver,
            bidirectional=edge.bidirectional,
        )

    road_map = RoadMap(map_id=map_id, nodes=nodes, edges=normalized)
    validate_map(road_map)
    return road_map


def validate_map(road_map: RoadMap) -> None:
    """Raise ``ValueError`` for malformed topology or geometry."""

    for edge in road_map.edges.values():
        if edge.from_node not in road_map.nodes or edge.to_node not in road_map.nodes:
            raise ValueError(
                f"edge {edge.edge_id} references an unknown endpoint "
                f"({edge.from_node}->{edge.to_node})"
            )
        for successor in edge.successors:
            if successor not in road_map.edges:
                raise ValueError(f"edge {edge.edge_id} references unknown successor {successor}")
            if road_map.edges[successor].from_node != edge.to_node:
                raise ValueError(
                    f"edge {edge.edge_id} successor {successor} does not start at "
                    f"{edge.to_node}"
                )


def load_map(path: str | Path) -> RoadMap:
    """Load a UTF-8 JSON road map and validate it before use."""

    map_path = Path(path)
    with map_path.open("r", encoding="utf-8") as stream:
        return map_from_dict(json.load(stream))
