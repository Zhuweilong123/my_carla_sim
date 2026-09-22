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


def _offset_polyline(
    centerline: tuple[Point2D, ...], offset_m: float
) -> tuple[Point2D, ...]:
    """Generate a same-direction lane boundary from a centerline."""

    boundary = []
    for index, point in enumerate(centerline):
        previous = centerline[max(0, index - 1)]
        following = centerline[min(len(centerline) - 1, index + 1)]
        tangent_x = following[0] - previous[0]
        tangent_y = following[1] - previous[1]
        length = math.hypot(tangent_x, tangent_y)
        if length <= 1e-9:
            raise ValueError("lane edge centerline has repeated tangent points")
        normal_x = -tangent_y / length
        normal_y = tangent_x / length
        boundary.append((point[0] + offset_m * normal_x, point[1] + offset_m * normal_y))
    return tuple(boundary)


def _lane_boundary(
    raw: Mapping[str, Any], key: str, centerline: tuple[Point2D, ...], offset_m: float
) -> tuple[Point2D, ...]:
    value = raw.get(key)
    if value is None:
        return _offset_polyline(centerline, offset_m)
    return _centerline(value)


def map_from_dict(data: Mapping[str, Any]) -> RoadMap:
    """Build and validate a :class:`RoadMap` from a JSON-compatible object.

    The compact schema is intentionally explicit at junctions.  Every lane
    edge lists its legal successor edge IDs; lane changes can be represented
    by short connector edges in exactly the same graph.
    """

    map_id = str(data.get("map_id", "unnamed_map"))
    lane_width = float(data.get("lane_width", 3.5))
    if not math.isfinite(lane_width) or lane_width <= 0.0:
        raise ValueError("map lane_width must be finite and positive")
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
        left_boundary = _lane_boundary(
            raw, "left_boundary", centerline, lane_width / 2.0
        )
        right_boundary = _lane_boundary(
            raw, "right_boundary", centerline, -lane_width / 2.0
        )
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
            left_boundary=left_boundary,
            right_boundary=right_boundary,
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
            left_boundary=edge.left_boundary,
            right_boundary=edge.right_boundary,
            speed_limit_kmh=edge.speed_limit_kmh,
            successors=successors,
            maneuver=edge.maneuver,
            bidirectional=edge.bidirectional,
        )

    inferred_num_lanes = max(
        1, max(max(edge.lane_index, 0) for edge in normalized.values()) + 1
    )
    num_lanes = int(data.get("num_lanes", inferred_num_lanes))
    if num_lanes <= 0:
        raise ValueError("map num_lanes must be positive")
    if num_lanes < inferred_num_lanes:
        raise ValueError("map num_lanes cannot exclude a declared lane_index")
    road_map = RoadMap(
        map_id=map_id,
        nodes=nodes,
        edges=normalized,
        lane_width=lane_width,
        num_lanes=num_lanes,
    )
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
        if len(edge.left_boundary) < 2 or len(edge.right_boundary) < 2:
            raise ValueError(f"edge {edge.edge_id} requires two-point lane boundaries")
        if (
            math.dist(edge.left_boundary[0], edge.right_boundary[0]) <= 1e-6
            or math.dist(edge.left_boundary[-1], edge.right_boundary[-1]) <= 1e-6
        ):
            raise ValueError(f"edge {edge.edge_id} has collapsed lane boundaries")


def load_map(path: str | Path) -> RoadMap:
    """Load a UTF-8 JSON road map and validate it before use."""

    map_path = Path(path)
    with map_path.open("r", encoding="utf-8") as stream:
        return map_from_dict(json.load(stream))
