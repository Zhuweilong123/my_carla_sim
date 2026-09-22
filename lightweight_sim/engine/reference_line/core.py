"""Build continuous, bounded-deviation reference lines from routing topology."""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Sequence, Tuple

from ..algorithms.utils.geometry import cal_heading_kappa
from ..routing.models import LaneEdge, RoadMap, RoutePlan
from .models import ReferenceLinePlan


Point2D = Tuple[float, float]


class ReferenceLineCore:
    """Translate ordered routing edges into a planner-ready lane reference."""

    def __init__(
        self,
        maps: Iterable[RoadMap] = (),
        *,
        sample_spacing_m: float = 1.0,
        max_lateral_deviation_m: float = 0.15,
        join_tolerance_m: float = 0.25,
    ) -> None:
        if sample_spacing_m <= 0.0:
            raise ValueError("sample spacing must be positive")
        if max_lateral_deviation_m < 0.0:
            raise ValueError("max lateral deviation must be non-negative")
        self.sample_spacing_m = float(sample_spacing_m)
        self.max_lateral_deviation_m = float(max_lateral_deviation_m)
        self.join_tolerance_m = float(join_tolerance_m)
        self._maps: Dict[str, RoadMap] = {}
        for road_map in maps:
            self.register_map(road_map)

    def register_map(self, road_map: RoadMap) -> None:
        self._maps[road_map.map_id] = road_map

    def map_ids(self) -> Tuple[str, ...]:
        return tuple(sorted(self._maps))

    def build(self, route: RoutePlan, *, reference_id: int) -> ReferenceLinePlan:
        if not route.success:
            return ReferenceLinePlan.failure(
                reference_id,
                route.route_id,
                route.request_id,
                route.map_id,
                f"routing failed: {route.failure_reason}",
            )
        road_map = self._maps.get(route.map_id)
        if road_map is None:
            return ReferenceLinePlan.failure(
                reference_id,
                route.route_id,
                route.request_id,
                route.map_id,
                f"unknown map: {route.map_id}",
            )
        try:
            edges = self._resolve_edges(route, road_map)
            raw = _stitch_centerlines(edges, self.join_tolerance_m)
        except ValueError as exc:
            return ReferenceLinePlan.failure(
                reference_id,
                route.route_id,
                route.request_id,
                route.map_id,
                str(exc),
            )

        samples = _resample_polyline(raw, self.sample_spacing_m)
        smoothed = _constrained_smooth(
            samples, max_deviation_m=self.max_lateral_deviation_m
        )
        points = _with_geometry(smoothed)
        return ReferenceLinePlan(
            reference_id=reference_id,
            route_id=route.route_id,
            request_id=route.request_id,
            map_id=route.map_id,
            success=True,
            total_length_m=_polyline_length(smoothed),
            sample_spacing_m=self.sample_spacing_m,
            reference_lane_index=edges[0].lane_index,
            target_lane=route.target_lane,
            segments=route.segments,
            points=points,
        )

    @staticmethod
    def _resolve_edges(route: RoutePlan, road_map: RoadMap) -> List[LaneEdge]:
        if not route.segments:
            raise ValueError("routing route contains no segments")
        edges: List[LaneEdge] = []
        for segment in route.segments:
            edge = road_map.edges.get(segment.edge_id)
            if edge is None:
                raise ValueError(f"route references unknown edge: {segment.edge_id}")
            if edge.lane_id != segment.lane_id or edge.lane_index != segment.lane_index:
                raise ValueError(f"route segment metadata mismatch: {segment.edge_id}")
            edges.append(edge)
        for previous, current in zip(edges[:-1], edges[1:]):
            if current.edge_id not in previous.successors:
                raise ValueError(
                    f"route edges are not topologically connected: "
                    f"{previous.edge_id}->{current.edge_id}"
                )
        return edges


def _stitch_centerlines(
    edges: Sequence[LaneEdge], join_tolerance_m: float
) -> List[Point2D]:
    stitched: List[Point2D] = []
    for edge in edges:
        if stitched:
            gap = math.dist(stitched[-1], edge.centerline[0])
            if gap > join_tolerance_m:
                raise ValueError(
                    f"route geometry is discontinuous at {edge.edge_id}: gap={gap:.3f}m"
                )
        for point in edge.centerline:
            if stitched and math.dist(stitched[-1], point) <= 1e-9:
                continue
            stitched.append(point)
    if len(stitched) < 2:
        raise ValueError("reference line requires at least two distinct points")
    return stitched


def _resample_polyline(points: Sequence[Point2D], spacing_m: float) -> List[Point2D]:
    sampled: List[Point2D] = [points[0]]
    for first, second in zip(points[:-1], points[1:]):
        length = math.dist(first, second)
        steps = max(1, int(math.ceil(length / spacing_m)))
        for step in range(1, steps + 1):
            ratio = step / steps
            point = (
                first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio,
            )
            if math.dist(sampled[-1], point) > 1e-9:
                sampled.append(point)
    return sampled


def _constrained_smooth(
    points: Sequence[Point2D], *, max_deviation_m: float
) -> List[Point2D]:
    """Smooth locally while never moving a sample beyond its map position cap."""

    if len(points) < 3 or max_deviation_m <= 0.0:
        return list(points)
    original = list(points)
    result = list(points)
    for _ in range(2):
        next_result = list(result)
        for index in range(1, len(result) - 1):
            previous, current, following = result[index - 1:index + 2]
            candidate = (
                (previous[0] + 2.0 * current[0] + following[0]) / 4.0,
                (previous[1] + 2.0 * current[1] + following[1]) / 4.0,
            )
            delta_x = candidate[0] - original[index][0]
            delta_y = candidate[1] - original[index][1]
            delta = math.hypot(delta_x, delta_y)
            if delta > max_deviation_m:
                scale = max_deviation_m / delta
                candidate = (
                    original[index][0] + delta_x * scale,
                    original[index][1] + delta_y * scale,
                )
            next_result[index] = candidate
        result = next_result
    return result


def _with_geometry(points: Sequence[Point2D]) -> Tuple[Tuple[float, float, float, float], ...]:
    headings, curvatures = cal_heading_kappa(points)
    return tuple(
        (float(point[0]), float(point[1]), float(headings[index]), float(curvatures[index]))
        for index, point in enumerate(points)
    )


def _polyline_length(points: Sequence[Point2D]) -> float:
    return sum(math.dist(first, second) for first, second in zip(points[:-1], points[1:]))
