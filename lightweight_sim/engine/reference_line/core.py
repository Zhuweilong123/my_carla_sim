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
        boundary_margin_m: float = 0.1,
    ) -> None:
        if sample_spacing_m <= 0.0:
            raise ValueError("sample spacing must be positive")
        if max_lateral_deviation_m < 0.0:
            raise ValueError("max lateral deviation must be non-negative")
        if boundary_margin_m < 0.0:
            raise ValueError("boundary margin must be non-negative")
        self.sample_spacing_m = float(sample_spacing_m)
        self.max_lateral_deviation_m = float(max_lateral_deviation_m)
        self.join_tolerance_m = float(join_tolerance_m)
        self.boundary_margin_m = float(boundary_margin_m)
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
            samples, left_boundary, right_boundary = _stitch_lane_geometry(
                edges, self.sample_spacing_m, self.join_tolerance_m
            )
            smoothed = _constrained_smooth(
                samples,
                left_boundary,
                right_boundary,
                max_deviation_m=self.max_lateral_deviation_m,
                boundary_margin_m=self.boundary_margin_m,
            )
        except ValueError as exc:
            return ReferenceLinePlan.failure(
                reference_id,
                route.route_id,
                route.request_id,
                route.map_id,
                str(exc),
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
            lane_width=road_map.lane_width,
            num_lanes=road_map.num_lanes,
            reference_lane_index=edges[0].lane_index,
            target_lane=route.target_lane,
            segments=route.segments,
            points=points,
            left_boundary=_with_geometry(left_boundary),
            right_boundary=_with_geometry(right_boundary),
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


def _stitch_lane_geometry(
    edges: Sequence[LaneEdge], spacing_m: float, join_tolerance_m: float
) -> Tuple[List[Point2D], List[Point2D], List[Point2D]]:
    """Sample and stitch center and lane boundaries with common indices."""

    stitched: List[Point2D] = []
    left_stitched: List[Point2D] = []
    right_stitched: List[Point2D] = []
    for edge in edges:
        steps = max(1, int(math.ceil(edge.length / spacing_m)))
        centerline = _resample_polyline_to_count(edge.centerline, steps + 1)
        left_boundary = _resample_polyline_to_count(edge.left_boundary, steps + 1)
        right_boundary = _resample_polyline_to_count(edge.right_boundary, steps + 1)
        if stitched:
            gap = math.dist(stitched[-1], centerline[0])
            if gap > join_tolerance_m:
                raise ValueError(
                    f"route geometry is discontinuous at {edge.edge_id}: gap={gap:.3f}m"
                )
            left_stitched[-1] = _midpoint(left_stitched[-1], left_boundary[0])
            right_stitched[-1] = _midpoint(right_stitched[-1], right_boundary[0])
        for point, left, right in zip(centerline, left_boundary, right_boundary):
            if stitched and math.dist(stitched[-1], point) <= 1e-9:
                continue
            stitched.append(point)
            left_stitched.append(left)
            right_stitched.append(right)
    if len(stitched) < 2:
        raise ValueError("reference line requires at least two distinct points")
    return stitched, left_stitched, right_stitched


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


def _resample_polyline_to_count(
    points: Sequence[Point2D], count: int
) -> List[Point2D]:
    if count < 2:
        raise ValueError("polyline sample count must be at least two")
    lengths = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        lengths.append(lengths[-1] + math.dist(first, second))
    if lengths[-1] <= 1e-9:
        raise ValueError("lane boundary must have nonzero length")

    sampled = []
    segment_index = 0
    for sample_index in range(count):
        target = lengths[-1] * sample_index / (count - 1)
        while segment_index < len(lengths) - 2 and lengths[segment_index + 1] < target:
            segment_index += 1
        start_length, end_length = lengths[segment_index:segment_index + 2]
        ratio = (target - start_length) / max(end_length - start_length, 1e-9)
        first, second = points[segment_index:segment_index + 2]
        sampled.append(
            (
                first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio,
            )
        )
    return sampled


def _constrained_smooth(
    points: Sequence[Point2D],
    left_boundary: Sequence[Point2D],
    right_boundary: Sequence[Point2D],
    *,
    max_deviation_m: float,
    boundary_margin_m: float,
) -> List[Point2D]:
    """Smooth locally while respecting both deviation and map-corridor caps."""

    if len(points) < 3 or max_deviation_m <= 0.0:
        return list(points)
    if len(points) != len(left_boundary) or len(points) != len(right_boundary):
        raise ValueError("reference line and lane boundaries must share sample indices")
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
            next_result[index] = _clamp_to_lane_corridor(
                candidate,
                original[index],
                left_boundary[index],
                right_boundary[index],
                boundary_margin_m,
            )
        result = next_result
    return result


def _clamp_to_lane_corridor(
    candidate: Point2D,
    center: Point2D,
    left: Point2D,
    right: Point2D,
    margin_m: float,
) -> Point2D:
    """Clamp only lateral displacement, keeping longitudinal smoothing intact."""

    normal_x = left[0] - right[0]
    normal_y = left[1] - right[1]
    normal_length = math.hypot(normal_x, normal_y)
    if normal_length <= 1e-9:
        raise ValueError("lane boundaries collapse at a reference sample")
    normal_x /= normal_length
    normal_y /= normal_length
    left_offset = (left[0] - center[0]) * normal_x + (left[1] - center[1]) * normal_y
    right_offset = (right[0] - center[0]) * normal_x + (right[1] - center[1]) * normal_y
    lower = min(left_offset, right_offset) + margin_m
    upper = max(left_offset, right_offset) - margin_m
    if lower > upper:
        raise ValueError("lane corridor is narrower than twice the boundary margin")
    offset_x = candidate[0] - center[0]
    offset_y = candidate[1] - center[1]
    lateral = offset_x * normal_x + offset_y * normal_y
    clamped_lateral = min(max(lateral, lower), upper)
    return (
        candidate[0] + (clamped_lateral - lateral) * normal_x,
        candidate[1] + (clamped_lateral - lateral) * normal_y,
    )


def _midpoint(first: Point2D, second: Point2D) -> Point2D:
    return ((first[0] + second[0]) / 2.0, (first[1] + second[1]) / 2.0)


def _with_geometry(points: Sequence[Point2D]) -> Tuple[Tuple[float, float, float, float], ...]:
    headings, curvatures = cal_heading_kappa(points)
    return tuple(
        (float(point[0]), float(point[1]), float(headings[index]), float(curvatures[index]))
        for index, point in enumerate(points)
    )


def _polyline_length(points: Sequence[Point2D]) -> float:
    return sum(math.dist(first, second) for first, second in zip(points[:-1], points[1:]))
