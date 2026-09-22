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
        max_curvature_1pm: float = 0.15,
        corner_angle_threshold_rad: float = 0.7,
    ) -> None:
        if sample_spacing_m <= 0.0:
            raise ValueError("sample spacing must be positive")
        if max_lateral_deviation_m < 0.0:
            raise ValueError("max lateral deviation must be non-negative")
        if boundary_margin_m < 0.0:
            raise ValueError("boundary margin must be non-negative")
        if max_curvature_1pm <= 0.0:
            raise ValueError("maximum curvature must be positive")
        if corner_angle_threshold_rad <= 0.0:
            raise ValueError("corner angle threshold must be positive")
        self.sample_spacing_m = float(sample_spacing_m)
        self.max_lateral_deviation_m = float(max_lateral_deviation_m)
        self.join_tolerance_m = float(join_tolerance_m)
        self.boundary_margin_m = float(boundary_margin_m)
        self.max_curvature_1pm = float(max_curvature_1pm)
        self.corner_angle_threshold_rad = float(corner_angle_threshold_rad)
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
            (
                samples,
                left_boundary,
                right_boundary,
                drivable_left_boundary,
                drivable_right_boundary,
            ) = _stitch_lane_geometry(
                edges, road_map, self.sample_spacing_m, self.join_tolerance_m
            )
            raw_route = _raw_route_geometry(edges)
            rounded = _round_polyline_corners(
                raw_route,
                max_curvature_1pm=self.max_curvature_1pm,
                angle_threshold_rad=self.corner_angle_threshold_rad,
                spacing_m=self.sample_spacing_m,
            )
            rounded = _resample_polyline(rounded, self.sample_spacing_m)
            lane_index = max(0, int(edges[0].lane_index))
            left_boundary = _offset_polyline(rounded, road_map.lane_width / 2.0)
            right_boundary = _offset_polyline(rounded, -road_map.lane_width / 2.0)
            drivable_left_boundary = _offset_polyline(
                rounded,
                (road_map.num_lanes - lane_index - 0.5) * road_map.lane_width,
            )
            drivable_right_boundary = _offset_polyline(
                rounded,
                -(lane_index + 0.5) * road_map.lane_width,
            )
            smoothed = _constrained_smooth(
                rounded,
                left_boundary,
                right_boundary,
                max_deviation_m=self.max_lateral_deviation_m,
                boundary_margin_m=self.boundary_margin_m,
            )
            left_boundary = _offset_polyline(smoothed, road_map.lane_width / 2.0)
            right_boundary = _offset_polyline(smoothed, -road_map.lane_width / 2.0)
            drivable_left_boundary = _offset_polyline(
                smoothed,
                (road_map.num_lanes - lane_index - 0.5) * road_map.lane_width,
            )
            drivable_right_boundary = _offset_polyline(
                smoothed,
                -(lane_index + 0.5) * road_map.lane_width,
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
            drivable_left_boundary=_with_geometry(drivable_left_boundary),
            drivable_right_boundary=_with_geometry(drivable_right_boundary),
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


def _raw_route_geometry(edges: Sequence[LaneEdge]) -> List[Point2D]:
    points: List[Point2D] = []
    for edge in edges:
        for point in edge.centerline:
            if not points or math.dist(points[-1], point) > 1e-9:
                points.append(point)
    return points


def _stitch_lane_geometry(
    edges: Sequence[LaneEdge],
    road_map: RoadMap,
    spacing_m: float,
    join_tolerance_m: float,
) -> Tuple[List[Point2D], List[Point2D], List[Point2D], List[Point2D], List[Point2D]]:
    """Sample and stitch lane and full-road geometry with common indices."""

    stitched: List[Point2D] = []
    left_stitched: List[Point2D] = []
    right_stitched: List[Point2D] = []
    drivable_left_stitched: List[Point2D] = []
    drivable_right_stitched: List[Point2D] = []
    for edge in edges:
        steps = max(1, int(math.ceil(edge.length / spacing_m)))
        centerline = _resample_polyline_to_count(edge.centerline, steps + 1)
        left_boundary = _resample_polyline_to_count(edge.left_boundary, steps + 1)
        right_boundary = _resample_polyline_to_count(edge.right_boundary, steps + 1)
        road_left, road_right = _drivable_boundaries_for_edge(edge, road_map)
        drivable_left = _resample_polyline_to_count(road_left, steps + 1)
        drivable_right = _resample_polyline_to_count(road_right, steps + 1)
        if stitched:
            gap = math.dist(stitched[-1], centerline[0])
            if gap > join_tolerance_m:
                raise ValueError(
                    f"route geometry is discontinuous at {edge.edge_id}: gap={gap:.3f}m"
                )
            left_stitched[-1] = _midpoint(left_stitched[-1], left_boundary[0])
            right_stitched[-1] = _midpoint(right_stitched[-1], right_boundary[0])
            drivable_left_stitched[-1] = _midpoint(
                drivable_left_stitched[-1], drivable_left[0]
            )
            drivable_right_stitched[-1] = _midpoint(
                drivable_right_stitched[-1], drivable_right[0]
            )
        for point, left, right, road_left, road_right in zip(
            centerline,
            left_boundary,
            right_boundary,
            drivable_left,
            drivable_right,
        ):
            if stitched and math.dist(stitched[-1], point) <= 1e-9:
                continue
            stitched.append(point)
            left_stitched.append(left)
            right_stitched.append(right)
            drivable_left_stitched.append(road_left)
            drivable_right_stitched.append(road_right)
    if len(stitched) < 2:
        raise ValueError("reference line requires at least two distinct points")
    return (
        stitched,
        left_stitched,
        right_stitched,
        drivable_left_stitched,
        drivable_right_stitched,
    )


def _drivable_boundaries_for_edge(
    edge: LaneEdge, road_map: RoadMap
) -> Tuple[Sequence[Point2D], Sequence[Point2D]]:
    """Return the outer road edges for parallel lanes on this map segment."""

    parallel_lanes = sorted(
        (
            candidate
            for candidate in road_map.edges.values()
            if candidate.road_id == edge.road_id
            and candidate.from_node == edge.from_node
            and candidate.to_node == edge.to_node
            and candidate.lane_index >= 0
        ),
        key=lambda candidate: candidate.lane_index,
    )
    if not parallel_lanes:
        return edge.left_boundary, edge.right_boundary
    return parallel_lanes[-1].left_boundary, parallel_lanes[0].right_boundary


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


def _round_polyline_corners(
    points: Sequence[Point2D],
    *,
    max_curvature_1pm: float = 0.15,
    angle_threshold_rad: float = 0.25,
    spacing_m: float = 1.0,
) -> List[Point2D]:
    """Replace sharp topological corners with tangent circular transitions."""

    source = []
    for point in points:
        current = (float(point[0]), float(point[1]))
        if len(source) >= 2:
            first, second = source[-2], source[-1]
            incoming = (second[0] - first[0], second[1] - first[1])
            outgoing = (current[0] - second[0], current[1] - second[1])
            cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
            dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
            if abs(cross) <= 1e-6 and dot > 0.0:
                source[-1] = current
                continue
        source.append(current)
    if len(source) < 3:
        return source
    radius = 1.0 / max(float(max_curvature_1pm), 1e-6)
    rounded = [source[0]]
    previous_corner_index = 0
    previous_exit = source[0]
    for index in range(1, len(source) - 1):
        previous, corner, following = source[index - 1:index + 2]
        incoming_length = math.dist(previous, corner)
        outgoing_length = math.dist(corner, following)
        if incoming_length <= 1e-9 or outgoing_length <= 1e-9:
            continue
        incoming = ((corner[0] - previous[0]) / incoming_length,
                    (corner[1] - previous[1]) / incoming_length)
        outgoing = ((following[0] - corner[0]) / outgoing_length,
                    (following[1] - corner[1]) / outgoing_length)
        dot = max(-1.0, min(1.0, incoming[0] * outgoing[0] +
                             incoming[1] * outgoing[1]))
        angle = math.acos(dot)
        cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
        if angle < angle_threshold_rad or abs(cross) <= 1e-9:
            continue
        tangent = min(radius * math.tan(angle / 2.0),
                      0.45 * incoming_length, 0.45 * outgoing_length)
        if tangent <= 1e-6:
            continue
        actual_radius = tangent / max(math.tan(angle / 2.0), 1e-9)
        start = (corner[0] - incoming[0] * tangent,
                 corner[1] - incoming[1] * tangent)
        end = (corner[0] + outgoing[0] * tangent,
               corner[1] + outgoing[1] * tangent)
        sign = 1.0 if cross > 0.0 else -1.0
        normal = (-incoming[1], incoming[0])
        center = (start[0] + sign * actual_radius * normal[0],
                  start[1] + sign * actual_radius * normal[1])
        start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
        end_angle = math.atan2(end[1] - center[1], end[0] - center[0])
        delta = end_angle - start_angle
        if sign > 0.0:
            while delta < 0.0:
                delta += 2.0 * math.pi
        else:
            while delta > 0.0:
                delta -= 2.0 * math.pi
        steps = max(2, int(math.ceil(abs(delta) * actual_radius / spacing_m)))
        if previous_corner_index == 0:
            rounded.extend(source[1:index])
        else:
            _append_polyline_line(
                rounded, previous_exit, source[previous_corner_index + 1], spacing_m
            )
            rounded.extend(source[previous_corner_index + 2:index])
        _append_polyline_line(rounded, rounded[-1], start, spacing_m)
        for step in range(1, steps + 1):
            current = start_angle + delta * step / steps
            rounded.append((center[0] + actual_radius * math.cos(current),
                            center[1] + actual_radius * math.sin(current)))
        previous_exit = end
        previous_corner_index = index
    if previous_corner_index == 0:
        rounded.extend(source[1:])
    else:
        _append_polyline_line(
            rounded, previous_exit, source[previous_corner_index + 1], spacing_m
        )
        rounded.extend(source[previous_corner_index + 2:])
    return rounded


def _append_polyline_line(output, start, end, spacing_m):
    length = math.dist(start, end)
    steps = max(1, int(math.ceil(length / max(spacing_m, 1e-6))))
    for step in range(1, steps + 1):
        ratio = step / steps
        point = (start[0] + (end[0] - start[0]) * ratio,
                 start[1] + (end[1] - start[1]) * ratio)
        if math.dist(output[-1], point) > 1e-9:
            output.append(point)


def _offset_polyline(points: Sequence[Point2D], offset_m: float) -> List[Point2D]:
    """Offset a centerline with its local tangent normal."""

    if len(points) < 2:
        raise ValueError("offset requires at least two points")
    offset = []
    for index, point in enumerate(points):
        previous = points[max(0, index - 1)]
        following = points[min(len(points) - 1, index + 1)]
        tangent_x = following[0] - previous[0]
        tangent_y = following[1] - previous[1]
        length = math.hypot(tangent_x, tangent_y)
        if length <= 1e-9:
            raise ValueError("cannot offset a zero-length reference segment")
        normal_x = -tangent_y / length
        normal_y = tangent_x / length
        offset.append((point[0] + offset_m * normal_x,
                       point[1] + offset_m * normal_y))
    return offset
