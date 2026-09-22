"""Data model for map-derived, planner-ready reference lines."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from ..routing.models import PathTuple, RouteSegment


@dataclass(frozen=True)
class ReferenceLinePlan:
    """A continuous geometric reference derived from one topology route."""

    reference_id: int
    route_id: int
    request_id: int
    map_id: str
    success: bool
    failure_reason: str = ""
    total_length_m: float = 0.0
    sample_spacing_m: float = 1.0
    lane_width: float = 3.5
    num_lanes: int = 1
    reference_lane_index: int = -1
    target_lane: int = -1
    segments: Tuple[RouteSegment, ...] = ()
    points: Tuple[PathTuple, ...] = ()
    left_boundary: Tuple[PathTuple, ...] = ()
    right_boundary: Tuple[PathTuple, ...] = ()

    @classmethod
    def failure(
        cls,
        reference_id: int,
        route_id: int,
        request_id: int,
        map_id: str,
        reason: str,
    ) -> "ReferenceLinePlan":
        return cls(
            reference_id=reference_id,
            route_id=route_id,
            request_id=request_id,
            map_id=map_id,
            success=False,
            failure_reason=reason,
        )
