"""Planner selection kept separate from the ROS adapter for fair comparison."""

from __future__ import annotations

from ..core.types import ParkingConfig
from .hybrid_astar import HybridAStarPlanner
from .reverse_parking import ReverseParkingPlanner


def create_planner(name: str, parking_config: ParkingConfig | None = None):
    """Create one of the available planners using the shared trajectory API."""

    normalized = str(name).strip().lower().replace("-", "_")
    if normalized in {"baseline", "dubins", "reverse_parking"}:
        return ReverseParkingPlanner(parking_config)
    if normalized in {"hybrid_astar", "hybrid_a_star"}:
        return HybridAStarPlanner(parking_config)
    raise ValueError(
        f"unknown parking planner {name!r}; choose baseline or hybrid_astar"
    )


__all__ = ["create_planner"]
