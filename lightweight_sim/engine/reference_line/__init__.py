"""Map-derived reference-line generation for topology routing outputs."""

from .core import ReferenceLineCore
from .models import ReferenceLinePlan
from .route_aware_planner import RouteAwareMotionPlanner, BaselinePathPlanner

__all__ = ["ReferenceLineCore", "ReferenceLinePlan", "RouteAwareMotionPlanner", "BaselinePathPlanner", "DPQPPathPlanner", "create_local_planner"]


def __getattr__(name):
    # Geometry, simulator and GUI imports must not load the QP solver.
    if name in {"DPQPPathPlanner", "create_local_planner"}:
        from . import dp_qp_planner
        return getattr(dp_qp_planner, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
