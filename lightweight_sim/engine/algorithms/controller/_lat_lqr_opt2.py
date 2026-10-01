"""Compatibility entry for the projected dynamic Riccati LQR."""

from ._lat_lqr_impl import LateralLQRController as _DynamicLateralLQRController
from .reference_tracking import ProjectedLateralController


class LateralLQRController(_DynamicLateralLQRController):
    control = ProjectedLateralController.control
