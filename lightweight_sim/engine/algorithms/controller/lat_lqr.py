"""Public entry point for the projected dynamic Riccati LQR controller."""

from ._lat_lqr_opt2 import LateralLQRController as _ProjectedLateralLQRController


class LateralLQRController(_ProjectedLateralLQRController):
    """Stable public name for the projected dynamic LQR implementation."""

    pass
