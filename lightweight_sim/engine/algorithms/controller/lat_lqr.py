"""Tuned lateral controller boundary for closed-loop road tracking."""

from ._lat_lqr_impl import *  # noqa: F401,F403


_LegacyLateralLQRController = LateralLQRController


class LateralLQRController(_LegacyLateralLQRController):
    """Use a longer preview and stronger feedback for curved roads."""

    def __init__(self, vehicle_para, Q=None, R=1.0, ts=0.1):
        super().__init__(vehicle_para, Q=Q, R=R, ts=ts)
        self.k_lat = 12.0
        self.k_heading = 0.8
