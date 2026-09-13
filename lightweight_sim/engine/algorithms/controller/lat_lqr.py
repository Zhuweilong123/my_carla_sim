"""Tuned public entry point for the projected lateral controller."""

from ._lat_lqr_opt2 import LateralLQRController as _ProjectedLateralLQRController


class LateralLQRController(_ProjectedLateralLQRController):
    """Use the third-round gains selected for the figure-eight benchmark."""

    def __init__(self, vehicle_para, Q=None, R=1.0, ts=0.05):
        super().__init__(vehicle_para, Q=Q, R=R, ts=ts)
        self.k_lat = 20.0
        self.k_heading = 0.5
