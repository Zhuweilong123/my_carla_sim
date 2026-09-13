"""Dependency-free MPC-compatible lateral controller.
The interface is kept stable; the short-horizon preview uses the same bounded
kinematic error feedback until a numerical QP backend is configured.
"""
from .lat_lqr import LateralLQRController
class LateralMPCController(LateralLQRController):
    def __init__(self,vehicle_para,Q=None,F=None,R=1.0,N=6,P=2,ts=0.05): super().__init__(vehicle_para,Q,R,ts)