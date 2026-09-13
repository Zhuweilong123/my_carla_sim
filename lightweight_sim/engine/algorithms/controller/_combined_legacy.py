"""Unified lateral/longitudinal vehicle controller."""
from .lat_lqr import LateralLQRController
from .lat_mpc import LateralMPCController
from .lon_pid import LongitudinalPIDController
class VehicleController:
    def __init__(self,vehicle_para,controller_type='LQR_controller',target_speed_kmh=50.0):
        self.controller_type=controller_type; self.vehicle_para=vehicle_para; self.lat=LateralMPCController(vehicle_para) if controller_type=='MPC_controller' else LateralLQRController(vehicle_para); self.lon=LongitudinalPIDController(); self.lon.set_target(target_speed_kmh); self.ref_path=[]
    def update_ref_path(self,ref_path): self.ref_path=ref_path; self.lat.min_index=0
    def set_target_speed(self,speed_kmh): self.lon.set_target(speed_kmh)
    def step(self,x,y,phi,vx,vy,r):
        if not self.ref_path: return 0.0,0.0,0.0
        steer=self.lat.control(x,y,phi,vx,vy,r,self.ref_path); accel=self.lon.control((vx*vx+vy*vy)**0.5); return steer,(accel/3.0 if accel>=0 else 0.0),(abs(accel)/6.0 if accel<0 else 0.0)