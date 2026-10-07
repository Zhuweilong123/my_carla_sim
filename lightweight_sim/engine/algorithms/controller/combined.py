"""Unified lateral/longitudinal vehicle controller."""

from .lat_lqr import LateralLQRController
from .lat_mpc import LateralMPCController
from .lon_pid import LongitudinalPIDController
from .base import LateralController, LongitudinalController
from ...simulator.data_types import VehicleParams
from ...simulator.steering import SteeringParams
from ...runtime_config import DEFAULT_RUNTIME_CONFIG
import math
import numpy as np


class VehicleController:
    def __init__(self, vehicle_para=None, controller_type="LQR_controller", target_speed_kmh=50.0,
                 *, vehicle_params=None, steering_params=None, dt=0.05,
                 actuator_compensation=True, lateral_q=None, lateral_r=100.0,
                 dynamic_lateral_r=300.0, feedback_horizon_s=0.0,
                 smooth_reference_heading=True, discretization="plant",
                 longitudinal_params=None, lateral_controller=None,
                 longitudinal_controller=None, mpc_params=None,
                 dynamic_max_substep_s=DEFAULT_RUNTIME_CONFIG.dynamic_max_substep_s):
        self.params = vehicle_params or VehicleParams()
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("control period must be positive and finite")
        if lateral_controller is None and controller_type not in ("LQR_controller", "MPC_controller"):
            raise ValueError("unknown lateral controller: " + str(controller_type))
        if vehicle_params is not None and vehicle_para is not None and not np.allclose(
                vehicle_para, vehicle_params.lateral_tuple, rtol=0, atol=1e-9):
            raise ValueError("vehicle_para disagrees with vehicle_params")
        vehicle_para = vehicle_para or self.params.lateral_tuple
        self.controller_type = controller_type
        self.vehicle_para = vehicle_para
        if lateral_q is not None:
            lateral_q = np.asarray(lateral_q, dtype=float)
            if lateral_q.ndim == 1:
                if lateral_q.size != 4:
                    raise ValueError("lateral_q must contain four diagonal weights")
                lateral_q = np.diag(lateral_q)
        actuator_params = (steering_params if steering_params and actuator_compensation
                           else SteeringParams())
        control_cost = dynamic_lateral_r if actuator_params.mode == "dynamic" else lateral_r
        lateral_kwargs = dict(Q=lateral_q, R=float(control_cost), ts=dt,
                              discretization=discretization,
                              max_substep_s=dynamic_max_substep_s)
        self.lat: LateralController = lateral_controller if lateral_controller is not None else (
            LateralMPCController(vehicle_para, **(lateral_kwargs | (mpc_params or {})))
            if controller_type == "MPC_controller"
            else LateralLQRController(vehicle_para, **lateral_kwargs)
        )
        self.lon: LongitudinalController = (
            longitudinal_controller if longitudinal_controller is not None
            else LongitudinalPIDController(**(longitudinal_params or {}))
        )
        if not isinstance(self.lat, LateralController):
            raise TypeError("lateral_controller must implement LateralController")
        if not isinstance(self.lon, LongitudinalController):
            raise TypeError("longitudinal_controller must implement LongitudinalController")
        self.lat.ts = self.lon.dt = dt
        self.lat.max_steer = self.params.max_steer
        self.lon.max_accel, self.lon.max_decel = self.params.max_accel, self.params.max_decel
        self.lat.configure_actuator(actuator_params)
        self.lat.configure_tracking(feedback_horizon_s=feedback_horizon_s,
                                    smooth_reference_heading=smooth_reference_heading)
        self.lon.set_target(target_speed_kmh)
        self.ref_path = []

    def update_ref_path(self, ref_path, *, reset=True):
        # Application reset calls retain their explicit reset semantics.
        # ROS passes reset=False for plans and never calls this per tick.
        if not reset and (ref_path is self.ref_path or ref_path == self.ref_path):
            return
        self.ref_path = ref_path
        if not reset and len(ref_path) >= 2:
            self.lat.set_path(ref_path, preserve=True)
            return
        self.lat.reset_tracking()
        if reset:
            self.lon.reset()

    def set_target_speed(self, speed_kmh):
        self.lon.set_target(speed_kmh)

    def step(self, x, y, phi, vx, vy, r, *, actual_steer=None, reference_accel=None,
             actual_accel=None):
        if not self.ref_path:
            return 0.0, 0.0, 0.0
        if self.lat.actuator_params.mode == "dynamic":
            if actual_steer is None or not math.isfinite(actual_steer):
                raise ValueError("actuator-aware control requires measured front-wheel angle")
            self.lat.actual_steer = actual_steer
        steer = self.lat.control(x, y, phi, vx, vy, r, self.ref_path)
        feedforward = {} if reference_accel is None else dict(reference_accel=reference_accel)
        if actual_accel is not None:
            feedforward['actual_accel'] = actual_accel
        accel = self.lon.control(
            (vx * vx + vy * vy) ** 0.5,
            coupling_accel=r * vy,
            **feedforward,
        )
        throttle = accel / self.params.max_accel if accel >= 0.0 else 0.0
        brake = abs(accel) / self.params.max_decel if accel < 0.0 else 0.0
        return steer, throttle, brake
