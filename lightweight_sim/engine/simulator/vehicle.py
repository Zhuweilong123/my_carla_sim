"""Vehicle models with a consistent physical command interface."""
import math
import numpy as np
from typing import List, Tuple, Optional
from .data_types import VehicleState, VehicleParams
from ..algorithms.utils.frenet import find_match_points

class EgoVehicle:
    def __init__(self, state: VehicleState, params: Optional[VehicleParams] = None):
        self._state = state
        self.params = params or VehicleParams()
        self.length = self.params.wheelbase + 1.0
        self.width = 2.0
        self._prev_state: Optional[VehicleState] = None

    def _clamp_steer(self, steer: float) -> float:
        return max(-self.params.max_steer, min(self.params.max_steer, float(steer)))

    def kinematic_step(self, steer: float, accel: float, dt: float) -> VehicleState:
        prev = self._state
        steer = self._clamp_steer(steer)
        v0 = max(0.0, prev.vx)
        v1 = max(0.0, v0 + float(accel) * dt)
        yaw_rate = v1 / self.params.wheelbase * math.tan(steer)
        phi = prev.phi + yaw_rate * dt
        x = prev.x + v1 * math.cos(phi) * dt
        y = prev.y + v1 * math.sin(phi) * dt
        self._prev_state = prev
        self._state = VehicleState(x=x, y=y, phi=phi, vx=v1, vy=0.0,
                                   r=yaw_rate, steer=steer, accel=accel,
                                   timestamp=prev.timestamp + dt)
        return self._state

    def dynamic_step(self, steer: float, accel: float, dt: float) -> VehicleState:
        prev = self._state
        steer = self._clamp_steer(steer)
        if prev.vx < 0.5:
            return self.kinematic_step(steer, accel, dt)
        a, b = self.params.a, self.params.b
        m, iz = self.params.m, self.params.Iz
        cf, cr = self.params.Cf, self.params.Cr
        vx = max(prev.vx, 0.5)
        vy_dot = ((cf + cr) / (m * vx)) * prev.vy + ((a * cf - b * cr) / (m * vx) - vx) * prev.r - (cf / m) * steer
        r_dot = ((a * cf - b * cr) / (iz * vx)) * prev.vy + ((a*a*cf + b*b*cr) / (iz * vx)) * prev.r - (a * cf / iz) * steer
        vx_dot = float(accel) + prev.r * prev.vy
        vx1 = max(0.0, prev.vx + vx_dot * dt)
        vy1 = prev.vy + vy_dot * dt
        r1 = prev.r + r_dot * dt
        phi = prev.phi + r1 * dt
        c, s = math.cos(prev.phi), math.sin(prev.phi)
        x = prev.x + (prev.vx*c - prev.vy*s) * dt
        y = prev.y + (prev.vx*s + prev.vy*c) * dt
        self._prev_state = prev
        self._state = VehicleState(x=x, y=y, phi=phi, vx=vx1, vy=vy1, r=r1,
                                   steer=steer, accel=accel,
                                   timestamp=prev.timestamp + dt)
        return self._state

    def step(self, steer: float, accel: float, dt: float, model: str = "kinematic") -> VehicleState:
        if model == "kinematic":
            return self.kinematic_step(steer, accel, dt)
        if model == "dynamic":
            return self.dynamic_step(steer, accel, dt)
        raise ValueError(f"Unknown vehicle model: {model}")

    def get_state(self) -> VehicleState:
        return self._state

    def set_state(self, state: VehicleState):
        self._prev_state = self._state
        self._state = state

    def get_error_state(self, ref_path: List[Tuple[float, float, float, float]], ts: float = 0.1) -> np.ndarray:
        if not ref_path:
            return np.zeros(4)
        state = self._state
        px = state.x + state.vx * ts * math.cos(state.phi) - state.vy * ts * math.sin(state.phi)
        py = state.y + state.vy * ts * math.cos(state.phi) + state.vx * ts * math.sin(state.phi)
        pphi = state.phi + state.r * ts
        indices, projections = find_match_points([(px, py)], ref_path, True, 0)
        idx = int(indices[0])
        rx, ry, rtheta, rkappa = projections[0]
        dx, dy = px - rx, py - ry
        ed = -math.sin(rtheta) * dx + math.cos(rtheta) * dy
        heading_error = math.atan2(math.sin(pphi-rtheta), math.cos(pphi-rtheta))
        vxw, vyw = state.world_velocity
        ed_dot = -math.sin(rtheta) * vxw + math.cos(rtheta) * vyw
        s_dot = (math.cos(rtheta) * vxw + math.sin(rtheta) * vyw) / max(1e-3, 1-rkappa*ed)
        return np.array([ed, ed_dot, heading_error, state.r-rkappa*s_dot])

    def get_corners(self) -> np.ndarray:
        s = self._state
        hl, hw = self.length/2, self.width/2
        local = np.array([[-hl,-hw],[hl,-hw],[hl,hw],[-hl,hw]])
        c, si = math.cos(s.phi), math.sin(s.phi)
        return local @ np.array([[c,-si],[si,c]]).T + np.array([[s.x,s.y]])