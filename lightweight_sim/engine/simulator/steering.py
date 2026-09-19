"""Physical front-wheel actuator; parameters are assumptions, not calibration.

Transport delay is an integer number of outer control/physics periods. The
command is held within that period. Lag and slew are integrated at substeps.
"""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SteeringParams:
    mode: str = "ideal"
    time_constant_s: float = 0.15
    delay_s: float = 0.05
    rate_limit_rad_s: float = 0.6
    source: str = "engineering_assumption_not_vehicle_calibrated"

    def __post_init__(self):
        if self.mode not in ("ideal", "dynamic"):
            raise ValueError("steering mode must be ideal or dynamic")
        if (not all(math.isfinite(v) for v in
                    (self.time_constant_s, self.delay_s, self.rate_limit_rad_s))
                or self.time_constant_s <= 0 or self.delay_s < 0
                or self.rate_limit_rad_s <= 0):
            raise ValueError("invalid steering parameters")

    def delay_steps(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("actuator period must be positive and finite")
        if self.mode == "ideal":
            return 0
        count = round(self.delay_s / dt)
        if not math.isclose(count * dt, self.delay_s, abs_tol=1e-9):
            raise ValueError("steering delay_s must be an integer multiple of control/physics dt")
        return count


def steering_profile(name):
    if name not in ("ideal", "assumed"):
        raise ValueError("unknown steering profile: " + str(name))
    return SteeringParams(mode="dynamic" if name == "assumed" else "ideal")


class SteeringActuator:
    def __init__(self, params, max_angle, initial_angle=0.0):
        self.params = params
        if not math.isfinite(max_angle) or max_angle <= 0:
            raise ValueError("max front-wheel angle must be positive")
        self.max_angle = max_angle
        self.angle = max(-max_angle, min(max_angle, initial_angle))
        self.requested = self.delayed = self.angle
        self.queue = None
        self.period = None
        self.rate_limited = False
        self.peak_rate_rad_s = 0.0

    def begin_period(self, command, dt):
        if not math.isfinite(command):
            raise ValueError("steering command must be finite")
        count = self.params.delay_steps(dt)
        if self.period is not None and not math.isclose(dt, self.period, abs_tol=1e-12):
            if self.params.mode != "ideal":
                raise ValueError("dynamic actuator requires a fixed outer period; reset before changing dt")
        self.period = dt
        if self.queue is None:
            self.queue = deque([self.angle] * count)
        self.requested = max(-self.max_angle, min(self.max_angle, command))
        if count:
            self.delayed = self.queue.popleft()
            self.queue.append(self.requested)
        else:
            self.delayed = self.requested
        self.rate_limited = False
        self.peak_rate_rad_s = 0.0

    def advance(self, dt):
        if self.period is None or not math.isfinite(dt) or dt <= 0:
            raise ValueError("begin_period must precede a positive actuator substep")
        previous = self.angle
        if self.params.mode == "ideal":
            self.angle = self.delayed
        else:
            # Exact solution of d(delta)/dt = clip((target-delta)/tau, +/-rate).
            # Unlike clipping an Euler step, the derivative itself is bounded.
            error = self.delayed - previous
            distance = abs(error)
            tau, rate = self.params.time_constant_s, self.params.rate_limit_rad_s
            self.peak_rate_rad_s = max(self.peak_rate_rad_s, min(distance/tau, rate))
            threshold = tau * rate
            linear_time = max(0.0, (distance-threshold)/rate)
            linear_used = min(dt, linear_time)
            self.rate_limited |= linear_time > 0
            remaining = distance - rate*linear_used
            remaining *= math.exp(-(dt-linear_used)/tau)
            self.angle = previous + math.copysign(distance-remaining, error)
        return self.angle
