"""Longitudinal PID producing physical acceleration in m/s^2."""

from collections import deque
import math

from .base import LongitudinalController


class LongitudinalPIDController(LongitudinalController):
    """Speed controller with derivative damping and dynamic coupling rejection."""

    def __init__(
        self,
        K_P=1.15,
        K_I=0.0,
        K_D=0.55,
        dt=0.05,
        error_threshold=1.0,
        max_accel=3.0,
        max_decel=6.0,
        max_jerk=40.0,
        coupling_gain=0.5,
    ):
        super().__init__(dt=dt, max_accel=max_accel, max_decel=max_decel)
        self.K_P = float(K_P)
        self.K_I = float(K_I)
        self.K_D = float(K_D)
        self.error_threshold = float(error_threshold)
        self.max_jerk = float(max_jerk)
        self.coupling_gain = float(coupling_gain)
        if (not all(math.isfinite(v) and v >= 0 for v in
                    (self.K_P, self.K_I, self.K_D, self.error_threshold, self.coupling_gain))
                or not math.isfinite(self.max_jerk) or self.max_jerk <= 0):
            raise ValueError("PID gains/threshold must be non-negative and jerk positive, all finite")
        self.error_buffer = deque(maxlen=60)
        self._previous_error = None
        self._filtered_derivative = 0.0
        self._previous_accel = 0.0

    def control(self, current_speed_ms, coupling_accel=0.0, reference_accel=None):
        if not all(math.isfinite(float(v)) for v in
                   (current_speed_ms, coupling_accel, reference_accel or 0.)):
            raise ValueError('longitudinal input must be finite')
        error_ms = self.target_speed / 3.6 - float(current_speed_ms)
        error_kmh = error_ms * 3.6
        self.error_buffer.append(error_kmh)

        if self._previous_error is None:
            derivative = 0.0
        else:
            raw_derivative = (error_ms - self._previous_error) / self.dt
            if reference_accel is not None:
                # D acts on tracking acceleration error, avoiding a setpoint
                # step kick while preserving nominal acceleration feedforward.
                raw_derivative = float(reference_accel) - (float(current_speed_ms)-self._previous_speed)/self.dt
            self._filtered_derivative = (
                0.7 * self._filtered_derivative + 0.3 * raw_derivative
            )
            derivative = self._filtered_derivative
        self._previous_error = error_ms
        self._previous_speed = float(current_speed_ms)

        if abs(error_kmh) > self.error_threshold:
            integral = 0.0
            self.error_buffer.clear()
        else:
            integral = sum(self.error_buffer) * self.dt / 3.6

        requested_accel = (
            self.K_P * error_ms
            + self.K_I * integral
            + self.K_D * derivative
            + (float(reference_accel) if reference_accel is not None else 0.)
        )
        # EgoVehicle.dynamic_step uses vx_dot = accel + r * vy. Cancel the
        # lateral inertial coupling so the speed loop controls its target.
        requested_accel -= self.coupling_gain * float(coupling_accel)
        if ((requested_accel > self.max_accel and error_ms > 0)
                or (requested_accel < -self.max_decel and error_ms < 0)):
            self.error_buffer.clear()
        requested_accel = max(
            -self.max_decel,
            min(self.max_accel, requested_accel),
        )

        max_delta = self.max_jerk * self.dt
        accel = max(
            self._previous_accel - max_delta,
            min(self._previous_accel + max_delta, requested_accel),
        )
        self._previous_accel = accel
        return accel

    def reset(self):
        self.error_buffer.clear()
        self._previous_error = None
        self._filtered_derivative = 0.0
        self._previous_accel = 0.0
