"""Longitudinal PID producing physical acceleration in m/s^2."""

from collections import deque


class LongitudinalPIDController:
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
        self.K_P = float(K_P)
        self.K_I = float(K_I)
        self.K_D = float(K_D)
        self.dt = float(dt)
        self.error_threshold = float(error_threshold)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_jerk = float(max_jerk)
        self.coupling_gain = float(coupling_gain)
        self.target_speed = 50.0
        self.error_buffer = deque(maxlen=60)
        self._previous_error = None
        self._filtered_derivative = 0.0
        self._previous_accel = 0.0

    def control(self, current_speed_ms, coupling_accel=0.0):
        error_ms = self.target_speed / 3.6 - float(current_speed_ms)
        error_kmh = error_ms * 3.6
        self.error_buffer.append(error_kmh)

        if self._previous_error is None:
            derivative = 0.0
        else:
            raw_derivative = (error_ms - self._previous_error) / self.dt
            self._filtered_derivative = (
                0.7 * self._filtered_derivative + 0.3 * raw_derivative
            )
            derivative = self._filtered_derivative
        self._previous_error = error_ms

        if abs(error_kmh) > self.error_threshold:
            integral = 0.0
            self.error_buffer.clear()
        else:
            integral = sum(self.error_buffer) * self.dt / 3.6

        requested_accel = (
            self.K_P * error_ms
            + self.K_I * integral
            + self.K_D * derivative
        )
        # EgoVehicle.dynamic_step uses vx_dot = accel + r * vy. Cancel the
        # lateral inertial coupling so the speed loop controls its target.
        requested_accel -= self.coupling_gain * float(coupling_accel)
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

    def set_target(self, speed_kmh):
        self.target_speed = float(speed_kmh)

    def reset(self):
        self.error_buffer.clear()
        self._previous_error = None
        self._filtered_derivative = 0.0
        self._previous_accel = 0.0
