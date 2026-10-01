"""Algorithm-independent lateral and longitudinal controller contracts."""

from abc import ABC, abstractmethod
import math

from ...simulator.steering import SteeringParams


class LateralController(ABC):
    """Physical steering interface; angles are radians and ``ts`` is seconds."""

    def __init__(self, ts=0.05):
        self.ts = float(ts)
        if not math.isfinite(self.ts) or self.ts <= 0:
            raise ValueError("control period must be positive and finite")
        self.max_steer = 0.5
        self.min_index = 0
        self.actuator_params = SteeringParams()
        self.actual_steer = 0.0
        self.command_history = []

    def configure_actuator(self, params):
        self.actuator_params = params
        self.command_history = [0.0] * params.delay_steps(self.ts)

    def reset_actuator_history(self):
        self.command_history = [0.0] * self.actuator_params.delay_steps(self.ts)
        self.actual_steer = 0.0

    def synchronize_actuator_state(self, angle, pending_commands):
        """Replace speculative candidate history with timestamped plant feedback."""
        expected = self.actuator_params.delay_steps(self.ts)
        values = list(pending_commands)
        if (len(values) != expected or not math.isfinite(angle)
                or abs(angle) > self.max_steer+1e-6
                or any(not math.isfinite(v) or abs(v) > self.max_steer+1e-6 for v in values)):
            raise ValueError('invalid actuator delay-queue feedback')
        self.command_history = values
        self.actual_steer = float(angle)

    def configure_tracking(self, *, feedback_horizon_s=0.0,
                           smooth_reference_heading=True):
        """Configure reference preprocessing independently of the solver."""
        self.feedback_horizon_s = float(feedback_horizon_s)
        if not math.isfinite(self.feedback_horizon_s) or self.feedback_horizon_s < 0:
            raise ValueError("feedback horizon must be non-negative and finite")
        self.smooth_reference_heading = bool(smooth_reference_heading)

    @abstractmethod
    def set_path(self, path, *, preserve=False):
        """Accept reference points (x, y, heading, curvature)."""

    @abstractmethod
    def reset_tracking(self):
        """Clear route progress and actuator history for a new run."""

    @abstractmethod
    def control(self, x, y, phi, vx, vy, r, ref_path):
        """Return a bounded front-wheel steering command in radians."""


class LongitudinalController(ABC):
    """Speed control interface returning physical acceleration in m/s²."""

    def __init__(self, dt=0.05, max_accel=3.0, max_decel=6.0):
        self.dt = float(dt)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        if not all(math.isfinite(v) and v > 0 for v in (self.dt, self.max_accel, self.max_decel)):
            raise ValueError("longitudinal period and acceleration limits must be positive and finite")
        self.target_speed = 50.0

    def set_target(self, speed_kmh):
        value = float(speed_kmh)
        if not math.isfinite(value) or value < 0:
            raise ValueError("target speed must be non-negative and finite")
        self.target_speed = value

    @abstractmethod
    def control(self, current_speed_ms, coupling_accel=0.0):
        """Return acceleration, with an optional lateral coupling input."""

    @abstractmethod
    def reset(self):
        """Clear controller memory while retaining configuration and target."""
