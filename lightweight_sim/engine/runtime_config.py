"""Shared runtime defaults for all lightweight simulator front ends.

The standalone and ROS 2 adapters keep different transport layers, but they
must start from the same timing and safety defaults. Scenario-specific values
remain in :class:`ScenarioConfig` and are propagated by the ROS simulator
through ``sim/context``.
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class RuntimeConfig:
    """Defaults shared by the standalone and ROS 2 execution paths."""

    physics_dt: float = 0.05
    control_period: float = 0.05
    plan_period: float = 0.5
    prediction_time: float = 0.2
    command_timeout: float = 0.25
    state_timeout: float = 0.25
    plan_timeout: float = 1.5
    controller: str = "LQR_controller"
    target_speed_kmh: float = 40.0
    lane_width: float = 3.5
    num_lanes: int = 2
    steering_profile: str = "ideal"

    def plan_interval_steps(self, physics_dt: Optional[float] = None) -> int:
        """Return the fixed-step count matching ``plan_period``."""

        dt = self.physics_dt if physics_dt is None else float(physics_dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("physics_dt must be positive and finite")
        return max(1, round(self.plan_period / dt))


DEFAULT_RUNTIME_CONFIG = RuntimeConfig()
