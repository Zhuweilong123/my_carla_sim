"""SimulationEngine adapter with signed gear support for parking maneuvers."""

from __future__ import annotations

from .data_types import ControlCommand, ScenarioConfig
from .engine import SimulationEngine as _BaseSimulationEngine


class SimulationEngine(_BaseSimulationEngine):
    """Use the common engine while mapping reverse gear to signed acceleration.

    The legacy engine computes acceleration from normalized throttle/brake.
    Temporarily mirroring the actuator limits lets a reverse throttle produce
    negative longitudinal acceleration without duplicating the physics loop.
    """

    def __init__(self, config: ScenarioConfig):
        super().__init__(config)
        self.physics_dt = config.physics_dt

    def step(self, control: ControlCommand | None = None, dt=None):
        if control is None or int(control.gear) >= 0:
            return super().step(control, dt=dt)

        params = self.ego.params
        max_accel = params.max_accel
        max_decel = params.max_decel
        reverse_was_enabled = self.ego.allow_reverse
        params.max_accel = -abs(max_accel)
        params.max_decel = -abs(max_decel)
        self.ego.allow_reverse = True
        try:
            return super().step(control, dt=dt)
        finally:
            params.max_accel = abs(max_accel)
            params.max_decel = abs(max_decel)
            self.ego.allow_reverse = reverse_was_enabled
