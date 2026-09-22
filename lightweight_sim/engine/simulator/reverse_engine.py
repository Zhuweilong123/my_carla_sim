"""SimulationEngine adapter with signed gear support for parking maneuvers."""

from __future__ import annotations

from dataclasses import replace

from .data_types import ControlCommand, ScenarioConfig
from .engine import SimulationEngine as _BaseSimulationEngine


class SimulationEngine(_BaseSimulationEngine):
    """Enable signed reverse motion while reusing the common physics loop."""

    def __init__(self, config: ScenarioConfig):
        super().__init__(config)
        self.physics_dt = config.physics_dt

    def step(self, control: ControlCommand | None = None, dt=None):
        if control is None:
            return super().step(control, dt=dt)

        # A brake command must oppose the current signed velocity, not inherit
        # the gear selected by the last throttle command.  This is important
        # when the driver releases S and presses SPACE while still rolling
        # backward: the GUI may emit neutral/forward gear, but the physical
        # brake must still reduce the negative velocity.
        current_vx = float(self.ego.get_state().vx)
        effective_gear = int(control.gear)
        if control.brake > 0.0 and abs(current_vx) > 1e-6:
            effective_gear = -1 if current_vx < 0.0 else 1
        effective_brake = control.brake
        if control.brake > 0.0 and abs(current_vx) <= 1e-6 and control.throttle <= 0.0:
            # Holding the brake on a stationary vehicle must not create a
            # synthetic acceleration in either direction.
            effective_brake = 0.0
        control = replace(control, gear=effective_gear, brake=effective_brake)

        reverse_requested = int(control.gear) < 0
        reverse_was_enabled = self.ego.allow_reverse
        self.ego.allow_reverse = reverse_requested
        params = self.ego.params
        max_accel = params.max_accel
        max_decel = params.max_decel
        if reverse_requested:
            params.max_accel = -abs(max_accel)
            params.max_decel = -abs(max_decel)
        try:
            return super().step(control, dt=dt)
        finally:
            params.max_accel = abs(max_accel)
            params.max_decel = abs(max_decel)
            self.ego.allow_reverse = reverse_was_enabled
