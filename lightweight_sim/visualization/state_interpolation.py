"""Wall-clock playback of timestamped poses, used only for GUI drawing."""

from dataclasses import replace
import math
import time

from ..engine.simulator.data_types import VehicleState


class VehicleStateInterpolator:
    """Play the last state interval without predicting beyond received data."""

    MAX_INTERVAL_S = 0.25

    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.latest = None
        self.received_at = None
        self.interval_s = 0.0

    def push(self, state: VehicleState, received_at=None):
        now = time.monotonic() if received_at is None else received_at
        current = replace(state)
        if self.latest is not None:
            dt = current.timestamp - self.latest.timestamp
            wall_dt = now - self.received_at
            if dt == 0:
                if (current.x, current.y, current.phi) == (
                        self.latest.x, self.latest.y, self.latest.phi):
                    # Paused messages must not keep restarting playback.
                    self.latest = current
                    return
                self.reset()  # Teleport/reset at the same timestamp.
            elif 0 < dt <= self.MAX_INTERVAL_S and 0 < wall_dt <= self.MAX_INTERVAL_S:
                self.previous = self.latest
                self.interval_s = wall_dt
            else:
                self.reset()  # Time reset, long gap, or stale transport.
        self.latest = current
        self.received_at = now

    def sample(self, now=None, paused=False):
        if self.latest is None:
            return None
        if paused:
            self.previous = None
            return self.latest
        if self.previous is None:
            return self.latest
        now = time.monotonic() if now is None else now
        alpha = max(0.0, min(1.0, (now - self.received_at) / self.interval_s))
        if alpha >= 1.0:
            return self.latest  # Lost messages freeze at the last actual pose.
        start, end = self.previous, self.latest
        heading_delta = math.atan2(math.sin(end.phi - start.phi),
                                   math.cos(end.phi - start.phi))
        heading = start.phi + alpha * heading_delta
        return replace(
            end,
            x=start.x + alpha * (end.x - start.x),
            y=start.y + alpha * (end.y - start.y),
            phi=math.atan2(math.sin(heading), math.cos(heading)),
            timestamp=start.timestamp + alpha * (end.timestamp - start.timestamp),
        )
