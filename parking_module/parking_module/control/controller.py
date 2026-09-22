"""Gear-aware pure-pursuit controller for a planned parking trajectory."""

from __future__ import annotations

import math

from ..core.types import ControlCommand, ParkingConfig, ParkingTrajectory, VehicleState, wrap_angle


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class ParkingController:
    """Track a trajectory while explicitly handling forward/reverse changes."""

    def __init__(self, config: ParkingConfig | None = None, lookahead: float = 1.0):
        self.config = config or ParkingConfig()
        self.lookahead = max(0.2, lookahead)
        self._active_gear = 0
        self._progress_index = 0

    def reset(self) -> None:
        self._active_gear = 0
        self._progress_index = 0

    def command(self, state: VehicleState, trajectory: ParkingTrajectory) -> ControlCommand:
        if trajectory.empty:
            return ControlCommand()

        target_index = self._target_index(state, trajectory)
        first_reverse = self._first_reverse_index(trajectory)
        if self._progress_index < first_reverse:
            staging = trajectory.points[first_reverse].pose
            staging_distance = math.hypot(staging.x - state.x, staging.y - state.y)
            if staging_distance <= 0.4 and abs(state.vx) > self.config.speed_tolerance:
                return ControlCommand(brake=1.0, gear=0)
        target = trajectory.points[target_index]
        cfg = self.config
        position_error = math.hypot(target.pose.x - state.x, target.pose.y - state.y)
        goal = trajectory.goal or target.pose
        goal_error = math.hypot(goal.x - state.x, goal.y - state.y)
        goal_heading_error = abs(wrap_angle(goal.yaw - state.yaw))
        if goal_error <= cfg.position_tolerance and goal_heading_error <= cfg.heading_tolerance and abs(state.vx) <= cfg.speed_tolerance:
            self._active_gear = 0
            return ControlCommand(gear=-1 if state.vx < 0.0 else 0)

        desired_gear = 1 if target.gear >= 0 else -1
        if self._active_gear not in (0, desired_gear):
            if abs(state.vx) > cfg.speed_tolerance:
                return ControlCommand(brake=1.0, gear=0)
        self._active_gear = desired_gear

        motion_heading = target.pose.yaw if desired_gear > 0 else wrap_angle(target.pose.yaw + math.pi)
        target_angle = math.atan2(target.pose.y - state.y, target.pose.x - state.x)
        # For forward motion, steer from the current body heading to the
        # target point.  For reverse motion, the vehicle's travel direction is
        # opposite its body heading, so use the reverse motion heading.
        alpha = wrap_angle(target_angle - (state.yaw if desired_gear > 0 else motion_heading))
        distance = max(position_error, 0.4)
        steering = math.atan2(2.0 * cfg.wheelbase * math.sin(alpha), distance)
        steering += 0.25 * wrap_angle(target.pose.yaw - state.yaw)
        # The simulator uses signed longitudinal velocity in the kinematic
        # model.  Reverse motion changes the yaw response sign, so compensate
        # the steering command at this interface rather than coupling the
        # controller to a simulator implementation detail.
        if desired_gear < 0:
            steering = -steering
            if goal_error < 1.0:
                heading_error = wrap_angle(goal.yaw - state.yaw)
                lateral_axis = (-math.sin(goal.yaw), math.cos(goal.yaw))
                lateral_error = (
                    (goal.x - state.x) * lateral_axis[0]
                    + (goal.y - state.y) * lateral_axis[1]
                )
                # Near the slot endpoint, heading alignment is more
                # important than aggressively eliminating the last lateral
                # centimeter; the latter can drive the footprint into a wall.
                steering = -0.8 * heading_error + 0.3 * lateral_error
        steering = _clamp(steering, -cfg.max_steer, cfg.max_steer)

        # Taper speed with remaining goal distance so the footprint settles in
        # the slot instead of carrying the nominal reverse speed into the rear
        # boundary.
        desired_speed = min(abs(target.speed), max(0.0, goal_error * 0.2))
        speed_error = desired_speed - abs(state.vx)
        throttle = _clamp(speed_error * 1.5, 0.0, 1.0)
        brake = _clamp(-speed_error * 2.0, 0.0, 1.0)
        if goal_error <= cfg.position_tolerance and state.vx < -cfg.speed_tolerance:
            throttle = 0.0
            brake = 1.0
            desired_gear = -1
        return ControlCommand(steering=steering, throttle=throttle, brake=brake, gear=desired_gear)

    def _target_index(self, state: VehicleState, trajectory: ParkingTrajectory) -> int:
        first_reverse = self._first_reverse_index(trajectory)
        approach_end = max(0, first_reverse - 1)
        if self._progress_index < first_reverse:
            # Do not let the reverse curve, which can be spatially close to an
            # approach point, steal the target before the vehicle reaches and
            # stops at the staging point.
            candidates = range(self._progress_index, approach_end + 1)
            nearest = min(
                candidates,
                key=lambda index: (trajectory.points[index].pose.x - state.x) ** 2 + (trajectory.points[index].pose.y - state.y) ** 2,
            )
            self._progress_index = max(self._progress_index, nearest)
            staging = trajectory.points[approach_end].pose
            if (
                math.hypot(staging.x - state.x, staging.y - state.y) <= 0.4
                and abs(state.vx) <= self.config.speed_tolerance
            ):
                self._progress_index = first_reverse
        else:
            nearest = min(
                range(self._progress_index, len(trajectory.points)),
                key=lambda index: (trajectory.points[index].pose.x - state.x) ** 2 + (trajectory.points[index].pose.y - state.y) ** 2,
            )
            self._progress_index = max(self._progress_index, nearest)

        nearest = self._progress_index
        offset = max(1, int(round(self.lookahead / max(self.config.sample_step, 1e-3))))
        if nearest < first_reverse:
            return min(approach_end, nearest + offset)
        return min(len(trajectory.points) - 1, nearest + offset)

    @staticmethod
    def _first_reverse_index(trajectory: ParkingTrajectory) -> int:
        return next(
            (index for index, point in enumerate(trajectory.points) if point.gear < 0),
            len(trajectory.points),
        )
