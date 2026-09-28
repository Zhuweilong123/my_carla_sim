"""Gear-aware pure-pursuit controller for a planned parking trajectory."""

from __future__ import annotations

import math
from bisect import bisect_right

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
        self._trajectory = None
        self._trajectory_s = []
        self._last_position = None
        self._uncredited_travel = 0.0

    def reset(self) -> None:
        self._active_gear = 0
        self._progress_index = 0
        self._trajectory = None
        self._trajectory_s = []
        self._last_position = None
        self._uncredited_travel = 0.0

    def command(self, state: VehicleState, trajectory: ParkingTrajectory) -> ControlCommand:
        if trajectory.empty:
            return ControlCommand()

        target_index = self._target_index(state, trajectory)
        transition = self._next_gear_transition(
            trajectory, self._progress_index
        )
        if transition is not None:
            staging = trajectory.points[transition].pose
            staging_distance = math.hypot(staging.x - state.x, staging.y - state.y)
            if (
                staging_distance <= self.config.staging_tolerance
                and abs(state.vx) > self.config.speed_tolerance
            ):
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

        motion_heading = state.yaw if desired_gear > 0 else wrap_angle(state.yaw + math.pi)
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
        if transition is not None:
            staging = trajectory.points[transition].pose
            staging_distance = math.hypot(staging.x - state.x, staging.y - state.y)
            remaining_distance = max(0.0, staging_distance - cfg.staging_tolerance)
            staging_speed = math.sqrt(
                2.0 * cfg.approach_deceleration * remaining_distance
            )
            desired_speed = min(desired_speed, staging_speed)
        speed_error = desired_speed - abs(state.vx)
        throttle = _clamp(speed_error * 1.5, 0.0, 1.0)
        brake = _clamp(-speed_error * 2.0, 0.0, 1.0)
        if goal_error <= cfg.position_tolerance and state.vx < -cfg.speed_tolerance:
            throttle = 0.0
            brake = 1.0
            desired_gear = -1
        return ControlCommand(steering=steering, throttle=throttle, brake=brake, gear=desired_gear)

    def _target_index(self, state: VehicleState, trajectory: ParkingTrajectory) -> int:
        if trajectory is not self._trajectory:
            self._trajectory = trajectory
            self._trajectory_s = [0.0]
            for first, second in zip(trajectory.points, trajectory.points[1:]):
                self._trajectory_s.append(
                    self._trajectory_s[-1]
                    + math.hypot(
                        second.pose.x - first.pose.x,
                        second.pose.y - first.pose.y,
                    )
                )
            self._progress_index = min(
                self._progress_index, len(trajectory.points) - 1
            )
            self._last_position = None
            self._uncredited_travel = 0.0

        max_progress_index = len(trajectory.points) - 1
        if self._last_position is not None:
            self._uncredited_travel += math.hypot(
                state.x - self._last_position[0],
                state.y - self._last_position[1],
            )
            # Permit a small discretization margin, but never let a nearby
            # future branch advance progress by an entire loop or turn.
            progress_slack = min(0.1, max(0.02, self.config.sample_step * 0.5))
            progress_limit = (
                self._trajectory_s[self._progress_index]
                + self._uncredited_travel
                + progress_slack
            )
            max_progress_index = max(
                self._progress_index,
                min(
                    len(trajectory.points) - 1,
                    bisect_right(self._trajectory_s, progress_limit) - 1,
                ),
            )

        progress_before = self._trajectory_s[self._progress_index]
        while True:
            transition = self._next_gear_transition(
                trajectory, self._progress_index
            )
            segment_end = (
                transition - 1
                if transition is not None
                else len(trajectory.points) - 1
            )
            candidate_end = min(segment_end, max_progress_index)
            candidates = range(self._progress_index, candidate_end + 1)
            nearest = min(
                candidates,
                key=lambda index: (
                    trajectory.points[index].pose.x - state.x
                ) ** 2 + (
                    trajectory.points[index].pose.y - state.y
                ) ** 2,
            )
            self._progress_index = max(self._progress_index, nearest)

            if transition is None:
                break
            transition_pose = trajectory.points[transition].pose
            transition_distance = math.hypot(
                transition_pose.x - state.x, transition_pose.y - state.y
            )
            if (
                transition_distance <= self.config.staging_tolerance
                and abs(state.vx) <= self.config.speed_tolerance
            ):
                self._progress_index = transition
                max_progress_index = max(max_progress_index, transition)
                continue
            break

        progress_after = self._trajectory_s[self._progress_index]
        self._uncredited_travel = max(
            0.0, self._uncredited_travel - (progress_after - progress_before)
        )
        self._last_position = (state.x, state.y)

        # The planner may use different point spacing in each segment. Accrue
        # physical distance along the active gear segment instead of converting
        # a nominal sample spacing into an index offset.
        transition = self._next_gear_transition(
            trajectory, self._progress_index
        )
        segment_end = (
            transition - 1
            if transition is not None
            else len(trajectory.points) - 1
        )
        target_index = self._progress_index
        distance = 0.0
        while target_index < segment_end and distance < self.lookahead:
            first = trajectory.points[target_index].pose
            second = trajectory.points[target_index + 1].pose
            distance += math.hypot(second.x - first.x, second.y - first.y)
            target_index += 1
        return target_index

    @staticmethod
    def _next_gear_transition(
        trajectory: ParkingTrajectory, progress_index: int
    ) -> int | None:
        for index in range(max(1, progress_index + 1), len(trajectory.points)):
            if trajectory.points[index].gear != trajectory.points[index - 1].gear:
                return index
        return None

    @staticmethod
    def _first_reverse_index(trajectory: ParkingTrajectory) -> int:
        return next(
            (index for index, point in enumerate(trajectory.points) if point.gear < 0),
            len(trajectory.points),
        )
