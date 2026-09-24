"""Asynchronous latest-only obstacle-aware lane planner."""

from __future__ import annotations

import math
import queue
import threading
import time

from ...simulator.logging_utils import get_run_logger
from ..utils.frenet import find_match_points


class MotionPlanner:
    def __init__(
        self,
        global_frenet_path,
        lane_width=3.5,
        num_lanes=2,
        *,
        horizon_points=80,
        corridor_margin_m=1.1,
        transition_distance_m=35.0,
        obstacle_longitudinal_min_m=-5.0,
        obstacle_longitudinal_max_m=65.0,
        obstacle_lateral_clearance_m=2.2,
    ):
        self.global_path = global_frenet_path
        self.lane_width = lane_width
        self.num_lanes = num_lanes
        self.horizon_points = int(horizon_points)
        self.corridor_margin_m = float(corridor_margin_m)
        self.transition_distance_m = float(transition_distance_m)
        self.obstacle_longitudinal_min_m = float(obstacle_longitudinal_min_m)
        self.obstacle_longitudinal_max_m = float(obstacle_longitudinal_max_m)
        self.obstacle_lateral_clearance_m = float(obstacle_lateral_clearance_m)
        if self.horizon_points < 2:
            raise ValueError("horizon_points must be at least two")
        if self.corridor_margin_m < 0.0:
            raise ValueError("corridor margin must be non-negative")
        if self.transition_distance_m <= 0.0:
            raise ValueError("transition distance must be positive")
        self._requests = queue.Queue(maxsize=1)
        self._responses = queue.Queue(maxsize=1)
        self._thread = None
        self._running = False
        self._request_seq = 0
        self.logger = get_run_logger()
        self.logger.info(
            "planner initialized; path_points=%d lanes=%d lane_width=%.2f",
            len(self.global_path),
            self.num_lanes,
            self.lane_width,
        )

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._worker,
            daemon=True,
            name="lightweight-motion-planner",
        )
        self._thread.start()
        self.logger.info("planner worker started")

    def stop(self):
        if not self._running:
            return
        self._running = False
        try:
            self._requests.put_nowait(None)
        except queue.Full:
            pass
        if self._thread:
            self._thread.join(timeout=2)
        if self._thread and self._thread.is_alive():
            self.logger.warning("planner worker did not stop within timeout")
        else:
            self.logger.info("planner worker stopped")

    def plan(
        self,
        ego_state,
        obstacles,
        vehicle_v=(0, 0),
        vehicle_a=(0, 0),
        pred_loc=None,
        vehicle_loc=None,
    ):
        del vehicle_v, vehicle_a
        if not self._running or not self.global_path:
            self.logger.warning(
                "planning request rejected; running=%s path_points=%d",
                self._running,
                len(self.global_path),
            )
            return False

        vehicle_loc = vehicle_loc or (ego_state.x, ego_state.y)
        pred_loc = pred_loc or vehicle_loc
        while True:
            try:
                self._requests.get_nowait()
            except queue.Empty:
                break

        self._request_seq += 1
        request_id = self._request_seq
        obstacle_data = [
            (o.x, o.y, o.length, o.width, o.speed, o.heading)
            for o in obstacles
        ]
        data = (request_id, pred_loc, vehicle_loc, obstacle_data)
        try:
            self._requests.put_nowait(data)
            obstacle_summary = ";".join(
                "id=%s pos=(%.2f,%.2f) size=(%.2f,%.2f)" % (
                    obstacle.id,
                    obstacle.x,
                    obstacle.y,
                    obstacle.length,
                    obstacle.width,
                )
                for obstacle in obstacles
            )
            self.logger.debug(
                "planning request queued; id=%d obstacles=%d %s",
                request_id,
                len(obstacle_data),
                obstacle_summary,
            )
            return True
        except queue.Full:
            self.logger.debug("planning request dropped because queue is full")
            return False

    def poll_result(self):
        return not self._responses.empty()

    def get_result(self):
        try:
            return self._responses.get_nowait()
        except queue.Empty:
            return None

    def _worker(self):
        while self._running:
            try:
                data = self._requests.get(timeout=0.2)
            except queue.Empty:
                continue
            if data is None:
                break

            request_id, pred_loc, vehicle_loc, obstacles = data
            started = time.perf_counter()
            try:
                path = self._plan(pred_loc, vehicle_loc, obstacles)
                while True:
                    try:
                        self._responses.get_nowait()
                    except queue.Empty:
                        break
                self._responses.put_nowait(path)
                self.logger.debug(
                    "planning completed; id=%d duration_ms=%.2f points=%d",
                    request_id,
                    (time.perf_counter() - started) * 1000,
                    len(path),
                )
            except Exception:
                self.logger.exception("planning failed; id=%d", request_id)
                try:
                    self._responses.put_nowait([])
                except queue.Full:
                    pass

    def _plan(self, pred_loc, vehicle_loc, obstacles):
        path = self.global_path
        if len(path) < 2:
            return []

        idx, _ = find_match_points([pred_loc], path, True, 0)
        start = max(0, int(idx[0]))
        horizon = min(len(path), start + self.horizon_points)
        ref = path[start:horizon]
        theta = ref[0][2]
        normal = (-math.sin(theta), math.cos(theta))
        l0 = normal[0] * (vehicle_loc[0] - ref[0][0]) + normal[1] * (
            vehicle_loc[1] - ref[0][1]
        )

        half_road = self.num_lanes * self.lane_width / 2
        usable = half_road - self.corridor_margin_m
        candidates = [
            -half_road + self.lane_width * (i + 0.5)
            for i in range(self.num_lanes)
        ]
        candidates.append(max(-usable, min(usable, l0)))
        candidates = sorted(set(round(value, 3) for value in candidates))

        obstacle_sl = []
        for ox, oy, length, width, speed, heading in obstacles:
            del length, width, speed, heading
            obstacle_index, _ = find_match_points([(ox, oy)], path, True, 0)
            j = max(0, min(len(path) - 1, int(obstacle_index[0])))
            reference = path[j]
            normal = (-math.sin(reference[2]), math.cos(reference[2]))
            ds = (ox - reference[0]) * math.cos(reference[2]) + (
                oy - reference[1]
            ) * math.sin(reference[2])
            lateral = (ox - reference[0]) * normal[0] + (
                oy - reference[1]
            ) * normal[1]
            obstacle_sl.append((j - start, ds, lateral))

        def is_safe(target):
            for ds_index, _, lateral in obstacle_sl:
                if (
                    self.obstacle_longitudinal_min_m
                    <= ds_index
                    <= self.obstacle_longitudinal_max_m
                    and abs(target - lateral) < self.obstacle_lateral_clearance_m
                ):
                    return False
            return True

        valid = [candidate for candidate in candidates if is_safe(candidate)]
        target = min(
            valid or [max(-usable, min(usable, l0))],
            key=lambda candidate: abs(candidate - l0),
        )

        # Complete a lane change within a bounded longitudinal distance.
        # The previous index-based profile could take about 100 m on the
        # downsampled three-lane road and was restarted at every replan.
        result = []
        travelled = 0.0
        for index, point in enumerate(ref):
            if index > 0:
                previous = ref[index - 1]
                travelled += math.hypot(
                    point[0] - previous[0],
                    point[1] - previous[1],
                )
            ratio = max(0.0, min(1.0, travelled / self.transition_distance_m))
            smooth = ratio * ratio * (3 - 2 * ratio)
            lateral = l0 + (target - l0) * smooth
            normal = (-math.sin(point[2]), math.cos(point[2]))
            result.append(
                (
                    point[0] + lateral * normal[0],
                    point[1] + lateral * normal[1],
                    point[2],
                    point[3],
                )
            )
        return result
