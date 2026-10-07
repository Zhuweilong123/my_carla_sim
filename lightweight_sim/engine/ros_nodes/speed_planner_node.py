"""Independent asynchronous ST speed planner; geometry is immutable input."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lightweight_sim_msgs.msg import (Path, VehicleState, ObstacleArray,
                                      ReferenceLine, SpeedProfile, SpeedPoint)
from ..algorithms.planner.st_speed import STSpeedPlanner, SpeedConfig
from ..algorithms.utils.route import RouteGeometry
from ..simulator.data_types import VehicleParams, Obstacle
from .message_conversions import message_to_state, path_to_tuples
from .route_session import decode_sequence, parse_context
from .qos import latched_path_qos, sensor_data_qos


def projected_speed_limits(path, reference, context):
    """Transfer route limits onto local arc length, preserving crossing branches."""
    if reference is None or not reference.success or len(reference.points) < 2:
        return []
    route = RouteGeometry(path_to_tuples(reference))
    local = RouteGeometry(path)
    boundaries, speeds, end = [], [], 0.
    for segment in reference.segments:
        end += max(0., segment.length_m)
        maneuver = segment.maneuver.lower()
        kind = ('curve' if maneuver in ('left', 'right', 'u_turn', 'curve') else
                'lane_change' if maneuver in ('lane_change', 'merge') else
                'intersection' if maneuver in ('intersection', 'junction') else 'straight')
        boundaries.append(end)
        speeds.append(float(context.get('speed_limits', {}).get(kind, segment.speed_limit_kmh))
                      *float(context.get('target_speed_ratio', 1.)))
    if not speeds:
        return []
    # Vectorize segment projection; the scalar route projector would scan
    # every global segment in Python for every local sample, competing with
    # ROS callbacks for the GIL and adding avoidable handover latency.
    coordinates = np.array(route.path)[:, :2]
    delta = np.diff(coordinates, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    valid_segments = lengths > 1e-9
    base = np.array(route.s[:-1])[valid_segments]
    start = coordinates[:-1][valid_segments]
    delta, lengths = delta[valid_segments], lengths[valid_segments]
    headings = np.arctan2(delta[:, 1], delta[:, 0])
    limits, previous = [], None
    for i, point in enumerate(path):
        starts = base.copy()
        if route.closed and previous is not None:
            starts += np.round((previous-starts)/route.length)*route.length
        lo, hi = np.zeros_like(lengths), np.ones_like(lengths)
        if previous is not None:
            lo = np.maximum(lo, (previous-3.-starts)/lengths)
            hi = np.minimum(hi, (previous+10.-starts)/lengths)
        ratios = np.einsum('ij,ij->i', np.array(point[:2])-start, delta)/lengths**2
        ratios = np.maximum(lo, np.minimum(hi, ratios))
        projected = start+ratios[:, None]*delta
        heading_error = np.arctan2(np.sin(point[2]-headings), np.cos(point[2]-headings))
        scores = np.sum((projected-np.array(point[:2]))**2, axis=1)+2*heading_error**2
        scores[lo > hi] = np.inf
        if not np.isfinite(scores).any():
            raise ValueError('no continuous route projection for speed limits')
        if previous is not None:
            scores += 1e-6*(starts+ratios*lengths-previous)**2
        best = int(np.argmin(scores))
        previous = float(starts[best]+ratios[best]*lengths[best])
        station = previous % route.length if route.closed else previous
        index = min(len(speeds)-1, int(np.searchsorted(boundaries, station)))
        begin = local.s[max(0, i-1)]
        limits.append((begin, local.s[min(i+1, len(path)-1)], speeds[index]))
    return limits


class SpeedPlannerNode(Node):
    def __init__(self):
        super().__init__('speed_planner_node')
        for name, value in dict(plan_period=.1, result_poll_period=.02,
                                state_timeout=.25, path_timeout=1.5,
                                max_result_age_s=.4, reference_reuse_s=.6,
                                **SpeedConfig().__dict__).items():
            self.declare_parameter(name, value)
        config = SpeedConfig(**{name: self.get_parameter(name).value
                                for name in SpeedConfig.__dataclass_fields__})
        self.planner = STSpeedPlanner(config)
        self.route_context = self.state = self.path = self.reference = None
        self.obstacles = []
        self.obstacle_time = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='st-speed')
        self.future = None
        self.accepted = None
        self.request_obstacles = None
        self.pub = self.create_publisher(SpeedProfile, 'speed_profile', latched_path_qos())
        self.diagnostics = self.create_publisher(String, 'speed/diagnostics', sensor_data_qos())
        self.create_subscription(String, 'sim/context', self._on_context, latched_path_qos())
        self.create_subscription(Path, 'planned_path', self._on_path, latched_path_qos())
        self.create_subscription(VehicleState, 'vehicle/state', self._on_state, sensor_data_qos())
        self.create_subscription(ObstacleArray, 'obstacles', self._on_obstacles, sensor_data_qos())
        self.create_subscription(ReferenceLine, 'routing/reference_line', self._on_reference, latched_path_qos())
        self.create_timer(float(self.get_parameter('plan_period').value), self._request)
        self.create_timer(float(self.get_parameter('result_poll_period').value), self._poll)

    def _now(self):
        return self.get_clock().now().nanoseconds/1e9

    def _on_context(self, message):
        context = parse_context(message)
        if self.route_context and context['run_id'] <= self.route_context['run_id']:
            return
        self.route_context = context
        self.accepted = None
        self.state = self.path = None
        if self.reference and self.reference.request_id != context['run_id']:
            self.reference = None
        self.obstacles = [Obstacle(**o) for o in context.get('scene_obstacles', [])]
        self.obstacle_time = None

    def _on_reference(self, message):
        if self.route_context is None or message.request_id >= self.route_context['run_id']:
            self.reference = message

    def _on_state(self, message):
        self.state = message_to_state(message)

    def _on_obstacles(self, message):
        self.obstacles = [Obstacle(id=o.id, x=o.x, y=o.y, length=o.length, width=o.width,
                                  speed=o.speed, heading=o.heading, type=o.type) for o in message.obstacles]
        self.obstacle_time = self._now()

    def _on_path(self, message):
        run, version = decode_sequence(message.sequence)
        if not self.route_context or run != self.route_context['run_id'] or version <= 0:
            return
        if self.path and message.sequence <= self.path.sequence:
            return
        self.path = message
        if len(message.points) < 2:
            self.accepted = None
            self._publish(None, run, message.sequence, self._now(), 'empty_path')

    def _obstacle_signature(self):
        return tuple(sorted((o.id, o.x, o.y, o.heading, o.speed, o.length, o.width)
                            for o in self.obstacles))

    def _request(self):
        if (self.future is not None or self.route_context is None or self.path is None
                or self.state is None or len(self.path.points) < 2
                or self.route_context.get('maneuver') == 'reverse_parking'):
            return
        now = self._now()
        stamp = self.path.header.stamp.sec+self.path.header.stamp.nanosec/1e9
        if (not 0 <= now-self.state.timestamp <= float(self.get_parameter('state_timeout').value)
                or not -.1 <= now-stamp <= float(self.get_parameter('path_timeout').value)):
            return
        if (self.obstacle_time is None or not 0 <= now-self.obstacle_time <=
                float(self.get_parameter('state_timeout').value)):
            self._publish(None, self.route_context['run_id'], self.path.sequence, now, 'stale_obstacles')
            return
        context, path = dict(self.route_context), path_to_tuples(self.path)
        state, obstacles = replace(self.state), list(self.obstacles)
        self.request_obstacles = self._obstacle_signature()
        run, sequence = context['run_id'], self.path.sequence
        reference = self.reference if self.reference and self.reference.request_id == run else None
        started = time.monotonic()
        def solve():
            try:
                result = self.planner.plan(path, state, obstacles,
                    vehicle=VehicleParams(**context.get('vehicle_parameters', {})),
                    target_speed_kmh=context['target_speed_kmh'],
                    max_lateral_accel=context.get('max_lateral_accel_mps2', 2.),
                    destination=context.get('destination'),
                    closed_route=context.get('routing_closed_loop', False),
                    speed_limits=projected_speed_limits(path, reference, context))
                return result, run, sequence, state.timestamp, result.status, time.monotonic()-started
            except (ValueError, RuntimeError, ArithmeticError) as exc:
                return None, run, sequence, state.timestamp, 'solver_error:'+str(exc), time.monotonic()-started
        self.future = self.worker.submit(solve)

    def _poll(self):
        if self.future is None or not self.future.done():
            return
        future, self.future = self.future, None
        result, run, sequence, stamp, status, duration = future.result()
        if (not self.route_context or run != self.route_context['run_id'] or self.path is None
                or len(self.path.points) < 2):
            return
        # A newer geometric candidate does not invalidate a coherent pair
        # still within the handover age. Empty paths invalidate immediately.
        if not 0 <= self._now()-stamp <= float(self.get_parameter('max_result_age_s').value):
            result, status = None, 'late_result'
        if self.request_obstacles != self._obstacle_signature():
            result, status = None, 'obstacles_changed'
        published_status = self._publish(result, run, sequence, stamp, status)
        served = self.accepted if published_status.startswith('reused:') else None
        self.diagnostics.publish(String(data=json.dumps(dict(run_id=run,
            path_sequence=served[2] if served else sequence, requested_path_sequence=sequence,
            status=published_status, solve_time_s=duration,
            stop_s=served[0].stop_s if served else result.stop_s if result else None))))

    def _publish(self, result, run, sequence, stamp, status):
        reusable = self.accepted
        if (not (result and result.valid) and reusable is not None
                and status not in ('empty_path', 'stale_obstacles', 'unsupported_moving_conflict')):
            old, old_run, old_sequence, old_stamp, signature = reusable
            elapsed = self._now()-old_stamp
            if (run == old_run and self.state is not None and self.path is not None
                    and 0 <= self._now()-self.state.timestamp <= float(self.get_parameter('state_timeout').value)
                    and len(self.path.points) >= 2 and self.obstacle_time is not None
                    and 0 <= self._now()-self.obstacle_time <= float(self.get_parameter('state_timeout').value)
                    and 0 <= elapsed <= float(self.get_parameter('reference_reuse_s').value)
                    and elapsed <= old.time[-1] and signature == self._obstacle_signature()
                    and abs(old.sample(elapsed)[1]-self.state.speed) <= 1.5):
                result, run, sequence, stamp = old, old_run, old_sequence, old_stamp
                status = 'reused:'+status
        elif result and result.valid:
            self.accepted = (result, run, sequence, stamp, self.request_obstacles)
        message = SpeedProfile(run_id=int(run), path_sequence=int(sequence), status=status)
        message.header.stamp.sec = int(stamp)
        message.header.stamp.nanosec = int(round((stamp-int(stamp))*1e9))
        if message.header.stamp.nanosec >= 1000000000:
            message.header.stamp.sec += 1
            message.header.stamp.nanosec = 0
        message.header.frame_id = 'map'
        message.valid = bool(result and result.valid)
        if message.valid:
            message.origin_s = float(result.origin_s)
            for i, t in enumerate(result.time):
                message.points.append(SpeedPoint(time_from_start=float(t), s=float(result.s[i]),
                    speed=float(max(0., result.speed[i])), acceleration=float(result.accel[i]),
                    jerk=float(result.jerk[i]) if i < len(result.jerk) else 0.))
        self.pub.publish(message)
        return status

    def destroy_node(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeedPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass
