"""ROS 2 node that exposes the deterministic SimulationEngine."""

import math
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time
from lightweight_sim_msgs.msg import ControlCommand as RosControlCommand
from lightweight_sim_msgs.msg import Obstacle as RosObstacle
from lightweight_sim_msgs.msg import ObstacleArray
from lightweight_sim_msgs.msg import Path as RosPath
from lightweight_sim_msgs.msg import PathPoint
from lightweight_sim_msgs.msg import SimulationStatus
from lightweight_sim_msgs.msg import VehicleState as RosVehicleState
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty, SetBool, Trigger

from ..simulator.data_types import ControlCommand, VehicleParams
from ..simulator.reverse_engine import SimulationEngine
from ..simulator.scenarios import make_scenario
from ..simulator.steering import SteeringParams, steering_profile
from ..runtime_config import DEFAULT_RUNTIME_CONFIG
from .qos import clock_qos, command_qos, latched_path_qos, sensor_data_qos, status_qos


def seconds_to_time(seconds: float) -> Time:
    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    return Time(sec=whole, nanosec=int((seconds - whole) * 1e9))


class SimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("simulator_node")
        self.declare_parameter("scenario", "obstacle")
        self.declare_parameter("steering_profile", "ideal")
        self.declare_parameter("physics_dt", DEFAULT_RUNTIME_CONFIG.physics_dt)
        self.declare_parameter("command_timeout", DEFAULT_RUNTIME_CONFIG.command_timeout)
        self.declare_parameter(
            "default_speed_limit_kmh", DEFAULT_RUNTIME_CONFIG.default_speed_limit_kmh
        )
        self.declare_parameter(
            "straight_speed_limit_kmh", DEFAULT_RUNTIME_CONFIG.straight_speed_limit_kmh
        )
        self.declare_parameter(
            "curve_speed_limit_kmh", DEFAULT_RUNTIME_CONFIG.curve_speed_limit_kmh
        )
        self.declare_parameter(
            "intersection_speed_limit_kmh",
            DEFAULT_RUNTIME_CONFIG.intersection_speed_limit_kmh,
        )
        self.declare_parameter(
            "lane_change_speed_limit_kmh",
            DEFAULT_RUNTIME_CONFIG.lane_change_speed_limit_kmh,
        )
        self.declare_parameter(
            "parking_speed_limit_kmh", DEFAULT_RUNTIME_CONFIG.parking_speed_limit_kmh
        )
        self.declare_parameter(
            "target_speed_ratio", DEFAULT_RUNTIME_CONFIG.target_speed_ratio
        )
        self.declare_parameter(
            "max_lateral_accel_mps2", DEFAULT_RUNTIME_CONFIG.max_lateral_accel_mps2
        )
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("vehicle_a_m", 1.015)
        self.declare_parameter("vehicle_b_m", 1.895)
        self.declare_parameter("vehicle_mass_kg", 1412.0)
        self.declare_parameter("vehicle_cf_n_per_rad", -148970.0)
        self.declare_parameter("vehicle_cr_n_per_rad", -82204.0)
        self.declare_parameter("vehicle_iz_kgm2", 1537.0)
        self.declare_parameter("vehicle_max_steer_rad", 0.5)
        self.declare_parameter("vehicle_max_accel_mps2", 3.0)
        self.declare_parameter("vehicle_max_decel_mps2", 6.0)
        self.declare_parameter("vehicle_width_m", 2.0)
        self.declare_parameter("vehicle_body_overhang_m", 1.0)
        self.declare_parameter("steering_time_constant_s", 0.15)
        self.declare_parameter("steering_delay_s", 0.05)
        self.declare_parameter("steering_rate_limit_rad_s", 0.6)

        scenario_name = str(self.get_parameter("scenario").value)
        self.physics_dt = float(self.get_parameter("physics_dt").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.publish_clock_enabled = bool(self.get_parameter("publish_clock").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        config = make_scenario(scenario_name)
        config.physics_dt = self.physics_dt
        self._apply_runtime_parameters(config)
        self.engine = SimulationEngine(config)
        self.command = ControlCommand()
        self.last_command_time = self.get_clock().now()
        self.paused = False

        sensor_qos = sensor_data_qos()
        self.state_pub = self.create_publisher(RosVehicleState, "vehicle/state", sensor_qos)
        self.obstacle_pub = self.create_publisher(ObstacleArray, "obstacles", sensor_qos)
        self.reference_pub = self.create_publisher(
            RosPath, "reference_path", latched_path_qos()
        )
        self.status_pub = self.create_publisher(
            SimulationStatus, "sim/status", status_qos()
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", clock_qos())
        self.command_sub = self.create_subscription(
            RosControlCommand, "control_command", self._on_command, command_qos()
        )
        self.reset_srv = self.create_service(Empty, "sim/reset", self._on_reset)
        self.pause_srv = self.create_service(SetBool, "sim/pause", self._on_pause)
        self.step_srv = self.create_service(Trigger, "sim/step", self._on_step)
        self.timer = self.create_timer(self.physics_dt, self._on_timer)
        self._publish_reference(sequence=0)
        self._publish_status()
        self.get_logger().info(
            f"simulator ready: scenario={scenario_name} dt={self.physics_dt:.3f}"
        )

    def _apply_runtime_parameters(self, config) -> None:
        speed_limits = {
            "default": float(self.get_parameter("default_speed_limit_kmh").value),
            "straight": float(self.get_parameter("straight_speed_limit_kmh").value),
            "curve": float(self.get_parameter("curve_speed_limit_kmh").value),
            "intersection": float(
                self.get_parameter("intersection_speed_limit_kmh").value
            ),
            "lane_change": float(
                self.get_parameter("lane_change_speed_limit_kmh").value
            ),
            "parking": float(self.get_parameter("parking_speed_limit_kmh").value),
        }
        speed_type = str(getattr(config, "speed_limit_type", "default"))
        if speed_type not in speed_limits:
            raise ValueError(f"unsupported speed_limit_type: {speed_type}")
        target_ratio = float(self.get_parameter("target_speed_ratio").value)
        if not math.isfinite(target_ratio) or not 0.0 < target_ratio <= 1.0:
            raise ValueError("target_speed_ratio must be in the range (0, 1]")
        speed_limit = speed_limits[speed_type]
        if not math.isfinite(speed_limit) or speed_limit <= 0.0:
            raise ValueError(f"invalid {speed_type} speed limit")
        config.speed_limit_kmh = speed_limit
        config.target_speed_ratio = target_ratio
        config.speed_limits = speed_limits
        config.max_lateral_accel_mps2 = float(
            self.get_parameter("max_lateral_accel_mps2").value
        )
        if not math.isfinite(config.max_lateral_accel_mps2) or config.max_lateral_accel_mps2 <= 0.0:
            raise ValueError("max_lateral_accel_mps2 must be positive")
        config.target_speed = speed_limit * target_ratio
        profile = steering_profile(str(self.get_parameter("steering_profile").value))
        config.vehicle_params = VehicleParams(
            a=float(self.get_parameter("vehicle_a_m").value),
            b=float(self.get_parameter("vehicle_b_m").value),
            m=float(self.get_parameter("vehicle_mass_kg").value),
            Cf=float(self.get_parameter("vehicle_cf_n_per_rad").value),
            Cr=float(self.get_parameter("vehicle_cr_n_per_rad").value),
            Iz=float(self.get_parameter("vehicle_iz_kgm2").value),
            max_steer=float(self.get_parameter("vehicle_max_steer_rad").value),
            max_accel=float(self.get_parameter("vehicle_max_accel_mps2").value),
            max_decel=float(self.get_parameter("vehicle_max_decel_mps2").value),
            width=float(self.get_parameter("vehicle_width_m").value),
            body_overhang=float(
                self.get_parameter("vehicle_body_overhang_m").value
            ),
        )
        config.steering = SteeringParams(
            mode=profile.mode,
            time_constant_s=float(
                self.get_parameter("steering_time_constant_s").value
            ),
            delay_s=float(self.get_parameter("steering_delay_s").value),
            rate_limit_rad_s=float(
                self.get_parameter("steering_rate_limit_rad_s").value
            ),
            source=profile.source,
        )
        config.steering.delay_steps(self.physics_dt)

    def _on_command(self, message: RosControlCommand) -> None:
        self.command = ControlCommand(
            steer=float(message.steering_angle),
            throttle=float(message.throttle),
            brake=float(message.brake),
            gear=int(message.gear),
        )
        self.last_command_time = self.get_clock().now()

    def _command_is_stale(self) -> bool:
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        return age > self.command_timeout

    def _advance_once(self) -> None:
        command = (
            ControlCommand(brake=1.0, gear=0)
            if self._command_is_stale()
            else self.command
        )
        self.engine.step(command, dt=self.physics_dt)
        self._publish_state()
        if self.engine.is_done:
            self.paused = True
            self.get_logger().warning(
                "simulation stopped: "
                f"collision={self.engine.collision_occurred} "
                f"offroad={self.engine.offroad_occurred} "
                f"reached={self.engine.reached_destination}"
            )
        self._publish_status()

    def _on_timer(self) -> None:
        if not self.paused:
            self._advance_once()

    def _on_step(self, _request, response):
        self._advance_once()
        response.success = True
        response.message = f"step={self.engine.step_count} sim_time={self.engine.sim_time:.3f}"
        return response

    def _on_reset(self, _request, response):
        self.engine.reset()
        self.command = ControlCommand()
        self.last_command_time = self.get_clock().now()
        self.paused = False
        self._publish_reference(sequence=0)
        self._publish_state()
        self._publish_status()
        return response

    def _on_pause(self, request, response):
        self.paused = bool(request.data)
        self._publish_status()
        response.success = True
        response.message = "paused" if self.paused else "running"
        return response

    def _publish_reference(self, sequence: int) -> None:
        message = RosPath()
        message.header.stamp = seconds_to_time(self.engine.sim_time)
        message.header.frame_id = self.frame_id
        message.sequence = sequence
        message.points = [
            PathPoint(x=p[0], y=p[1], theta=p[2], kappa=p[3])
            for p in self.engine.world.ref_path_as_tuples
        ]
        self.reference_pub.publish(message)

    def _publish_state(self) -> None:
        stamp = seconds_to_time(self.engine.sim_time)
        state = self.engine.get_state()
        message = RosVehicleState()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.x = state.x
        message.y = state.y
        message.yaw = state.phi
        message.vx = state.vx
        message.vy = state.vy
        message.yaw_rate = state.r
        message.steering_angle = state.steer
        message.acceleration = state.accel
        self.state_pub.publish(message)

        obstacles = ObstacleArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = self.frame_id
        obstacles.obstacles = [
            RosObstacle(
                id=o.id,
                x=o.x,
                y=o.y,
                length=o.length,
                width=o.width,
                speed=o.speed,
                heading=o.heading,
                type=o.type,
            )
            for o in self.engine.obstacles.get_all()
        ]
        self.obstacle_pub.publish(obstacles)

        if self.publish_clock_enabled:
            clock = Clock()
            clock.clock = stamp
            self.clock_pub.publish(clock)

    def _termination_reason(self) -> str:
        if self.engine.collision_occurred:
            return "collision"
        if self.engine.offroad_occurred:
            return "offroad"
        if self.engine.reached_destination:
            return "reached"
        if self.paused:
            return "paused"
        return ""

    def _publish_status(self, stamp: Optional[Time] = None) -> None:
        message = SimulationStatus()
        message.header.stamp = stamp or seconds_to_time(self.engine.sim_time)
        message.header.frame_id = self.frame_id
        message.running = not self.paused and not self.engine.is_done
        message.paused = self.paused
        message.done = self.engine.is_done
        message.collision = self.engine.collision_occurred
        message.offroad = self.engine.offroad_occurred
        message.reached = self.engine.reached_destination
        message.step_count = self.engine.step_count
        message.sim_time = self.engine.sim_time
        message.scenario = self.engine.config.name
        message.termination_reason = self._termination_reason()
        self.status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[SimulatorNode] = None
    try:
        node = SimulatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass
