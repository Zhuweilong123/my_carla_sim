"""Fixed-step simulation engine with diagnostic logging."""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Optional

from .data_types import ControlCommand, ScenarioConfig, VehicleParams, VehicleState
from .logging_utils import get_run_logger
from .obstacle import ObstacleManager
from .vehicle import EgoVehicle
from .world import World
from .steering import SteeringActuator


class SimulationEngine:
    """Own the world state and advance it with a deterministic physics step."""

    def __init__(self, config: ScenarioConfig):
        self.logger = get_run_logger()
        self.config = config
        self.world = World(config.road)
        self.ego = EgoVehicle(
            VehicleState(
                x=config.ego_start_x,
                y=config.ego_start_y,
                phi=config.ego_start_phi,
                vx=config.ego_start_speed,
            ),
            replace(config.vehicle_params),
        )
        self.steering = SteeringActuator(config.steering, self.ego.params.max_steer)
        self.obstacles = ObstacleManager()
        self.obstacles.add_from_config(config.obstacles)
        self.sim_time = 0.0
        self.step_count = 0
        self.physics_dt = 0.05
        self.vehicle_model = getattr(config, "vehicle_model", "kinematic")
        self.dynamic_max_substep_s = float(config.dynamic_max_substep_s)
        if not math.isfinite(self.dynamic_max_substep_s) or self.dynamic_max_substep_s <= 0.0:
            raise ValueError("dynamic_max_substep_s must be positive and finite")
        self.target_speed = config.target_speed
        self.destination = config.destination
        self.collision_occurred = False
        self.reached_destination = False
        self.offroad_occurred = False
        self._collision_ids = []
        self._last_collision = False
        self._last_offroad = False
        self._last_reached = False
        self.logger.info(
            "engine initialized; scenario=%s model=%s path_points=%d obstacles=%d initial=(%.3f,%.3f) speed=%.3f destination=%s",
            config.name,
            self.vehicle_model,
            len(self.world.ref_path),
            len(self.obstacles.get_all()),
            config.ego_start_x,
            config.ego_start_y,
            config.ego_start_speed,
            config.destination,
        )

    def reset(self):
        """Reset the complete simulation state using the original configuration."""

        self.logger.info("engine reset requested; scenario=%s", self.config.name)
        self.__init__(self.config)

    def step(
        self,
        control: ControlCommand | None = None,
        dt: Optional[float] = None,
    ) -> VehicleState:
        """Advance the simulation and log periodic state plus terminal events."""

        dt = self.physics_dt if dt is None else float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            self.logger.error("invalid simulation dt=%r", dt)
            raise ValueError("dt must be a positive finite number")

        control = control or ControlCommand()
        params = self.ego.params
        raw_steer = float(control.steer)
        raw_throttle = float(control.throttle)
        raw_brake = float(control.brake)
        if not all(math.isfinite(v) for v in (raw_steer, raw_throttle, raw_brake)):
            raise ValueError("control commands must be finite")
        steer = max(-params.max_steer, min(params.max_steer, raw_steer))
        throttle = max(0.0, min(1.0, raw_throttle))
        brake = max(0.0, min(1.0, raw_brake))
        if (steer, throttle, brake) != (raw_steer, raw_throttle, raw_brake):
            self.logger.debug(
                "control clamped; step=%d raw=(%.4f,%.4f,%.4f) applied=(%.4f,%.4f,%.4f)",
                self.step_count,
                raw_steer,
                raw_throttle,
                raw_brake,
                steer,
                throttle,
                brake,
            )

        accel = throttle * params.max_accel - brake * params.max_decel
        self.steering.begin_period(raw_steer, dt)
        substeps = max(1, int(math.ceil(self.ego.get_state().speed * dt / 0.5)))
        if self.vehicle_model == "dynamic":
            substeps = max(
                substeps,
                int(math.ceil(dt / self.dynamic_max_substep_s)),
            )
        subdt = dt / substeps
        state = self.ego.get_state()

        try:
            for _ in range(substeps):
                steer = self.steering.advance(subdt)
                state = self.ego.step(steer, accel, subdt, self.vehicle_model)
                self.obstacles.step(subdt)
                collision_hit = self.obstacles.check_collision(
                    state.x,
                    state.y,
                    self.ego.length,
                    self.ego.width,
                    state.phi,
                )
                if collision_hit:
                    self._collision_ids = list(self.obstacles.last_collision_ids)
                self.collision_occurred = self.collision_occurred or collision_hit
        except Exception:
            self.logger.exception(
                "physics step failed; step=%d sim_time=%.3f model=%s",
                self.step_count,
                self.sim_time,
                self.vehicle_model,
            )
            raise

        self.offroad_occurred = not self.world.is_on_road(
            state.x, state.y, -self.ego.width / 2.0
        )
        if self.destination:
            distance = math.hypot(
                state.x - self.destination[0], state.y - self.destination[1]
            )
            if distance < 2.0:
                self.reached_destination = True
        else:
            distance = None

        self.sim_time += dt
        self.step_count += 1
        if self.collision_occurred and not self._last_collision:
            self.logger.warning(
                "collision detected; step=%d sim_time=%.3f position=(%.3f,%.3f) obstacle_ids=%s obstacle_positions=%s",
                self.step_count,
                self.sim_time,
                state.x,
                state.y,
                self._collision_ids,
                [
                    (o.id, round(o.x, 3), round(o.y, 3))
                    for o in self.obstacles.get_all()
                    if o.id in self._collision_ids
                ],
            )
        if self.offroad_occurred and not self._last_offroad:
            self.logger.warning(
                "vehicle left road; step=%d position=(%.3f,%.3f) road_distance=%.3f road_limit=%.3f",
                self.step_count,
                state.x,
                state.y,
                self.world.distance_to_reference(state.x, state.y),
                self.world.num_lanes * self.world.lane_width / 2.0 - self.ego.width / 2.0,
            )
        if self.reached_destination and not self._last_reached:
            self.logger.info(
                "destination reached; step=%d distance=%.3f",
                self.step_count,
                distance if distance is not None else 0.0,
            )
        self._last_collision = self.collision_occurred
        self._last_offroad = self.offroad_occurred
        self._last_reached = self.reached_destination

        if self.step_count == 1 or self.step_count % 100 == 0:
            self.logger.debug(
                "state; step=%d sim_time=%.3f position=(%.3f,%.3f) speed=%.3f steer=%.4f accel=%.3f",
                self.step_count,
                self.sim_time,
                state.x,
                state.y,
                state.speed,
                steer,
                accel,
            )
        return state

    def get_state(self) -> VehicleState:
        return self.ego.get_state()

    def get_error_state(self, ts: float = 0.1):
        path = self.world.ref_path_as_tuples
        return self.ego.get_error_state(path, ts) if path else None

    @property
    def is_done(self) -> bool:
        return self.collision_occurred or self.reached_destination or self.offroad_occurred
