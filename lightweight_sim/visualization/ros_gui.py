"""Pygame view and view-model types for the ROS 2 simulator client."""

import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import pygame

from ..simulator.data_types import Obstacle, PathPoint, VehicleState
from .colors import HUD_TEXT, HUD_WARNING
from .hud import HUD
from .pygame_compat import configure_display_driver, patch_sysfont_for_python314
from .renderer import Camera, Renderer


@dataclass
class GuiStatus:
    running: bool = False
    paused: bool = False
    done: bool = False
    collision: bool = False
    offroad: bool = False
    reached: bool = False
    step_count: int = 0
    sim_time: float = 0.0
    scenario: str = "ROS 2"
    termination_reason: str = ""


@dataclass
class GuiControl:
    steering_angle: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0


@dataclass
class GuiSnapshot:
    state: Optional[VehicleState] = None
    obstacles: List[Obstacle] = field(default_factory=list)
    reference_path: List[PathPoint] = field(default_factory=list)
    planned_path: List[Tuple[float, float, float, float]] = field(default_factory=list)
    status: GuiStatus = field(default_factory=GuiStatus)
    control: GuiControl = field(default_factory=GuiControl)


@dataclass(frozen=True)
class GuiAction:
    kind: str
    value: Optional[bool] = None


class RosGuiView:
    """Pure GUI adapter; it knows nothing about ROS publishers or services."""

    def __init__(
        self,
        width: int = 1200,
        height: int = 800,
        lane_width: float = 3.5,
        num_lanes: int = 2,
        target_speed_kmh: float = 40.0,
    ) -> None:
        configure_display_driver()
        patch_sysfont_for_python314()
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Lightweight ROS 2 Simulator")
        self.clock = pygame.time.Clock()
        self.camera = Camera(width, height)
        self.renderer = Renderer(self.screen, self.camera)
        self.hud = HUD(self.screen)
        self.lane_width = lane_width
        self.num_lanes = num_lanes
        self.target_speed_kmh = target_speed_kmh
        self.started_at = time.monotonic()

    def poll_actions(self) -> List[GuiAction]:
        actions = []
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                actions.append(GuiAction("quit"))
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    actions.append(GuiAction("quit"))
                elif event.key == pygame.K_r:
                    actions.append(GuiAction("reset"))
                elif event.key == pygame.K_p:
                    actions.append(GuiAction("toggle_pause"))
                elif event.key == pygame.K_n:
                    actions.append(GuiAction("step"))
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    self.camera.zoom(0.1)
                elif event.key == pygame.K_MINUS:
                    self.camera.zoom(-0.1)
            elif event.type == pygame.MOUSEWHEEL:
                self.camera.zoom(0.1 if event.y > 0 else -0.1)
        return actions

    def render(self, snapshot: GuiSnapshot) -> None:
        if snapshot.state is not None:
            self.camera.follow(snapshot.state.x, snapshot.state.y, smooth=0.2)

        self.renderer.clear()
        self.renderer.draw_grid()
        if snapshot.reference_path:
            world = _WorldView(
                snapshot.reference_path,
                self.lane_width,
                self.num_lanes,
            )
            self.renderer.draw_road(world)
            self.renderer.draw_path(
                [(p.x, p.y, p.theta, p.kappa) for p in snapshot.reference_path],
                dashed=True,
            )
        if snapshot.planned_path:
            self.renderer.draw_path(snapshot.planned_path)
        self.renderer.draw_obstacles(snapshot.obstacles)

        if snapshot.state is None:
            self._draw_text("Waiting for /vehicle/state ...", HUD_WARNING)
        else:
            self.renderer.draw_vehicle(snapshot.state)
            self.hud.render(
                state=snapshot.state,
                target_speed=self.target_speed_kmh,
                control_info={
                    "steer": snapshot.control.steering_angle,
                    "throttle": snapshot.control.throttle,
                    "brake": snapshot.control.brake,
                },
                auto_mode=True,
                fps=self.clock.get_fps(),
                sim_time=snapshot.status.sim_time,
                real_time=time.monotonic() - self.started_at,
                collision=snapshot.status.collision,
                map_name=f"{snapshot.status.scenario} [ROS 2]",
            )
        self._draw_status(snapshot.status)
        pygame.display.flip()
        self.clock.tick(60)

    def _draw_text(self, text: str, color) -> None:
        self.screen.blit(self.renderer.font_small.render(text, True, color), (12, 12))

    def _draw_status(self, status: GuiStatus) -> None:
        if status.done:
            text = f"ROS 2: DONE ({status.termination_reason})  [R reset]"
            color = HUD_WARNING
        elif status.paused:
            text = "ROS 2: PAUSED  [P resume] [N step]"
            color = HUD_WARNING
        elif status.running:
            text = "ROS 2: RUNNING  [P pause] [R reset]"
            color = HUD_TEXT
        else:
            text = "ROS 2: waiting for /sim/status"
            color = HUD_WARNING
        self._draw_text(text, color)

    def close(self) -> None:
        pygame.quit()


class _WorldView:
    """Renderer-compatible road view without depending on SimulationEngine."""

    def __init__(self, ref_path, lane_width: float, num_lanes: int) -> None:
        self.ref_path = ref_path
        self.lane_width = lane_width
        self.num_lanes = num_lanes
