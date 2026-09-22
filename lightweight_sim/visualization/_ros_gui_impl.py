"""Pygame view and view-model types for the ROS 2 simulator client."""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import pygame

from ..simulator.data_types import Obstacle, PathPoint, VehicleState
from .colors import GRID, HUD_TEXT, HUD_WARNING, LANE_DASH, ROAD_EDGE, ROAD_SURFACE
from .hud import HUD
from .pygame_compat import configure_display_driver, patch_sysfont_for_python314
from .renderer import Camera, Renderer


REFERENCE_PATH = (120, 155, 175)
REFERENCE_LINE_PATH = (255, 210, 0)
ROUTING_PATH = (255, 165, 0)
LOCAL_PLANNED_PATH = (0, 220, 100)
LANE_BOUNDARY = (190, 190, 190)
DRIVABLE_BOUNDARY = (245, 245, 245)


def road_strip_polygons(path: List[PathPoint], lane_width: float, num_lanes: int):
    """Build independent road strips so self-crossing paths stay drawable."""

    half_width = num_lanes * lane_width / 2.0
    for first, second in zip(path[:-1], path[1:]):
        first_normal = (-math.sin(first.theta), math.cos(first.theta))
        second_normal = (-math.sin(second.theta), math.cos(second.theta))
        yield (
            (
                first.x - half_width * first_normal[0],
                first.y - half_width * first_normal[1],
            ),
            (
                second.x - half_width * second_normal[0],
                second.y - half_width * second_normal[1],
            ),
            (
                second.x + half_width * second_normal[0],
                second.y + half_width * second_normal[1],
            ),
            (
                first.x + half_width * first_normal[0],
                first.y + half_width * first_normal[1],
            ),
        )


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
    reference_line_path: List[PathPoint] = field(default_factory=list)
    routing_path: List[PathPoint] = field(default_factory=list)
    routing_request_id: int = 0
    routing_left_boundary: List[PathPoint] = field(default_factory=list)
    routing_right_boundary: List[PathPoint] = field(default_factory=list)
    drivable_left_boundary: List[PathPoint] = field(default_factory=list)
    drivable_right_boundary: List[PathPoint] = field(default_factory=list)
    planned_path: List[Tuple[float, float, float, float]] = field(default_factory=list)
    status: GuiStatus = field(default_factory=GuiStatus)
    control: GuiControl = field(default_factory=GuiControl)
    mode: str = "CRUISE"
    control_source: str = "AUTO"


@dataclass(frozen=True)
class GuiAction:
    kind: str
    value: Optional[object] = None


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
        self._history_scenario = ""

    @staticmethod
    def _tracking_error(snapshot: GuiSnapshot) -> Tuple[float, float]:
        """Return lateral and wrapped heading error for the active path.

        At a figure-eight crossing, several reference points can be equally
        close.  Prefer a nearby point whose tangent agrees with the vehicle
        heading so the error does not jump to the opposite branch.
        """
        state = snapshot.state
        path = snapshot.reference_line_path or snapshot.reference_path
        if state is None or not path:
            return 0.0, 0.0

        distances = [
            (point.x - state.x) ** 2 + (point.y - state.y) ** 2
            for point in path
        ]
        nearest_distance = min(distances)
        candidates = [
            index
            for index, distance in enumerate(distances)
            if distance <= nearest_distance + 9.0
        ]

        def heading_error(index: int) -> float:
            return math.atan2(
                math.sin(state.phi - path[index].theta),
                math.cos(state.phi - path[index].theta),
            )

        index = min(
            candidates,
            key=lambda item: abs(heading_error(item))
            + 0.05 * math.sqrt(distances[item]),
        )
        reference = path[index]
        dx = state.x - reference.x
        dy = state.y - reference.y
        ed = -math.sin(reference.theta) * dx + math.cos(reference.theta) * dy
        return ed, heading_error(index)

    def poll_actions(self) -> List[GuiAction]:
        actions = []
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                actions.append(GuiAction("quit"))
            elif event.type == pygame.KEYDOWN:
                actions.append(GuiAction("key_down", event.key))
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
            elif event.type == pygame.KEYUP:
                actions.append(GuiAction("key_up", event.key))
            elif event.type == pygame.MOUSEWHEEL:
                self.camera.zoom(0.1 if event.y > 0 else -0.1)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                actions.append(GuiAction("mouse_click", event.pos))
        return actions

    def render(self, snapshot: GuiSnapshot) -> None:
        if snapshot.state is not None:
            self.camera.follow(snapshot.state.x, snapshot.state.y, smooth=0.2)

        self.renderer.clear()
        self.renderer.draw_grid()
        active_reference = snapshot.reference_line_path or snapshot.reference_path
        if active_reference:
            world = _WorldView(
                active_reference,
                self.lane_width,
                self.num_lanes,
            )
            self._draw_road(world)
            self.renderer.draw_path(
                [(p.x, p.y, p.theta, p.kappa) for p in active_reference],
                color=(
                    REFERENCE_LINE_PATH
                    if snapshot.reference_line_path
                    else REFERENCE_PATH
                ),
                width=3 if snapshot.reference_line_path else 1,
                dashed=not snapshot.reference_line_path,
            )
        if snapshot.routing_path or snapshot.reference_line_path:
            for boundary in (
                snapshot.drivable_left_boundary,
                snapshot.drivable_right_boundary,
            ):
                self.renderer.draw_path(
                    [(p.x, p.y, p.theta, p.kappa) for p in boundary],
                    color=DRIVABLE_BOUNDARY,
                    width=2,
                )
            for boundary in (
                snapshot.routing_left_boundary,
                snapshot.routing_right_boundary,
            ):
                self.renderer.draw_path(
                    [(p.x, p.y, p.theta, p.kappa) for p in boundary],
                    color=LANE_BOUNDARY,
                    width=1,
                    dashed=True,
                )
            self.renderer.draw_path(
                [(p.x, p.y, p.theta, p.kappa) for p in snapshot.routing_path],
                color=ROUTING_PATH,
                width=1,
                dashed=True,
            )
        if snapshot.planned_path:
            self.renderer.draw_path(
                snapshot.planned_path,
                color=LOCAL_PLANNED_PATH,
                width=2,
            )
        self.renderer.draw_obstacles(snapshot.obstacles)

        if snapshot.state is None:
            self._draw_text("Waiting for /vehicle/state ...", HUD_WARNING)
        else:
            if snapshot.status.scenario != self._history_scenario:
                self.hud.ed_history.clear()
                self.hud.ephi_history.clear()
                self._history_scenario = snapshot.status.scenario
            ed, ephi = self._tracking_error(snapshot)
            self.hud.update_history(ed, ephi)
            self.renderer.draw_vehicle(snapshot.state)
            self.hud.render(
                state=snapshot.state,
                target_speed=self.target_speed_kmh,
                control_info={
                    "steer": snapshot.control.steering_angle,
                    "throttle": snapshot.control.throttle,
                    "brake": snapshot.control.brake,
                },
                auto_mode=snapshot.control_source.upper() == "AUTO",
                fps=self.clock.get_fps(),
                sim_time=snapshot.status.sim_time,
                real_time=time.monotonic() - self.started_at,
                collision=snapshot.status.collision,
                map_name=f"{snapshot.status.scenario} [ROS 2]",
                ed=ed,
                ephi=ephi,
            )
        self._draw_status(snapshot.status)
        self._draw_mode_controls(snapshot)
        pygame.display.flip()
        self.clock.tick(60)

    def _draw_road(self, world: "_WorldView") -> None:
        """Draw a road as local strips; whole-path polygons fail at crossings."""

        path = world.ref_path
        if len(path) < 2:
            return
        screen = self.renderer.screen
        camera = self.renderer.camera
        for strip in road_strip_polygons(path, world.lane_width, world.num_lanes):
            pygame.draw.polygon(
                screen,
                ROAD_SURFACE,
                [camera.world_to_screen(x, y) for x, y in strip],
            )

        screen_path = [camera.world_to_screen(point.x, point.y) for point in path]
        half_width = world.num_lanes * world.lane_width / 2.0
        for lane_index in range(world.num_lanes + 1):
            offset = -half_width + lane_index * world.lane_width
            if abs(offset) < 0.05:
                self.renderer._draw_offset_line(
                    screen_path, path, offset, LANE_DASH, 1, dashed=True
                )
            elif lane_index in (0, world.num_lanes):
                self.renderer._draw_offset_line(
                    screen_path, path, offset, ROAD_EDGE, 2
                )
            else:
                self.renderer._draw_offset_line(
                    screen_path, path, offset, LANE_DASH, 1, dashed=True
                )
        self.renderer._draw_dashed_line(screen_path, GRID, 1)

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

    def _draw_mode_controls(self, snapshot: GuiSnapshot) -> None:
        """Hook for concrete views to draw mode controls."""

        return None

    def close(self) -> None:
        pygame.quit()


class _WorldView:
    """Renderer-compatible road view without depending on SimulationEngine."""

    def __init__(self, ref_path, lane_width: float, num_lanes: int) -> None:
        self.ref_path = ref_path
        self.lane_width = lane_width
        self.num_lanes = num_lanes
