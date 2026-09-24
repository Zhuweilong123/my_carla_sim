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


REFERENCE_LINE_PATH = (35, 195, 255)
ROUTING_PATH = (220, 90, 255)
ROUTING_LANE_FILL = (165, 60, 220, 48)
ROUTING_LANE_EDGE = (220, 120, 255)
LOCAL_PLANNED_PATH = (0, 220, 100)
LANE_BOUNDARY = (190, 190, 190)
DRIVABLE_BOUNDARY = (245, 245, 245)


def road_strip_polygons(
    path: List[PathPoint],
    lane_width: float,
    num_lanes: int,
    reference_lane_index: int = -1,
):
    """Build road strips around the active reference lane."""

    if 0 <= reference_lane_index < num_lanes:
        right_offset = -(reference_lane_index + 0.5) * lane_width
        left_offset = (num_lanes - reference_lane_index - 0.5) * lane_width
    else:
        half_width = num_lanes * lane_width / 2.0
        right_offset = -half_width
        left_offset = half_width
    for first, second in zip(path[:-1], path[1:]):
        first_normal = (-math.sin(first.theta), math.cos(first.theta))
        second_normal = (-math.sin(second.theta), math.cos(second.theta))
        yield (
            (
                first.x + right_offset * first_normal[0],
                first.y + right_offset * first_normal[1],
            ),
            (
                second.x + right_offset * second_normal[0],
                second.y + right_offset * second_normal[1],
            ),
            (
                second.x + left_offset * second_normal[0],
                second.y + left_offset * second_normal[1],
            ),
            (
                first.x + left_offset * first_normal[0],
                first.y + left_offset * first_normal[1],
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
    road_network: List[List[Tuple[float, float]]] = field(default_factory=list)
    road_network_num_lanes: int = 0
    obstacles: List[Obstacle] = field(default_factory=list)
    reference_line_path: List[PathPoint] = field(default_factory=list)
    reference_line_request_id: int = 0
    reference_lane_index: int = -1
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
        render_fps: int = 60,
    ) -> None:
        configure_display_driver()
        patch_sysfont_for_python314()
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((width, height))
        self._routing_overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        pygame.display.set_caption("Lightweight ROS 2 Simulator")
        self.clock = pygame.time.Clock()
        self.camera = Camera(width, height)
        self.renderer = Renderer(self.screen, self.camera)
        self.hud = HUD(self.screen)
        self.lane_width = lane_width
        self.num_lanes = num_lanes
        self.target_speed_kmh = target_speed_kmh
        self.render_fps = int(render_fps)
        self.started_at = time.monotonic()
        self._history_scenario = ""
        self._camera_scenario = ""

    @staticmethod
    def _tracking_error(snapshot: GuiSnapshot) -> Tuple[float, float]:
        """Return lateral and wrapped heading error for the active path.

        At a figure-eight crossing, several reference points can be equally
        close.  Prefer a nearby point whose tangent agrees with the vehicle
        heading so the error does not jump to the opposite branch.
        """
        state = snapshot.state
        path = snapshot.reference_line_path
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
            if snapshot.status.scenario != self._camera_scenario:
                self.camera.cx = snapshot.state.x
                self.camera.cy = snapshot.state.y
                self._camera_scenario = snapshot.status.scenario
            else:
                self.camera.follow(snapshot.state.x, snapshot.state.y, smooth=0.2)

        self.renderer.clear()
        self.renderer.draw_grid()
        if snapshot.road_network:
            self._draw_road_network(snapshot)
        active_reference = snapshot.reference_line_path
        has_routing_lane = (
            len(snapshot.routing_left_boundary) >= 2
            and len(snapshot.routing_right_boundary) >= 2
        )
        if active_reference:
            world = _WorldView(
                active_reference,
                self.lane_width,
                self.num_lanes,
                snapshot.reference_lane_index,
            )
            if not snapshot.road_network:
                self._draw_road(world)
            if has_routing_lane:
                self._draw_routing_lane(snapshot)
            self.renderer.draw_path(
                [(p.x, p.y, p.theta, p.kappa) for p in active_reference],
                color=REFERENCE_LINE_PATH,
                width=3,
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
                    color=ROUTING_LANE_EDGE if has_routing_lane else LANE_BOUNDARY,
                    width=2 if has_routing_lane else 1,
                    dashed=True,
                )
            if has_routing_lane:
                self._draw_routing_directions(snapshot.routing_path)
            else:
                self.renderer.draw_path(
                    [(p.x, p.y, p.theta, p.kappa) for p in snapshot.routing_path],
                    color=ROUTING_PATH,
                    width=2,
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
        self._draw_legend(has_routing_lane, bool(snapshot.reference_line_path), bool(snapshot.planned_path))
        pygame.display.flip()
        self.clock.tick(self.render_fps)

    def _draw_road_network(self, snapshot: GuiSnapshot) -> None:
        lane_count = snapshot.road_network_num_lanes or self.num_lanes
        lane_width = self.lane_width
        road_paths = []
        for polyline in snapshot.road_network:
            if len(polyline) < 2:
                continue
            path = []
            for index, (x, y) in enumerate(polyline):
                previous = polyline[max(0, index - 1)]
                following = polyline[min(len(polyline) - 1, index + 1)]
                theta = math.atan2(following[1] - previous[1], following[0] - previous[0])
                path.append(PathPoint(float(x), float(y), theta, 0.0))
            road_paths.append(path)

        for path in road_paths:
            for strip in road_strip_polygons(path, lane_width, lane_count):
                pygame.draw.polygon(
                    self.screen,
                    ROAD_SURFACE,
                    [self.camera.world_to_screen(x, y) for x, y in strip],
                )

        for path in road_paths:
            screen_path = [self.camera.world_to_screen(point.x, point.y) for point in path]
            half_width = lane_count * lane_width / 2.0
            for lane_index in range(lane_count + 1):
                offset = -half_width + lane_index * lane_width
                if lane_index in (0, lane_count):
                    self.renderer._draw_offset_line(screen_path, path, offset, ROAD_EDGE, 2)
                elif abs(offset) > 0.05:
                    self.renderer._draw_offset_line(screen_path, path, offset, LANE_DASH, 1, dashed=True)
            if lane_count % 2 == 0:
                self.renderer._draw_offset_line(screen_path, path, -0.12, (238, 190, 60), 2)
                self.renderer._draw_offset_line(screen_path, path, 0.12, (238, 190, 60), 2)

    def _draw_routing_lane(self, snapshot: GuiSnapshot) -> None:
        """Tint the selected routing lane using its map-derived boundaries."""
        left = snapshot.routing_left_boundary
        right = snapshot.routing_right_boundary
        if len(left) < 2 or len(left) != len(right):
            return
        camera = self.renderer.camera
        polygon = [camera.world_to_screen(point.x, point.y) for point in left]
        polygon.extend(
            camera.world_to_screen(point.x, point.y) for point in reversed(right)
        )
        if (
            not hasattr(self, "_routing_overlay")
            or self._routing_overlay.get_size() != self.screen.get_size()
        ):
            self._routing_overlay = pygame.Surface(
                self.screen.get_size(), pygame.SRCALPHA
            )
        self._routing_overlay.fill((0, 0, 0, 0))
        pygame.draw.polygon(self._routing_overlay, ROUTING_LANE_FILL, polygon)
        self.screen.blit(self._routing_overlay, (0, 0))

    def _draw_routing_directions(self, path: List[PathPoint]) -> None:
        """Mark route direction sparsely without drawing a second centerline."""
        next_marker_distance = 24.0
        travelled = 0.0
        previous = None
        for point in path:
            if previous is not None:
                travelled += math.hypot(point.x - previous.x, point.y - previous.y)
            if travelled >= next_marker_distance:
                center = self.renderer.camera.world_to_screen(point.x, point.y)
                tangent = (math.cos(point.theta), -math.sin(point.theta))
                normal = (-tangent[1], tangent[0])
                tip = (center[0] + 7 * tangent[0], center[1] + 7 * tangent[1])
                rear = (center[0] - 5 * tangent[0], center[1] - 5 * tangent[1])
                arrow = [
                    tip,
                    (rear[0] + 4 * normal[0], rear[1] + 4 * normal[1]),
                    (rear[0] - 4 * normal[0], rear[1] - 4 * normal[1]),
                ]
                pygame.draw.polygon(self.screen, ROUTING_PATH, arrow)
                next_marker_distance += 24.0
            previous = point

    def _draw_legend(self, has_routing_lane, has_reference_line, has_local_plan) -> None:
        if not (has_routing_lane or has_reference_line or has_local_plan):
            return
        width, height = 190, 72
        legend = pygame.Surface((width, height), pygame.SRCALPHA)
        pygame.draw.rect(legend, (8, 13, 20, 220), (0, 0, width, height), border_radius=6)
        pygame.draw.rect(legend, (85, 100, 118, 220), (0, 0, width, height), 1, border_radius=6)
        rows = (
            (ROUTING_PATH, "ROUTING LANE", has_routing_lane, True),
            (REFERENCE_LINE_PATH, "REFERENCE LINE", has_reference_line, False),
            (LOCAL_PLANNED_PATH, "LOCAL PLAN", has_local_plan, False),
        )
        for row, (color, label, visible, filled) in enumerate(rows):
            y = 14 + row * 21
            if filled:
                pygame.draw.rect(legend, (*color, 100), (10, y - 4, 20, 9), border_radius=2)
                pygame.draw.rect(legend, color, (10, y - 4, 20, 9), 1, border_radius=2)
            else:
                pygame.draw.line(legend, color, (10, y), (30, y), 3 if visible else 1)
            text_color = HUD_TEXT if visible else (105, 115, 128)
            legend.blit(self.renderer.font_small.render(label, True, text_color), (38, y - 8))
        self.screen.blit(legend, (12, self.screen.get_height() - height - 12))

    def _draw_road(self, world: "_WorldView") -> None:
        """Draw a road as local strips; whole-path polygons fail at crossings."""

        path = world.ref_path
        if len(path) < 2:
            return
        screen = self.renderer.screen
        camera = self.renderer.camera
        for strip in road_strip_polygons(
            path,
            world.lane_width,
            world.num_lanes,
            world.reference_lane_index,
        ):
            pygame.draw.polygon(
                screen,
                ROAD_SURFACE,
                [camera.world_to_screen(x, y) for x, y in strip],
            )

        screen_path = [camera.world_to_screen(point.x, point.y) for point in path]
        half_width = world.num_lanes * world.lane_width / 2.0
        for lane_index in range(world.num_lanes + 1):
            if 0 <= world.reference_lane_index < world.num_lanes:
                offset = (
                    lane_index - world.reference_lane_index - 0.5
                ) * world.lane_width
            else:
                offset = -half_width + lane_index * world.lane_width
            if lane_index in (0, world.num_lanes):
                self.renderer._draw_offset_line(
                    screen_path, path, offset, ROAD_EDGE, 2
                )
            else:
                self.renderer._draw_offset_line(
                    screen_path, path, offset, LANE_DASH, 1, dashed=True
                )

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

    def __init__(
        self,
        ref_path,
        lane_width: float,
        num_lanes: int,
        reference_lane_index: int = -1,
    ) -> None:
        self.ref_path = ref_path
        self.lane_width = lane_width
        self.num_lanes = num_lanes
        self.reference_lane_index = reference_lane_index
