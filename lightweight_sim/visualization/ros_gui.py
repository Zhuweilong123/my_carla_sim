"""Public ROS GUI module with scenario shortcuts and WSLg-safe window setup."""

import importlib
import sys
import time
from dataclasses import replace

import pygame
import math
from ..engine.analysis.tracking import TrackingMonitor

_simulator = importlib.import_module("lightweight_sim.engine.simulator")
sys.modules.setdefault("lightweight_sim.simulator", _simulator)
for _name in ("data_types", "obstacle", "world", "vehicle", "engine", "scenarios"):
    _module = importlib.import_module(f"lightweight_sim.engine.simulator.{_name}")
    sys.modules.setdefault(f"lightweight_sim.simulator.{_name}", _module)

from ._ros_gui_impl import *  # noqa: F401,F403,E402


_LegacyRosGuiView = RosGuiView
_LegacyHUD = HUD


class _ScenarioHUD(_LegacyHUD):
    def _draw_controls_hint(self, x, y, w, h, auto_mode):
        right_width = min(310, int(self.w * 0.255))
        right_edge = (
            self.w - 18 if getattr(self, "compact_editor", False)
            else self.w - right_width - 30
        )
        x = 220
        w = max(150, right_edge - x)
        y = self.h - 64
        h = 48
        panel = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(panel, (12, 18, 27, 215), panel.get_rect(), border_radius=7)
        pygame.draw.rect(panel, self.BORDER, panel.get_rect(), 1, border_radius=7)
        if auto_mode:
            hint = (
                "1-7 scene   F1-F3 slot   C cruise   K parking   E/P pause   R reset"
                if w >= 500 else "1-7 scene  F1-3 slot  C/K mode  E/P pause  R reset"
            )
        else:
            hint = (
                "W/S drive   A/D steer   SPACE brake   Q auto   E/P pause"
                if w >= 500 else "W/S drive  A/D steer  SPACE brake  E/P pause"
            )
        self.screen.blit(panel, (x, y))
        self._text(hint, x + 14, y + 8, self.font_small, self.MUTED)
        navigation = (
            "2-finger / MMB: pan   Ctrl+scroll / +/-: zoom   F: follow"
            if w >= 500 else "Scroll/MMB pan  Ctrl+scroll zoom  F follow"
        )
        self._text(navigation, x + 14, y + 28, self.font_small, self.MUTED)


HUD = _ScenarioHUD


class RosGuiView(_LegacyRosGuiView):
    """Use a real window and expose number-key scene selection."""

    SCENARIO_KEYS = (
        (pygame.K_1, "default"),
        (pygame.K_2, "obstacle"),
        (pygame.K_3, "three_lane"),
        (pygame.K_4, "curve"),
        (pygame.K_5, "figure_eight"),
        (pygame.K_6, "reverse_parking"),
        (pygame.K_7, "demo_grid"),
    )
    PARKING_SLOT_KEYS = ((pygame.K_F1, 1), (pygame.K_F2, 2), (pygame.K_F3, 3))

    MODE_BUTTONS = (
        # Keep labels ASCII-only: the default Pygame font on WSL often lacks
        # CJK glyphs and renders Chinese labels as indistinguishable squares.
        ("CRUISE", "CRUISE"),
        ("PARKING", "PARKING"),
        ("TOGGLE_PAUSE", "PAUSE"),
    )

    @staticmethod
    def _tracking_error(snapshot):
        state = snapshot.state
        if state is None:
            return 0.0, 0.0
        reverse_parking = (
            snapshot.status.scenario == "reverse_parking" and state.vx < -1e-3
        )
        measured = getattr(snapshot, "tracking_metrics", None)
        if measured and abs(state.timestamp-measured["timestamp"]) <= 0.25:
            reference_heading = measured["reference_heading_rad"]
            if reverse_parking:
                # The route projection tangent follows travel direction. During
                # reverse parking the body faces the opposite way, so compare
                # body yaw with the body-aligned reference heading.
                reference_heading += math.pi
            ephi = math.atan2(
                math.sin(state.phi - reference_heading),
                math.cos(state.phi - reference_heading),
            )
            return measured["ed_m"], ephi
        source = snapshot.planned_path or snapshot.reference_line_path
        if len(source) < 2:
            return 0.0, 0.0
        monitor = getattr(snapshot, "_tracking_monitor", None)
        if monitor is None or getattr(snapshot, "_measurement_path", None) is not source:
            try:
                monitor = TrackingMonitor([tuple(p) for p in source])
            except ValueError:
                return 0.0, 0.0
            if measured:
                monitor.tracker.s = monitor.tracker.geometry.project(
                    measured["reference_x_m"], measured["reference_y_m"],
                    measured["reference_heading_rad"]).s
            snapshot._tracking_monitor = monitor
            snapshot._measurement_path = source
        if monitor.last_time is not None and state.timestamp < monitor.last_time:
            monitor = TrackingMonitor([tuple(p) for p in source])
            snapshot._tracking_monitor = monitor
        measurement_state = state
        if reverse_parking:
            # Project with the reverse travel heading so a nearby forward
            # branch cannot win solely because the body yaw is 180 degrees
            # from the direction of travel.
            measurement_state = replace(
                state,
                phi=math.atan2(
                    math.sin(state.phi + math.pi),
                    math.cos(state.phi + math.pi),
                ),
            )
        measured = monitor.update(measurement_state)
        return measured["ed_m"], math.radians(measured["ephi_deg"])

    def __init__(self, width=1200, height=800, lane_width=3.5,
                 num_lanes=2, target_speed_kmh=40.0, render_fps=60):
        configure_display_driver()
        patch_sysfont_for_python314()
        pygame.init()
        pygame.font.init()

        width = max(800, int(width))
        height = max(600, int(height))
        self.screen = pygame.display.set_mode(
            (width, height), pygame.RESIZABLE | pygame.SHOWN
        )
        pygame.display.set_caption("Lightweight ROS 2 Simulator")
        self.clock = pygame.time.Clock()
        self.camera = Camera(width, height)
        self.renderer = Renderer(self.screen, self.camera)
        self.hud = HUD(self.screen)
        self.lane_width = lane_width
        self.num_lanes = num_lanes
        self.target_speed_kmh = target_speed_kmh
        self.render_fps = max(1, int(render_fps))
        self.started_at = time.monotonic()
        self._history_scenario = ""
        self._camera_scenario = ""
        self._init_navigation()
        self._init_display_state()
        self._scenario_key_state = {
            key: False for key, _name in self.SCENARIO_KEYS
        }
        self._parking_slot_key_state = {
            key: False for key, _slot_id in self.PARKING_SLOT_KEYS
        }
        self._mode_button_rects = {}
        self._parking_slot_button_rects = {}
        self._scene_editor_button_rects = {}
        self._control_source_button_rect = None
        self._editor_message_position = (12, 178)
        self._editor_message_width = 400
        self._manual_keys = set()
        self._snapshot_scene_editing = False

    def poll_actions(self):
        actions = super().poll_actions()
        mapped = []
        for action in actions:
            if action.kind == "key_down":
                if action.value == pygame.K_e and action.value not in self._manual_keys:
                    mapped.append(GuiAction("toggle_pause"))
                self._manual_keys.add(action.value)
                if action.value == pygame.K_LEFTBRACKET:
                    mapped.append(GuiAction("scene_edit_rotate", -1))
                elif action.value == pygame.K_RIGHTBRACKET:
                    mapped.append(GuiAction("scene_edit_rotate", 1))
                continue
            if action.kind == "key_up":
                self._manual_keys.discard(action.value)
                continue
            if action.kind != "mouse_click":
                mapped.append(action)
                continue
            position = action.value
            editor_action = next(
                (
                    editor_action
                    for editor_action, rect in self._scene_editor_button_rects.items()
                    if rect.collidepoint(position)
                ),
                None,
            )
            if editor_action is not None:
                mapped.append(GuiAction(*editor_action))
                continue
            mode = next(
                (
                    name
                    for name, rect in self._mode_button_rects.items()
                    if rect.collidepoint(position)
                ),
                None,
            )
            if mode is not None:
                mapped.append(
                    GuiAction("toggle_pause") if mode == "TOGGLE_PAUSE"
                    else GuiAction("set_mode", mode)
                )
            elif (
                self._control_source_button_rect is not None
                and self._control_source_button_rect.collidepoint(position)
            ):
                mapped.append(GuiAction("toggle_control_source"))
            elif self._snapshot_scene_editing:
                mapped.append(GuiAction("scene_map_click", position))
            else:
                slot_id = next(
                    (
                        slot_id
                        for slot_id, rect in self._parking_slot_button_rects.items()
                        if rect.collidepoint(position)
                    ),
                    None,
                )
                if slot_id is not None:
                    mapped.append(GuiAction("select_parking_slot", slot_id))
        actions = mapped
        key_state = pygame.key.get_pressed()
        for key, mode in (
            (pygame.K_c, "CRUISE"),
            (pygame.K_k, "PARKING"),
        ):
            if key_state[key] and not getattr(self, "_mode_key_state", {}).get(key, False):
                actions.append(GuiAction("set_mode", mode))
        if not hasattr(self, "_mode_key_state"):
            self._mode_key_state = {}
        for key, _mode in (
            (pygame.K_c, "CRUISE"),
            (pygame.K_k, "PARKING"),
        ):
            self._mode_key_state[key] = bool(key_state[key])
        q_down = bool(key_state[pygame.K_q])
        if q_down and not getattr(self, "_q_key_state", False):
            actions.append(GuiAction("toggle_control_source"))
        self._q_key_state = q_down
        actions.append(GuiAction("manual_control", self._manual_control()))
        pressed = pygame.key.get_pressed()
        for key, name in self.SCENARIO_KEYS:
            is_down = bool(pressed[key])
            if is_down and not self._scenario_key_state[key]:
                actions.append(GuiAction("switch_scenario", name))
            self._scenario_key_state[key] = is_down
        for key, slot_id in self.PARKING_SLOT_KEYS:
            is_down = bool(pressed[key])
            if is_down and not self._parking_slot_key_state[key]:
                actions.append(GuiAction("select_parking_slot", slot_id))
            self._parking_slot_key_state[key] = is_down
        return actions

    def render(self, snapshot):
        screen_width, screen_height = self.screen.get_size()
        self.camera.w, self.camera.h = screen_width, screen_height
        self.renderer.w, self.renderer.h = screen_width, screen_height
        self.hud.w, self.hud.h = screen_width, screen_height
        self.hud.compact_editor = snapshot.scene_editing and screen_width < 1050
        # The ROS status contains the authoritative scenario name.  Keep the
        # road drawing aligned with the active scenario's lane count.
        self.num_lanes = (
            3
            if snapshot.status.scenario in {
                "three_lane_double_obs",
                "figure_eight_three_lane",
            }
            else 2
        )
        self._snapshot_scene_editing = snapshot.scene_editing
        actual_obstacles = snapshot.obstacles
        if snapshot.scene_editing:
            snapshot.obstacles = snapshot.draft_obstacles
        super().render(snapshot)
        snapshot.obstacles = actual_obstacles
        return None

    def _draw_scene_edit_world(self, snapshot):
        if not snapshot.scene_editing:
            return
        self._draw_ego_pose_marker(snapshot.draft_ego_pose)
        if snapshot.selected_obstacle_id is not None:
            selected = next(
                (item for item in snapshot.draft_obstacles
                 if item.id == snapshot.selected_obstacle_id),
                None,
            )
            if selected is not None:
                screen = self.camera.world_to_screen(selected.x, selected.y)
                pygame.draw.circle(self.screen, (255, 210, 60), screen, 10, 2)

    def _draw_editor_message(self, message):
        font = self.renderer.font_small
        lines = []
        line = ""
        for word in message.split():
            candidate = f"{line} {word}" if line else word
            if line and font.size(candidate)[0] > self._editor_message_width:
                lines.append(line)
                line = word
            else:
                line = candidate
        if line:
            lines.append(line)
        x, y = self._editor_message_position
        for index, line in enumerate(lines[:3]):
            text = font.render(line, True, (245, 215, 105))
            self.screen.blit(text, (x, y + index * 14))

    def _draw_ego_pose_marker(self, pose):
        if pose is None:
            return
        x, y, yaw = pose
        center = self.camera.world_to_screen(x, y)
        half_length = 2.2 * self.camera.scale
        half_width = 1.0 * self.camera.scale
        corners = []
        for longitudinal, lateral in (
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        ):
            world_x = x + longitudinal * math.cos(yaw) - lateral * math.sin(yaw)
            world_y = y + longitudinal * math.sin(yaw) + lateral * math.cos(yaw)
            corners.append(self.camera.world_to_screen(world_x, world_y))
        pygame.draw.polygon(self.screen, (255, 220, 80), corners, 2)
        pygame.draw.circle(self.screen, (255, 220, 80), center, 4)

    def _draw_mode_controls(self, snapshot):
        screen_width = self.screen.get_width()
        if self.hud.compact_editor:
            x = 18
            right_edge = screen_width - 18
        else:
            left_card_width = min(326, int(screen_width * 0.27))
            right_card_width = min(310, int(screen_width * 0.255))
            x = 18 + left_card_width + 12
            right_edge = screen_width - right_card_width - 30
        area_width = right_edge - x
        y = 76
        gap = 6
        button_height = 28
        columns = 4 if area_width >= 460 else 2
        button_width = (area_width - gap * (columns - 1)) // columns
        parking_scene = snapshot.status.scenario == "reverse_parking"
        secondary_count = len(snapshot.parking_slots) + 1 if parking_scene else 1
        primary_rows = 1 if columns == 4 else 2
        secondary_rows = (secondary_count + columns - 1) // columns
        secondary_y = y + primary_rows * (button_height + gap)
        tools_y = secondary_y + secondary_rows * (button_height + gap)
        tool_columns = 5 if area_width >= 460 else 3
        tool_width = (area_width - gap * (tool_columns - 1)) // tool_columns
        tool_height = 26
        tool_rows = 0 if not snapshot.scene_editing else (10 + tool_columns - 1) // tool_columns
        controls_bottom = (
            tools_y + tool_rows * (tool_height + gap) - gap
            if tool_rows else secondary_y + secondary_rows * (button_height + gap) - gap
        )
        show_message = bool(snapshot.scene_edit_message)
        panel_bottom = controls_bottom + (53 if show_message else 7)
        panel = pygame.Surface((area_width + 12, panel_bottom - y + 12), pygame.SRCALPHA)
        pygame.draw.rect(panel, (8, 13, 20, 220), panel.get_rect(), border_radius=7)
        pygame.draw.rect(panel, (85, 105, 125, 210), panel.get_rect(), 1, border_radius=7)
        self.screen.blit(panel, (x - 6, y - 6))
        self._editor_message_position = (x + 8, controls_bottom + 9)
        self._editor_message_width = area_width - 16

        def cell(index, row_y, count, width, height):
            return pygame.Rect(
                x + (index % count) * (width + gap),
                row_y + (index // count) * (height + gap),
                width,
                height,
            )

        def draw_button(rect, label, color, selected=False):
            pygame.draw.rect(self.screen, color, rect, border_radius=5)
            pygame.draw.rect(
                self.screen,
                (170, 205, 190) if selected else (125, 145, 163),
                rect,
                2 if selected else 1,
                border_radius=5,
            )
            text = self.renderer.font_small.render(label, True, (235, 240, 245))
            self.screen.blit(text, text.get_rect(center=rect.center))

        self._mode_button_rects = {}
        for index, (mode, label) in enumerate(self.MODE_BUTTONS):
            rect = cell(index, y, columns, button_width, button_height)
            self._mode_button_rects[mode] = rect
            if mode == "TOGGLE_PAUSE":
                active = snapshot.status.paused
                label = "RESUME" if active else "PAUSE"
                color = (145, 105, 35) if active else (105, 70, 38)
            else:
                active = snapshot.mode.upper() == mode
                color = (38, 116, 95) if active else (36, 49, 62)
            draw_button(rect, label, color, active)
        self._control_source_button_rect = cell(
            len(self.MODE_BUTTONS), y, columns, button_width, button_height
        )
        source = snapshot.control_source.upper()
        source_color = (39, 105, 145) if source == "MANUAL" else (31, 91, 74)
        draw_button(self._control_source_button_rect, source, source_color, True)

        self._parking_slot_button_rects = {}
        if parking_scene:
            for index, slot in enumerate(snapshot.parking_slots):
                slot_id = int(slot["id"])
                occupied = bool(slot.get("occupied", False))
                selected = slot_id == snapshot.selected_parking_slot_id
                rect = cell(index, secondary_y, columns, button_width, button_height)
                self._parking_slot_button_rects[slot_id] = rect
                color = (
                    (75, 58, 58) if occupied
                    else (39, 111, 85) if selected else (36, 49, 62)
                )
                draw_button(rect, f"SLOT {slot_id}" + (" FULL" if occupied else ""), color, selected)

        self._scene_editor_button_rects = {}
        edit_index = len(snapshot.parking_slots) if parking_scene else 0
        edit_rect = cell(edit_index, secondary_y, columns, button_width, button_height)
        self._scene_editor_button_rects[("toggle_scene_edit", None)] = edit_rect
        edit_label = "EXIT EDIT" if snapshot.scene_editing else "EDIT SCENE"
        edit_color = (145, 105, 35) if snapshot.scene_editing else (36, 49, 62)
        draw_button(edit_rect, edit_label, edit_color, snapshot.scene_editing)
        if snapshot.scene_editing:
            tools = [
                ("EGO", "ego"), ("ADD", "add"), ("MOVE", "move"),
                ("DELETE", "delete"), ("ROT -", ("rotate", -1)),
                ("ROT +", ("rotate", 1)), ("SIZE -", ("resize", -1)),
                ("SIZE +", ("resize", 1)), ("APPLY", "apply"),
                ("CANCEL", "cancel"),
            ]
            for index, (label, action) in enumerate(tools):
                rect = cell(index, tools_y, tool_columns, tool_width, tool_height)
                if action in {"ego", "add", "move", "delete"}:
                    action_key = ("scene_edit_tool", action)
                elif action == "apply":
                    action_key = ("apply_scene_edit", None)
                elif action == "cancel":
                    action_key = ("cancel_scene_edit", None)
                else:
                    kind, value = action
                    action_key = (f"scene_edit_{kind}", value)
                self._scene_editor_button_rects[action_key] = rect
                active_tool = isinstance(action, str) and action == snapshot.scene_edit_tool
                color = (39, 111, 85) if active_tool else (36, 49, 62)
                draw_button(rect, label, color, active_tool)

    def _manual_control(self):
        forward = pygame.K_w in self._manual_keys or pygame.K_UP in self._manual_keys
        reverse = pygame.K_s in self._manual_keys or pygame.K_DOWN in self._manual_keys
        left = pygame.K_a in self._manual_keys or pygame.K_LEFT in self._manual_keys
        right = pygame.K_d in self._manual_keys or pygame.K_RIGHT in self._manual_keys
        brake = pygame.K_SPACE in self._manual_keys
        if forward and reverse:
            forward = reverse = False
            brake = True
        steering = 0.0
        if left and not right:
            # The simulator uses positive yaw/steering for a left turn.  The
            # renderer flips screen Y, so this is the driver's visual-left
            # direction even though the screen-space rotation is inverted.
            steering = 0.45
        elif right and not left:
            steering = -0.45
        return {
            "steering_angle": steering,
            "throttle": 0.35 if forward or reverse else 0.0,
            "brake": 1.0 if brake else 0.0,
            "gear": -1 if reverse and not forward else 1,
        }

    def _draw_status(self, status):
        return None
