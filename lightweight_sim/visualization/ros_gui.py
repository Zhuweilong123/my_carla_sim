"""Public ROS GUI module with scenario shortcuts and WSLg-safe window setup."""

import importlib
import sys
import time

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
        panel = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(panel, (12, 18, 27, 215), panel.get_rect(), border_radius=7)
        pygame.draw.rect(panel, self.BORDER, panel.get_rect(), 1, border_radius=7)
        if auto_mode:
            hint = "1-6 scene   C cruise   K parking   E stop   P pause   R reset"
        else:
            hint = "W/S drive   A/D steer   SPACE brake   Q auto   E stop"
        self._text(hint, x + 14, y + 10, self.font_small, self.MUTED)


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
    )

    MODE_BUTTONS = (
        # Keep labels ASCII-only: the default Pygame font on WSL often lacks
        # CJK glyphs and renders Chinese labels as indistinguishable squares.
        ("CRUISE", "CRUISE"),
        ("PARKING", "PARKING"),
        ("EMERGENCY_STOP", "E-STOP"),
    )

    @staticmethod
    def _tracking_error(snapshot):
        state = snapshot.state
        if state is None:
            return 0.0, 0.0
        measured = getattr(snapshot, "tracking_metrics", None)
        if measured and abs(state.timestamp-measured["timestamp"]) <= 0.25:
            return measured["ed_m"], math.radians(measured["ephi_deg"])
        source = snapshot.planned_path or snapshot.reference_path
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
        measured = monitor.update(state)
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
        self._scenario_key_state = {
            key: False for key, _name in self.SCENARIO_KEYS
        }
        self._mode_button_rects = {}
        self._control_source_button_rect = None
        self._manual_keys = set()

    def poll_actions(self):
        actions = super().poll_actions()
        mapped = []
        for action in actions:
            if action.kind == "key_down":
                self._manual_keys.add(action.value)
                continue
            if action.kind == "key_up":
                self._manual_keys.discard(action.value)
                continue
            if action.kind != "mouse_click":
                mapped.append(action)
                continue
            position = action.value
            mode = next(
                (
                    name
                    for name, rect in self._mode_button_rects.items()
                    if rect.collidepoint(position)
                ),
                None,
            )
            if mode is not None:
                mapped.append(GuiAction("set_mode", mode))
            elif (
                self._control_source_button_rect is not None
                and self._control_source_button_rect.collidepoint(position)
            ):
                mapped.append(GuiAction("toggle_control_source"))
        actions = mapped
        key_state = pygame.key.get_pressed()
        for key, mode in (
            (pygame.K_c, "CRUISE"),
            (pygame.K_k, "PARKING"),
            (pygame.K_e, "EMERGENCY_STOP"),
        ):
            if key_state[key] and not getattr(self, "_mode_key_state", {}).get(key, False):
                actions.append(GuiAction("set_mode", mode))
        if not hasattr(self, "_mode_key_state"):
            self._mode_key_state = {}
        for key, _mode in (
            (pygame.K_c, "CRUISE"),
            (pygame.K_k, "PARKING"),
            (pygame.K_e, "EMERGENCY_STOP"),
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
        return actions

    def render(self, snapshot):
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
        return super().render(snapshot)

    def _draw_mode_controls(self, snapshot):
        x = 12
        y = 38
        button_width = 112
        button_height = 32
        gap = 8
        self._mode_button_rects = {}
        for index, (mode, label) in enumerate(self.MODE_BUTTONS):
            rect = pygame.Rect(x + index * (button_width + gap), y, button_width, button_height)
            self._mode_button_rects[mode] = rect
            active = snapshot.mode.upper() == mode
            if mode == "EMERGENCY_STOP":
                color = (170, 45, 45) if active else (105, 38, 38)
            else:
                color = (38, 116, 95) if active else (36, 49, 62)
            pygame.draw.rect(self.screen, color, rect, border_radius=5)
            pygame.draw.rect(self.screen, (170, 190, 205), rect, 1, border_radius=5)
            text = self.renderer.font_small.render(label, True, (235, 240, 245))
            text_rect = text.get_rect(center=rect.center)
            self.screen.blit(text, text_rect)
        source_x = x + len(self.MODE_BUTTONS) * (button_width + gap)
        self._control_source_button_rect = pygame.Rect(
            source_x, y, button_width, button_height
        )
        source = snapshot.control_source.upper()
        source_color = (39, 105, 145) if source == "MANUAL" else (31, 91, 74)
        pygame.draw.rect(
            self.screen, source_color, self._control_source_button_rect,
            border_radius=5,
        )
        pygame.draw.rect(
            self.screen, (205, 220, 230), self._control_source_button_rect,
            2 if source == "MANUAL" else 1, border_radius=5,
        )
        text = self.renderer.font_small.render(source, True, (240, 245, 248))
        self.screen.blit(text, text.get_rect(center=self._control_source_button_rect.center))

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
