"""Public ROS GUI module with scenario shortcuts and WSLg-safe window setup."""

import importlib
import sys
import time

import pygame

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
            hint = "1-5 scene   Q mode   P pause   R reset   ESC quit"
        else:
            hint = "1-5 scene   WASD drive   Q auto   P pause   R reset"
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
    )

    def __init__(self, width=1200, height=800, lane_width=3.5,
                 num_lanes=2, target_speed_kmh=40.0):
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
        self.started_at = time.monotonic()
        self._history_scenario = ""
        self._scenario_key_state = {
            key: False for key, _name in self.SCENARIO_KEYS
        }

    def poll_actions(self):
        actions = super().poll_actions()
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

    def _draw_status(self, status):
        return None
