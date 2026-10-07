"""Structured HUD for the lightweight simulator."""

import datetime
import math
from typing import List, Tuple

import pygame

from ..engine.simulator.data_types import VehicleState
from .colors import *


class HUD:
    """Draw a compact dashboard without obscuring the simulation view."""

    PANEL = (12, 18, 27, 224)
    BORDER = (86, 105, 126, 210)
    MUTED = (158, 173, 190)
    ACCENT = (74, 204, 255)
    SUCCESS = (70, 224, 145)
    WARNING = (255, 104, 104)

    def __init__(self, screen: pygame.Surface):
        self.screen = screen
        self.w, self.h = screen.get_size()

        def get_font(size: int, bold: bool = False):
            try:
                return pygame.font.SysFont("segoeui", size, bold=bold)
            except Exception:
                return pygame.font.Font(None, size)

        scale = max(0.85, min(1.35, self.w / 1200.0))
        self.font = get_font(max(13, int(14 * scale)))
        self.font_small = get_font(max(10, int(11 * scale)))
        self.font_section = get_font(max(11, int(12 * scale)), True)
        self.font_title = get_font(max(16, int(18 * scale)), True)
        self.font_speed = get_font(max(28, int(34 * scale)), True)

        self.ed_history: List[float] = []
        self.ephi_history: List[float] = []
        self.speed_history = []
        self.history_times = []
        self.history_window_s = 20.0
        self.max_history = 401

    def clear_history(self):
        for series in (self.ed_history, self.ephi_history, self.speed_history, self.history_times):
            series.clear()

    def update_history(self, ed: float, ephi: float, speed_error=None, timestamp=None):
        timestamp = float(timestamp) if timestamp is not None else (
            self.history_times[-1] + 0.05 if self.history_times else 0.0)
        if self.history_times and timestamp < self.history_times[-1]:
            self.clear_history()
        if self.history_times and timestamp == self.history_times[-1]:
            return
        self.ed_history.append(ed)
        self.ephi_history.append(ephi)
        self.speed_history.append(speed_error)
        self.history_times.append(timestamp)
        while (len(self.history_times) > self.max_history or
               timestamp-self.history_times[0] > self.history_window_s):
            for series in (self.ed_history, self.ephi_history, self.speed_history, self.history_times):
                series.pop(0)

    def render(self, state: VehicleState, target_speed: float,
               control_info: dict, auto_mode: bool,
               fps: float, sim_time: float, real_time: float,
               collision: bool = False,
               paused: bool = False,
               map_name: str = "Default",
               ed: float = 0.0, ephi: float = 0.0, speed_error=None):
        """Render the top bar, telemetry cards, graph, and controls."""
        speed_kmh = state.speed_kmh
        steer_deg = math.degrees(state.steer)
        steer_cmd_deg = math.degrees(control_info.get("steer", 0.0))
        if collision:
            status_text, status_color = "COLLISION", self.WARNING
        elif paused:
            status_text, status_color = "PAUSED", (255, 190, 70)
        else:
            status_text, status_color = "RUNNING", self.SUCCESS

        self._draw_top_bar(map_name, auto_mode, status_text, status_color,
                           fps, sim_time, real_time)

        if not getattr(self, "compact_editor", False):
            left_x, top_y, card_w = 18, 76, min(326, int(self.w * 0.27))
            self._draw_vehicle_card(
                left_x, top_y, card_w, 282, state, speed_kmh, target_speed,
                steer_deg, paused,
            )

            right_w = min(310, int(self.w * 0.255))
            right_x = self.w - right_w - 18
            self._draw_control_card(
                right_x, top_y, right_w, 220, control_info, auto_mode,
                steer_cmd_deg,
            )
            self._draw_error_graph(right_x, self.h - 268, right_w, 240, ed, ephi, speed_error)
        self._draw_controls_hint(18, self.h - 54, 410, 34, auto_mode)

        if collision:
            self._draw_alert("COLLISION DETECTED", self.w // 2, self.h - 68)

    def _draw_top_bar(self, map_name: str, auto_mode: bool, status: str,
                      status_color: Tuple[int, int, int], fps: float,
                      sim_time: float, real_time: float):
        bar = pygame.Surface((self.w, 58), pygame.SRCALPHA)
        pygame.draw.rect(bar, (8, 12, 19, 238), bar.get_rect())
        pygame.draw.line(bar, self.ACCENT, (0, 57), (self.w, 57), 2)
        self.screen.blit(bar, (0, 0))

        self._text("LIGHTWEIGHT ROS 2 SIMULATOR", 18, 9,
                   self.font_title, HUD_TEXT)
        scenario = map_name.replace(" [", "  /  ").replace("]", "")
        self._text(scenario, 19, 34, self.font_small, self.MUTED)

        chip_y = 15
        status_w = 112
        self._chip(self.w - 18 - status_w, chip_y, status_w, status,
                   status_color)
        mode = "AUTO" if auto_mode else "MANUAL"
        mode_w = 108
        self._chip(self.w - 30 - status_w - mode_w, chip_y, mode_w, mode,
                   self.SUCCESS if auto_mode else (255, 190, 70))
        time_text = (
            f"SIM {datetime.timedelta(seconds=int(sim_time))}   "
            f"REAL {datetime.timedelta(seconds=int(real_time))}   "
            f"{fps:02.0f} FPS"
            if self.w >= 1000 else
            f"SIM {datetime.timedelta(seconds=int(sim_time))}   {fps:02.0f} FPS"
        )
        time_right = self.w - 30 - status_w - mode_w - 18
        surface = self.font_small.render(time_text, True, self.MUTED)
        self.screen.blit(surface, (max(300, time_right - surface.get_width()), 23))

    def _draw_vehicle_card(self, x: int, y: int, w: int, h: int,
                           state: VehicleState, speed: float,
                           target_speed: float, steer_deg: float,
                           paused: bool = False):
        self._panel(x, y, w, h, "VEHICLE STATE", self.ACCENT)
        speed_text = f"{speed:5.1f}"
        self._text(speed_text, x + 18, y + 32, self.font_speed, HUD_TEXT)
        speed_x = x + 18 + self.font_speed.size(speed_text)[0] + 8
        self._text("km/h", speed_x, y + 47, self.font, self.MUTED)
        target_x, target_y = (x + 18, y + 77) if w < 260 else (x + w - 128, y + 48)
        self._text(f"TARGET  {target_speed:.0f} km/h", target_x, target_y,
                   self.font_small, self.MUTED)

        self._metric_row(x + 18, y + 98, w - 36, "LOCATION",
                         f"({state.x:6.1f}, {state.y:6.1f})")
        self._metric_row(x + 18, y + 137, w - 36, "ACCELERATION",
                         f"{state.accel:+5.2f} m/s²")
        self._metric_row(x + 18, y + 176, w - 36, "STEERING",
                         f"{steer_deg:+5.1f}°")
        self._metric_row(x + 18, y + 215, w - 36, "SIMULATION",
                         "PAUSED" if paused else "ACTIVE",
                         (255, 190, 70) if paused else self.SUCCESS)

    def _draw_control_card(self, x: int, y: int, w: int, h: int,
                           control_info: dict, auto_mode: bool,
                           steer_cmd_deg: float):
        self._panel(x, y, w, h, "CONTROL OUTPUT", (255, 190, 70))
        mode = "AUTONOMOUS CONTROL" if auto_mode else "MANUAL CONTROL"
        self._text(mode, x + 18, y + 42, self.font_small,
                   self.SUCCESS if auto_mode else (255, 190, 70))
        self._metric_row(x + 18, y + 77, w - 36, "THROTTLE",
                         f"{control_info.get('throttle', 0):.2f}")
        self._metric_row(x + 18, y + 116, w - 36, "BRAKE",
                         f"{control_info.get('brake', 0):.2f}")
        self._metric_row(x + 18, y + 155, w - 36, "STEER COMMAND",
                         f"{steer_cmd_deg:+5.1f}°")

    def _draw_error_graph(self, x: int, y: int, w: int, h: int,
                          ed: float, ephi: float, speed_error=None):
        self._panel(x, y, w, h, "TRACKING ERROR", (180, 130, 255))
        row_height = (h - 60) // 3

        def draw_series(row_y, label, data, color, scale):
            self._text(label, x + 14, row_y, self.font_small, color)
            graph = pygame.Rect(x + 14, row_y + 20, w - 28, row_height - 28)
            pygame.draw.rect(self.screen, (7, 11, 17), graph, border_radius=4)
            pygame.draw.line(self.screen, (62, 72, 86),
                             (graph.left, graph.centery),
                             (graph.right - 1, graph.centery), 1)
            if len(data) < 2:
                return
            points = []
            end_time = self.history_times[-1] if self.history_times else 0.0
            for stamp, value in zip(self.history_times, data):
                if value is None or not math.isfinite(value):
                    if len(points) >= 2:
                        pygame.draw.lines(self.screen, color, False, points, 2)
                    points = []
                    continue
                px = graph.right - 1 - int((end_time-stamp) / self.history_window_s * (graph.width-1))
                py = graph.centery - int(value * scale)
                points.append((px, max(graph.top + 2, min(graph.bottom - 2, py))))
            if len(points) >= 2:
                pygame.draw.lines(self.screen, color, False, points, 2)

        draw_series(y + 40, f"LATERAL  ed {ed:+.3f} m",
                    self.ed_history, HUD_TEXT, 12)
        draw_series(y + 40 + row_height, f"HEADING  ephi {math.degrees(ephi):+.2f}°",
                    self.ephi_history, (255, 202, 74), 28)
        speed_label = "--" if speed_error is None else f"{speed_error:+.2f}"
        draw_series(y + 40 + 2*row_height, f"SPEED  ev {speed_label} km/h",
                    self.speed_history, self.ACCENT, 3)
        for fraction, label in ((0.0, "-20 s"), (0.5, "-10 s"), (1.0, "0 s")):
            label_width = self.font_small.size(label)[0]
            self._text(label, x + 14 + int(fraction*(w-28-label_width)),
                       y + h - 17, self.font_small, self.MUTED)

    def _draw_controls_hint(self, x: int, y: int, w: int, h: int,
                            auto_mode: bool):
        panel = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(panel, (12, 18, 27, 215), panel.get_rect(),
                         border_radius=7)
        pygame.draw.rect(panel, self.BORDER, panel.get_rect(), 1,
                         border_radius=7)
        hint = "Q mode   P pause   R reset   +/- zoom   ESC quit"
        if not auto_mode:
            hint = "WASD drive   Q auto   P pause   R reset   ESC quit"
        self._text(hint, x + 14, y + 10, self.font_small, self.MUTED)

    def _draw_alert(self, message: str, center_x: int, y: int):
        text = self.font_section.render(message, True, (255, 238, 238))
        rect = pygame.Rect(0, 0, text.get_width() + 34, 32)
        rect.center = (center_x, y)
        panel = pygame.Surface(rect.size, pygame.SRCALPHA)
        pygame.draw.rect(panel, (135, 28, 42, 235), panel.get_rect(),
                         border_radius=7)
        pygame.draw.rect(panel, (255, 112, 112, 240), panel.get_rect(), 1,
                         border_radius=7)
        panel.blit(text, (17, 8))
        self.screen.blit(panel, rect.topleft)

    def _panel(self, x: int, y: int, w: int, h: int, title: str,
               accent: Tuple[int, int, int]):
        panel = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(panel, self.PANEL, panel.get_rect(), border_radius=9)
        pygame.draw.rect(panel, self.BORDER, panel.get_rect(), 1,
                         border_radius=9)
        pygame.draw.line(panel, (*accent, 230), (14, 34), (w - 14, 34), 2)
        self.screen.blit(panel, (x, y))
        self._text(title, x + 16, y + 12, self.font_section, accent)

    def _metric_row(self, x: int, y: int, width: int, label: str,
                    value: str, value_color=HUD_TEXT):
        self._text(label, x, y, self.font_small, self.MUTED)
        rendered = self.font.render(value, True, value_color)
        self.screen.blit(rendered, (x + width - rendered.get_width(), y - 2))

    def _chip(self, x: int, y: int, w: int, text: str,
              color: Tuple[int, int, int]):
        chip = pygame.Surface((w, 28), pygame.SRCALPHA)
        pygame.draw.rect(chip, (*color, 32), chip.get_rect(), border_radius=6)
        pygame.draw.rect(chip, (*color, 210), chip.get_rect(), 1,
                         border_radius=6)
        label = self.font_section.render(text, True, color)
        chip.blit(label, ((w - label.get_width()) // 2,
                          (28 - label.get_height()) // 2))
        self.screen.blit(chip, (x, y))

    def _text(self, value: str, x: int, y: int, font, color):
        self.screen.blit(font.render(value, True, color), (x, y))
