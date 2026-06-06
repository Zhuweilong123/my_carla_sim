"""HUD 信息面板 — 显示速度、误差、控制器状态"""

import math
import pygame
import datetime
from typing import List, Optional
from ..simulator.data_types import VehicleState, LogEntry
from .colors import *


class HUD:
    """屏幕HUD信息面板"""

    def __init__(self, screen: pygame.Surface):
        self.screen = screen
        self.w, self.h = screen.get_size()

        # 安全获取字体 - pygame-ce on Python 3.14 的 SysFont 可能失败
        bar_width = int(self.w / 3)
        font_size = int(bar_width / 21) + 2
        try:
            self.font = pygame.font.SysFont("consolas", font_size)
        except Exception:
            self.font = pygame.font.Font(None, font_size)

        small_size = max(10, font_size - 2)
        try:
            self.font_small = pygame.font.SysFont("consolas", small_size)
        except Exception:
            self.font_small = pygame.font.Font(None, small_size)

        # 历史数据 (用于绘制碰撞曲线)
        self.ed_history: List[float] = []
        self.ephi_history: List[float] = []
        self.max_history = 380

    def update_history(self, ed: float, ephi: float):
        self.ed_history.append(ed)
        self.ephi_history.append(ephi)
        if len(self.ed_history) > self.max_history:
            self.ed_history.pop(0)
        if len(self.ephi_history) > self.max_history:
            self.ephi_history.pop(0)

    def render(self, state: VehicleState, target_speed: float,
               control_info: dict, auto_mode: bool,
               fps: float, sim_time: float, real_time: float,
               collision: bool = False,
               map_name: str = "Default",
               ed: float = 0.0, ephi: float = 0.0):
        """渲染HUD面板"""

        # 背景面板 (半透明)
        bar_width = int(self.w / 3)
        info_surface = pygame.Surface((bar_width + 40, self.h))
        info_surface.set_alpha(40)
        self.screen.blit(info_surface, (0, 0))

        # 构建信息文本
        speed_kmh = state.speed_kmh
        mode_str = '>>> AUTO <<<' if auto_mode else 'MANUAL'
        lines = [
            f"Sim FPS:  {fps:16.0f}",
            f"Sim Time: {datetime.timedelta(seconds=int(sim_time))!s:>20s}",
            f"Real Time:{datetime.timedelta(seconds=int(real_time))!s:>20s}",
            " ",
            f"Map:      {map_name:>20s}",
            f"Mode:     {mode_str:>20s}",
            " ",
            "--- Vehicle State ---",
            f"Location: ({state.x:6.1f}, {state.y:6.1f})",
            f"Speed:    {speed_kmh:5.1f} km/h  (target: {target_speed:.0f})",
            f"Accel:    {state.accel:5.2f} m/s^2",
            f"Steer:    {math.degrees(state.steer):5.1f} deg",
            " ",
            "--- Tracking Error ---",
            f"ed:       {ed:+6.3f} m",
            f"ephi:     {math.degrees(ephi):+6.2f} deg",
            " ",
            "--- Control ---",
            f"Throttle: {control_info.get('throttle', 0):5.2f}",
            f"Brake:    {control_info.get('brake', 0):5.2f}",
            f"Steer cmd:{math.degrees(control_info.get('steer', 0)):5.1f} deg",
        ]

        if not auto_mode:
            lines += [" ", "[WASD] drive  [Q] auto  [R] reset"]

        if collision:
            lines.append(" ")
            lines.append("!!! COLLISION !!!")

        # 渲染文本
        v_offset = 4
        for line in lines:
            color = HUD_WARNING if "COLLISION" in line else HUD_TEXT
            text = self.font.render(line, True, color)
            self.screen.blit(text, (8, v_offset))
            v_offset += int(self.h / 42)

        # 误差曲线 (右下角)
        self._draw_error_graph()

    def _draw_error_graph(self):
        """绘制ed/ephi历史曲线"""
        if len(self.ed_history) < 2:
            return

        graph_w = 200
        graph_h = 60
        gx = self.w - graph_w - 10
        gy = self.h - graph_h - 10

        # 背景
        pygame.draw.rect(self.screen, (0, 0, 0, 128),
                         (gx, gy, graph_w, graph_h))

        def draw_curve(data, color, y_offset, scale):
            if len(data) < 2:
                return
            pts = []
            for i, val in enumerate(data):
                sx = gx + int(i * graph_w / len(data))
                sy = gy + y_offset - int(val * scale)
                pts.append((sx, max(gy, min(gy + graph_h, sy))))
            if len(pts) >= 2:
                pygame.draw.lines(self.screen, color, False, pts, 1)

        # ed (白色)
        draw_curve(self.ed_history, (255, 255, 255), graph_h // 2, 10)
        # ephi (黄色)
        draw_curve(self.ephi_history, (255, 255, 0), graph_h // 2, 30)

        # 标签
        t1 = self.font_small.render("ed", True, (255, 255, 255))
        t2 = self.font_small.render("ephi", True, (255, 255, 0))
        self.screen.blit(t1, (gx, gy))
        self.screen.blit(t2, (gx + 30, gy))
