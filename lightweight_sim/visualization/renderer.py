"""俯视图渲染器 — pygame绘制道路、车辆、障碍物、轨迹"""

import math
import pygame
import numpy as np
from typing import List, Tuple, Optional
from ..simulator.data_types import VehicleState, PathPoint
from ..simulator.obstacle import Obstacle
from ..simulator.world import World
from .colors import *


class Camera:
    """2D相机 — 管理视口平移和缩放"""

    def __init__(self, screen_width: int, screen_height: int):
        self.w = screen_width
        self.h = screen_height
        self.cx: float = 0.0        # 视口中心X (世界坐标)
        self.cy: float = 0.0        # 视口中心Y (世界坐标)
        self.scale: float = 6.0     # 像素/米 (默认6px/m)
        self.min_scale = 2.0
        self.max_scale = 20.0

    def world_to_screen(self, wx: float, wy: float) -> Tuple[int, int]:
        sx = int((wx - self.cx) * self.scale + self.w / 2)
        sy = int(-(wy - self.cy) * self.scale + self.h / 2)  # Y轴翻转
        return sx, sy

    def screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        wx = (sx - self.w / 2) / self.scale + self.cx
        wy = -(sy - self.h / 2) / self.scale + self.cy
        return wx, wy

    def follow(self, wx: float, wy: float, smooth: float = 0.1):
        """平滑跟随目标"""
        self.cx += (wx - self.cx) * smooth
        self.cy += (wy - self.cy) * smooth

    def zoom(self, delta: float):
        self.scale = max(self.min_scale, min(self.max_scale, self.scale * (1 + delta)))


class Renderer:
    """俯视图渲染器"""

    def __init__(self, screen: pygame.Surface, camera: Camera):
        self.screen = screen
        self.camera = camera
        self.w, self.h = screen.get_size()
        try:
            self.font_small = pygame.font.SysFont("consolas", 12)
        except Exception:
            self.font_small = pygame.font.Font(None, 12)

    def clear(self):
        self.screen.fill(BACKGROUND)

    def draw_grid(self):
        """绘制参考网格"""
        # 网格间距 (世界坐标, 米)
        grid_spacing = 10
        # 计算可见范围
        left, top = self.camera.screen_to_world(0, 0)
        right, bottom = self.camera.screen_to_world(self.w, self.h)

        start_x = int(left / grid_spacing) * grid_spacing
        start_y = int(bottom / grid_spacing) * grid_spacing

        x = start_x
        while x <= right:
            sx, sy = self.camera.world_to_screen(x, 0)
            pygame.draw.line(self.screen, GRID, (sx, 0), (sx, self.h), 1)
            x += grid_spacing

        y = start_y
        while y <= top:
            sx, sy = self.camera.world_to_screen(0, y)
            pygame.draw.line(self.screen, GRID, (0, sy), (self.w, sy), 1)
            y += grid_spacing

    def draw_road(self, world: World):
        """绘制道路: 路面填充 + 车道边界 (参考线 = 道路中心线)"""
        path = world.ref_path
        if len(path) < 2:
            return

        pts = [self.camera.world_to_screen(p.x, p.y) for p in path]
        lane_w = world.lane_width
        n_lanes = world.num_lanes
        half_w = n_lanes * lane_w / 2  # 半幅路宽

        # ---- 路面填充 (灰色) ----
        # 左边界和右边界构成多边形
        left_edge = []
        right_edge = []
        for i, p in enumerate(path):
            nx = -math.sin(p.theta)
            ny = math.cos(p.theta)
            lx = p.x + nx * (-half_w)
            ly = p.y + ny * (-half_w)
            rx = p.x + nx * half_w
            ry = p.y + ny * half_w
            left_edge.append(self.camera.world_to_screen(lx, ly))
            right_edge.append(self.camera.world_to_screen(rx, ry))

        # 左边界 + 反向的右边界 = 闭合多边形
        road_poly = left_edge + list(reversed(right_edge))
        if len(road_poly) >= 4:
            pygame.draw.polygon(self.screen, ROAD_SURFACE, road_poly)

        # ---- 车道线 ----
        for i in range(n_lanes + 1):
            offset = -half_w + i * lane_w  # 从 -half 到 +half
            if abs(offset) < 0.05:
                # 道路中心线: 虚线 (参考线位置, 即车道分界线)
                self._draw_offset_line(pts, path, offset, LANE_DASH, 1, dashed=True)
            elif i == 0 or i == n_lanes:
                # 道路边界: 实线白色
                self._draw_offset_line(pts, path, offset, ROAD_EDGE, 2)
            else:
                # 内部车道线: 虚线灰色
                self._draw_offset_line(pts, path, offset, LANE_DASH, 1, dashed=True)

        # ---- 参考线: 绿色 (仅在自动模式下显示) ----
        # 注意: 参考线现在位于道路中心
        if len(pts) >= 2:
            pygame.draw.lines(self.screen, REF_PATH_RAW, False, pts, 2)

    def _draw_offset_line(self, screen_pts: List[Tuple[int, int]],
                          path: List[PathPoint], offset: float,
                          color: Tuple[int, int, int], width: int,
                          dashed: bool = False):
        """绘制与参考线平行的偏移线"""
        offset_pts = []
        for i, (sx, sy) in enumerate(screen_pts):
            p = path[i]
            nx = -math.sin(p.theta)
            ny = math.cos(p.theta)
            wx = p.x + nx * offset
            wy = p.y + ny * offset
            offset_pts.append(self.camera.world_to_screen(wx, wy))

        if dashed:
            self._draw_dashed_line(offset_pts, color, width)
        else:
            if len(offset_pts) >= 2:
                pygame.draw.lines(self.screen, color, False, offset_pts, width)

    def _draw_dashed_line(self, pts: List[Tuple[int, int]],
                          color: Tuple[int, int, int], width: int):
        """绘制虚线"""
        for i in range(0, len(pts) - 1, 8):
            seg = pts[i:min(i+4, len(pts))]
            if len(seg) >= 2:
                pygame.draw.lines(self.screen, color, False, seg, width)

    def draw_vehicle(self, state: VehicleState, color: Tuple[int, int, int] = EGO_COLOR):
        """绘制车辆 (带方向的矩形)"""
        sx, sy = self.camera.world_to_screen(state.x, state.y)
        length_px = 4.5 * self.camera.scale
        width_px = 2.0 * self.camera.scale

        # 旋转矩形
        corners = [
            (-length_px/2, -width_px/2),
            (length_px/2, -width_px/2),
            (length_px/2, width_px/2),
            (-length_px/2, width_px/2),
        ]
        phi = -state.phi  # Y轴已翻转
        rotated = []
        for cx, cy in corners:
            rx = cx * math.cos(phi) - cy * math.sin(phi) + sx
            ry = cx * math.sin(phi) + cy * math.cos(phi) + sy
            rotated.append((int(rx), int(ry)))

        pygame.draw.polygon(self.screen, color, rotated)
        pygame.draw.polygon(self.screen, (255, 255, 255), rotated, 1)

        # 方向箭头 (车头)
        arrow_len = length_px * 0.4
        arrow_x = int(sx + arrow_len * math.cos(phi))
        arrow_y = int(sy + arrow_len * math.sin(phi))
        pygame.draw.line(self.screen, (255, 255, 255), (sx, sy), (arrow_x, arrow_y), 2)

    def draw_obstacles(self, obstacles: List[Obstacle]):
        """绘制所有障碍物"""
        for obs in obstacles:
            color = STATIC_OBS if obs.speed < 0.1 else DYNAMIC_OBS
            sx, sy = self.camera.world_to_screen(obs.x, obs.y)
            length_px = obs.length * self.camera.scale
            width_px = obs.width * self.camera.scale

            corners = [
                (-length_px/2, -width_px/2),
                (length_px/2, -width_px/2),
                (length_px/2, width_px/2),
                (-length_px/2, width_px/2),
            ]
            heading = -obs.heading
            rotated = []
            for cx, cy in corners:
                rx = cx * math.cos(heading) - cy * math.sin(heading) + sx
                ry = cx * math.sin(heading) + cy * math.cos(heading) + sy
                rotated.append((int(rx), int(ry)))

            pygame.draw.polygon(self.screen, color, rotated)
            pygame.draw.polygon(self.screen, OBS_BORDER, rotated, 1)

            # 速度方向
            if obs.speed > 0.1:
                vx = int(sx + obs.speed * 0.5 * self.camera.scale * math.cos(-obs.heading))
                vy = int(sy + obs.speed * 0.5 * self.camera.scale * math.sin(-obs.heading))
                pygame.draw.line(self.screen, (255, 255, 0), (sx, sy), (vx, vy), 1)

    def draw_path(self, path: List[Tuple[float, float, float, float]],
                  color: Tuple[int, int, int] = PLANNED_TRAJ,
                  width: int = 2):
        """绘制规划路径"""
        if len(path) < 2:
            return
        pts = []
        for p in path:
            sx, sy = self.camera.world_to_screen(p[0], p[1])
            pts.append((sx, sy))
        pygame.draw.lines(self.screen, color, False, pts, width)

    def draw_points(self, points: List[Tuple[float, float]],
                    color: Tuple[int, int, int], size: int = 3):
        """绘制点序列 (用于debug: 匹配点/投影点等)"""
        for px, py in points:
            sx, sy = self.camera.world_to_screen(px, py)
            pygame.draw.circle(self.screen, color, (sx, sy), size)

    def draw_debug_marker(self, wx: float, wy: float,
                          color: Tuple[int, int, int],
                          size: int = 4, label: str = ""):
        """绘制debug标记点"""
        sx, sy = self.camera.world_to_screen(wx, wy)
        pygame.draw.circle(self.screen, color, (sx, sy), size)
        if label:
            text = self.font_small.render(label, True, color)
            self.screen.blit(text, (sx + 5, sy - 5))
