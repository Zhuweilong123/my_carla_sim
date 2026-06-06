"""道路模型 — 生成参考线和管理车道信息"""

import math
import numpy as np
from typing import List, Tuple, Optional
from ..algorithms.utils.geometry import cal_heading_kappa
from ..algorithms.utils.reference_line import smooth_reference_line
from .data_types import PathPoint, RoadDef, RoadSegment


class World:
    """
    仿真世界 — 管理道路几何和参考线.

    支持三种道路定义方式:
      - waypoints: 直接给定路点列表
      - straight:  参数化直线
      - arc:       参数化圆弧
    """

    def __init__(self, road_def: Optional[RoadDef] = None):
        self.road_def = road_def or RoadDef()
        self.lane_width = self.road_def.lane_width
        self.num_lanes = self.road_def.num_lanes

        # 参考线 (中心线)
        self._raw_waypoints: List[Tuple[float, float]] = []   # 原始路点
        self._ref_path: List[PathPoint] = []                  # 平滑后的参考线
        self._s_map: List[float] = []                         # 弧长映射

        # 生成道路
        self._generate_road()

    def _generate_road(self):
        """根据RoadDef生成道路几何"""
        if self.road_def.segments:
            all_waypoints = []
            for seg in self.road_def.segments:
                pts = self._generate_segment(seg)
                if all_waypoints and pts:
                    # 避免重复连接点
                    if math.hypot(pts[0][0] - all_waypoints[-1][0],
                                  pts[0][1] - all_waypoints[-1][1]) < 0.01:
                        pts = pts[1:]
                all_waypoints.extend(pts)
            self._raw_waypoints = all_waypoints
        else:
            # 默认: 200m直道
            self._raw_waypoints = self._generate_straight(200, 0)

        # 生成参考线
        if len(self._raw_waypoints) >= 2:
            self._ref_path = self._build_ref_path(self._raw_waypoints)
            self._build_s_map()
        else:
            self._ref_path = []

    def _generate_segment(self, seg: RoadSegment) -> List[Tuple[float, float]]:
        """根据路段类型生成路点"""
        t = seg.type.lower()
        p = seg.params
        if t == "straight":
            return self._generate_straight(
                length=p.get("length", 100),
                heading=p.get("heading", 0),
                start=p.get("start", (0, 0)),
                resolution=p.get("resolution", 2.0),
            )
        elif t == "arc":
            return self._generate_arc(
                radius=p.get("radius", 50),
                angle=p.get("angle", math.pi/2),
                center=p.get("center", (50, -50)),
                start_angle=p.get("start_angle", 0),
                resolution=p.get("resolution", 2.0),
            )
        elif t == "waypoints":
            return p.get("points", [])
        else:
            raise ValueError(f"Unknown road segment type: {t}")

    @staticmethod
    def _generate_straight(length: float, heading: float,
                           start: Tuple[float, float] = (0, 0),
                           resolution: float = 2.0) -> List[Tuple[float, float]]:
        """生成直线道路"""
        n_pts = max(2, int(length / resolution) + 1)
        xs = np.linspace(start[0], start[0] + length * math.cos(heading), n_pts)
        ys = np.linspace(start[1], start[1] + length * math.sin(heading), n_pts)
        return list(zip(xs, ys))

    @staticmethod
    def _generate_arc(radius: float, angle: float,
                      center: Tuple[float, float],
                      start_angle: float = 0,
                      resolution: float = 2.0) -> List[Tuple[float, float]]:
        """生成圆弧道路"""
        arc_length = abs(radius * angle)
        n_pts = max(2, int(arc_length / resolution) + 1)
        angles = np.linspace(start_angle, start_angle + angle, n_pts)
        xs = center[0] + radius * np.cos(angles)
        ys = center[1] + radius * np.sin(angles)
        return list(zip(xs, ys))

    def _build_ref_path(self, waypoints: List[Tuple[float, float]]) -> List[PathPoint]:
        """从原始路点构建平滑参考线"""
        # 降采样: 如果点太密, 先降采样再平滑
        if len(waypoints) > 100:
            step = len(waypoints) // 60
            waypoints = waypoints[::max(1, step)]

        # QP平滑
        smoothed = smooth_reference_line(waypoints)
        return [PathPoint(x=p[0], y=p[1], theta=p[2], kappa=p[3]) for p in smoothed]

    def _build_s_map(self):
        """构建弧长映射 (以第一个点为原点)"""
        self._s_map = [0.0]
        for i in range(1, len(self._ref_path)):
            p0, p1 = self._ref_path[i-1], self._ref_path[i]
            ds = math.hypot(p1.x - p0.x, p1.y - p0.y)
            self._s_map.append(self._s_map[-1] + ds)

    # ---- 查询接口 ----

    @property
    def ref_path(self) -> List[PathPoint]:
        return self._ref_path

    @property
    def ref_path_as_tuples(self) -> List[Tuple[float, float, float, float]]:
        """返回 [(x,y,theta,kappa), ...] 格式 (供frenet工具函数使用)"""
        return [(p.x, p.y, p.theta, p.kappa) for p in self._ref_path]

    @property
    def s_map(self) -> List[float]:
        return self._s_map

    @property
    def total_length(self) -> float:
        """道路总长 (m)"""
        return self._s_map[-1] if self._s_map else 0.0

    def get_lane_center(self, lane_idx: int) -> float:
        """
        获取车道中心线相对于道路中心线的横向偏移 (参考线=道路中心).

        lane_idx=0 为最右侧车道, 偏移为负值.
        例: 2车道, 单车道宽3.5m:
          lane 0 (右): -1.75m
          lane 1 (左): +1.75m
        """
        half_w = self.num_lanes * self.lane_width / 2
        return -half_w + (lane_idx + 0.5) * self.lane_width

    def get_lane_boundaries(self) -> List[float]:
        """返回所有车道边界相对于道路中心线的横向偏移"""
        boundaries = []
        half_w = self.num_lanes * self.lane_width / 2
        for i in range(self.num_lanes + 1):
            boundaries.append(-half_w + i * self.lane_width)
        return boundaries

    def is_on_road(self, x: float, y: float, margin: float = 2.0) -> bool:
        """检查点是否在道路上 (基于到参考线的距离)"""
        if not self._ref_path:
            return True
        # 找最近点
        min_d = float('inf')
        for p in self._ref_path:
            d = math.hypot(p.x - x, p.y - y)
            if d < min_d:
                min_d = d
        road_half_width = self.num_lanes * self.lane_width / 2 + margin
        return min_d < road_half_width
