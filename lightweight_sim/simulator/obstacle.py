"""障碍物管理"""

import numpy as np
from typing import List, Tuple
from .data_types import Obstacle


class ObstacleManager:
    """管理场景中所有障碍物的生命周期和状态更新"""

    def __init__(self):
        self._obstacles: List[Obstacle] = []
        self._next_id = 1

    def add_obstacle(self, obs: Obstacle):
        self._obstacles.append(obs)

    def add_from_config(self, config_list: List[dict]):
        """从场景配置加载障碍物"""
        for cfg in config_list:
            obs = Obstacle(
                id=cfg.get("id", self._next_id),
                x=cfg["x"],
                y=cfg["y"],
                length=cfg.get("length", 4.5),
                width=cfg.get("width", 2.0),
                speed=cfg.get("speed", 0.0),
                heading=cfg.get("heading", 0.0),
                type=cfg.get("type", "vehicle"),
            )
            self._obstacles.append(obs)
            self._next_id = max(self._next_id, obs.id) + 1

    def step(self, dt: float):
        """更新所有动态障碍物位置"""
        for obs in self._obstacles:
            obs.step(dt)

    def get_all(self) -> List[Obstacle]:
        return self._obstacles

    def get_obstacle_xy_list(self) -> List[Tuple[float, float]]:
        """返回所有障碍物的 (x,y) 坐标列表 (用于Frenet变换)"""
        return [(obs.x, obs.y) for obs in self._obstacles]

    def check_collision(self, ego_x: float, ego_y: float,
                        ego_length: float, ego_width: float,
                        ego_phi: float) -> bool:
        """检查自车是否与任何障碍物碰撞 (简化AABB + 旋转矩形)"""
        # 自车AABB
        ego_corners = self._rect_corners(ego_x, ego_y, ego_length, ego_width, ego_phi)
        ego_min = ego_corners.min(axis=0)
        ego_max = ego_corners.max(axis=0)

        for obs in self._obstacles:
            obs_corners = self._rect_corners(obs.x, obs.y, obs.length, obs.width, obs.heading)
            obs_min = obs_corners.min(axis=0)
            obs_max = obs_corners.max(axis=0)

            # AABB快速剔除
            if (ego_max[0] < obs_min[0] or ego_min[0] > obs_max[0] or
                ego_max[1] < obs_min[1] or ego_min[1] > obs_max[1]):
                continue

            # SAT (Separating Axis Theorem) 精确检测
            if self._sat_collision(ego_corners, obs_corners):
                return True
        return False

    @staticmethod
    def _rect_corners(x, y, length, width, heading):
        """计算矩形四角坐标"""
        hl, hw = length / 2, width / 2
        corners = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
        cos_h, sin_h = np.cos(heading), np.sin(heading)
        rot = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        return corners @ rot.T + np.array([[x, y]])

    @staticmethod
    def _sat_collision(corners1, corners2):
        """SAT碰撞检测 (简化: 只检测两个矩形的4条边的法向量)"""
        for corners in [corners1, corners2]:
            for i in range(len(corners)):
                edge = corners[(i+1) % len(corners)] - corners[i]
                axis = np.array([-edge[1], edge[0]])
                axis = axis / (np.linalg.norm(axis) + 1e-10)

                proj1 = corners1 @ axis
                proj2 = corners2 @ axis

                if proj1.max() < proj2.min() or proj2.max() < proj1.min():
                    return False
        return True

    def clear(self):
        self._obstacles.clear()
