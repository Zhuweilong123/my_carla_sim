"""SimulatorApp — 主循环 + pygame窗口管理 + 手动/自动模式切换"""

import math
import sys
import time
import pygame
import numpy as np

# ---------------------------------------------------------------------------
# Fix: pygame-ce on Python 3.14 — sysfont.initsysfonts_win32 crashes because
# some Win32 registry values come back as int instead of str.
# Monkey-patch the function to filter out non-string font names.
# ---------------------------------------------------------------------------
def _patch_pygame_sysfont():
    """Monkey-patch pygame.sysfont for Python 3.14 compatibility.

    Python 3.14 changed how Win32 registry enumeration returns values,
    causing some font names to appear as int instead of str.
    Patch both initsysfonts_win32 and get_fonts to be resilient.
    """
    try:
        import pygame.sysfont as _sf

        # Patch 1: replace initsysfonts_win32 with a safe version
        _orig_initsysfonts_win32 = _sf.initsysfonts_win32

        def _safe_initsysfonts_win32():
            fonts = {}
            try:
                result = _orig_initsysfonts_win32()
                for name, path in result.items():
                    if isinstance(name, str) and isinstance(path, str):
                        fonts[name] = path
            except Exception:
                pass
            return fonts

        _sf.initsysfonts_win32 = _safe_initsysfonts_win32

        # Patch 2: SysFont constructor — catch TypeError and fall back
        _orig_SysFont_init = _sf.SysFont.__init__

        def _safe_SysFont_init(self, name, size, bold=False, italic=False):
            try:
                _orig_SysFont_init(self, name, size, bold, italic)
            except TypeError:
                # Fallback: use pygame.font.Font directly
                self.__dict__.clear()
                pygame.font.Font.__init__(self, None, size)

        _sf.SysFont.__init__ = _safe_SysFont_init

    except Exception:
        pass

_patch_pygame_sysfont()
from typing import Optional, List, Tuple

from .data_types import ScenarioConfig, VehicleState, ControlCommand, LogEntry
from .engine import SimulationEngine
from .world import RoadDef, RoadSegment
from .vehicle import EgoVehicle, VehicleParams
from ..algorithms.controller.combined import VehicleController
from ..algorithms.planner.motion_planner import MotionPlanner
from ..visualization.renderer import Renderer, Camera
from ..visualization.hud import HUD
from ..visualization.colors import *


class SimulatorApp:
    """
    简易规划控制仿真器主程序.

    支持两种模式:
      - 手动模式: 键盘W/S/A/D控制
      - 自动模式: LQR/MPC控制器 (后续集成)

    按键:
      Q      - 切换手动/自动模式
      W/S    - 油门/倒车
      A/D    - 转向
      Space  - 刹车
      R      - 重置
      P      - 暂停
      +/-     - 缩放
      ESC    - 退出
    """

    def __init__(self, config: Optional[ScenarioConfig] = None,
                 screen_size: Tuple[int, int] = (1200, 800)):
        pygame.init()
        import warnings
        warnings.filterwarnings('ignore', category=UserWarning, module='pygame.sysfont')
        pygame.font.init()

        self.screen = pygame.display.set_mode(screen_size)
        pygame.display.set_caption("Lightweight Planning & Control Simulator")
        self.clock = pygame.time.Clock()

        # 相机和渲染
        self.camera = Camera(screen_size[0], screen_size[1])
        self.renderer = Renderer(self.screen, self.camera)
        self.hud = HUD(self.screen)

        # 仿真引擎
        if config is None:
            config = self._default_config()
        self.config = config
        self.engine = SimulationEngine(config)

        # 模式
        self.auto_mode = False
        self.paused = False

        # 手动控制状态
        self.manual_steer = 0.0
        self.manual_throttle = 0.0
        self.manual_brake = 0.0

        # 时间
        self.start_real_time = time.time()
        self.sim_time = 0.0

        # 控制器: LQR/MPC + PID
        self._vehicle_para = (1.015, 2.910 - 1.015, 1412, -148970, -82204, 1537)
        ctrl_type = getattr(config, 'controller', 'LQR_controller')
        self._controller_type = ctrl_type
        self.controller = self._create_controller(ctrl_type)
        # 初始参考线 (偏移到车道中心, 让LQR跟踪车道而非道路中心线)
        self._lane_offset = config.ego_start_y  # 车道中心相对于参考线的偏移
        lane_ref_path = self._shift_ref_path(
            self.engine.world.ref_path_as_tuples, self._lane_offset)
        self.controller.update_ref_path(lane_ref_path)

        # 规划器: DP+QP (使用道路中心线作为参考线进行规划)
        global_path = self.engine.world.ref_path_as_tuples
        self.planner = MotionPlanner(global_path)
        self.planner.start()
        self._plan_pending = False     # 是否有规划请求在处理中
        self._plan_counter = 0         # 规划周期计数器
        self._plan_interval = 50       # 每50步规划一次 (≈2.5s @ 20Hz)
        self._first_plan_done = False  # 首次规划是否完成

        # 路径 (用于渲染)
        self.original_ref_path = lane_ref_path  # 原始车道中心线 (始终绘制)
        self.planned_path: List[Tuple] = []     # 控制器当前跟踪路径
        self.planned_traj: List[Tuple] = []     # 规划器避障轨迹

        # 日志
        self.log_entries: List[LogEntry] = []
        self._last_ed = 0.0
        self._last_ephi = 0.0

        # 控制器debug信息
        self._last_ctrl_debug = {}

    # =========================================================================
    # 场景定义
    # =========================================================================

    @staticmethod
    def _shift_ref_path(ref_path: List[Tuple], offset: float):
        """将参考线沿法向量偏移指定距离 (用于车道中心定位)

        Args:
            ref_path: [(x,y,theta,kappa), ...] 道路中心线
            offset: 法向偏移量 (m), 正值=法向量方向
        Returns:
            偏移后的参考线
        """
        if abs(offset) < 0.01:
            return ref_path
        import math as _m
        result = []
        for p in ref_path:
            nx = -_m.sin(p[2])
            ny = _m.cos(p[2])
            result.append((p[0] + offset * nx,
                          p[1] + offset * ny,
                          p[2], p[3]))  # theta, kappa不变
        return result

    def _blend_paths(self, old_path: List[Tuple], new_path: List[Tuple],
                     keep_ahead: int = 5):
        """将新规划路径平滑拼接到当前路径末尾, 避免跳变.

        策略: 在新路径上找离车最近点 → 保留该点前keep_ahead个点 →
              后面全部用新路径

        Args:
            old_path: 当前控制器跟踪的路径 (仅用于回退)
            new_path: 规划器新生成的路径
            keep_ahead: 保留前方点数 (避免切掉车头前方的路径)
        Returns:
            融合后的路径
        """
        if not new_path or len(new_path) < 3:
            return old_path

        # 在新路径上找离车最近的点
        state = self.engine.get_state()
        car_x, car_y = state.x, state.y
        min_d = float('inf')
        car_idx = 0
        for i, p in enumerate(new_path):
            d = (p[0] - car_x)**2 + (p[1] - car_y)**2
            if d < min_d:
                min_d = d
                car_idx = i

        # 从车前方开始取新路径 (留几个点缓冲)
        start_idx = max(0, car_idx - keep_ahead)
        return new_path[start_idx:]

    def _create_controller(self, ctrl_type: str):
        """创建控制器实例"""
        return VehicleController(
            vehicle_para=self._vehicle_para,
            controller_type=ctrl_type,
            target_speed_kmh=self.config.target_speed,
        )

    def _switch_controller(self):
        """在 LQR 和 MPC 之间切换控制器"""
        ref_path = self.controller.ref_path  # 保存当前参考线
        target = self.controller.lon.target_speed

        if self._controller_type == "LQR_controller":
            self._controller_type = "MPC_controller"
        else:
            self._controller_type = "LQR_controller"

        self.controller = self._create_controller(self._controller_type)
        self.controller.update_ref_path(ref_path)
        self.controller.set_target_speed(target)
        print(f"[M] Controller switched to: {self._controller_type}")

    @staticmethod
    def _default_config() -> ScenarioConfig:
        """默认场景: 200m直道, 2车道, 无障碍物"""
        road = RoadDef(
            segments=[
                RoadSegment(
                    type="straight",
                    params={"length": 200, "heading": 0, "start": (0, 0)},
                    lane_width=3.5, num_lanes=2,
                )
            ],
            lane_width=3.5, num_lanes=2,
        )
        return ScenarioConfig(
            name="straight_200m",
            description="200m直道巡航",
            road=road,
            ego_start_x=20, ego_start_y=0.0,   # 参考线即道路中心线
            ego_start_phi=0, ego_start_speed=10.0,
            target_speed=20.0,
            controller="LQR_controller",
            destination=(190, 0.0),
        )

    @staticmethod
    def straight_with_obstacle() -> ScenarioConfig:
        """直道 + 静态障碍物场景

        双车道路面布局 (参考线=分道线y=0):
          - 右车道中心: y = -1.75m
          - 左车道中心: y = +1.75m
        自车和障碍物均置于右车道内.
        """
        road = RoadDef(
            segments=[
                RoadSegment(
                    type="straight",
                    params={"length": 200, "heading": 0, "start": (0, 0)},
                    lane_width=3.5, num_lanes=2,
                )
            ],
            lane_width=3.5, num_lanes=2,
        )
        lane_y = -1.75  # 右车道中心 (参考线y=0往右)
        return ScenarioConfig(
            name="straight_obstacle",
            description="直道右车道前方静止车辆",
            road=road,
            ego_start_x=20, ego_start_y=lane_y,
            ego_start_phi=0, ego_start_speed=10.0,
            target_speed=40.0,
            obstacles=[
                {"id": 1, "x": 60, "y": lane_y, "length": 4.5, "width": 2.0,
                 "speed": 0, "heading": 0, "type": "vehicle"},
            ],
            controller="LQR_controller",
            destination=(180, lane_y),
        )

    @staticmethod
    def three_lane_double_obstacle() -> ScenarioConfig:
        """三车道 + 双障碍物连续避障场景

        3车道路面 (参考线y=0=分道线):
          - 车道0 (右): y = -3.50m
          - 车道1 (中): y =  0.00m
          - 车道2 (左): y = +3.50m
        障碍物1: x=200m, 车道0 → 自车需换到车道1
        障碍物2: x=400m, 车道1 → 自车需换到车道2
        """
        road = RoadDef(
            segments=[
                RoadSegment(
                    type="straight",
                    params={"length": 1000, "heading": 0, "start": (0, 0)},
                    lane_width=3.5, num_lanes=3,
                )
            ],
            lane_width=3.5, num_lanes=3,
        )
        lane0 = -3.5   # 右车道中心
        lane1 = 0.0    # 中车道中心
        lane2 = +3.5   # 左车道中心
        return ScenarioConfig(
            name="three_lane_double_obs",
            description="三车道双障碍物连续避障",
            road=road,
            ego_start_x=20, ego_start_y=lane0,  # 从右车道出发
            ego_start_phi=0, ego_start_speed=5.56,  # 20 km/h
            target_speed=40.0,
            obstacles=[
                {"id": 1, "x": 200, "y": lane0, "length": 4.5, "width": 2.0,
                 "speed": 0, "heading": 0, "type": "vehicle"},
                {"id": 2, "x": 400, "y": lane1, "length": 4.5, "width": 2.0,
                 "speed": 0, "heading": 0, "type": "vehicle"},
            ],
            controller="LQR_controller",
            destination=(600, lane0),  # 最终回到右车道
        )

    @staticmethod
    def curve_scenario() -> ScenarioConfig:
        """弯道场景"""
        road = RoadDef(
            segments=[
                RoadSegment(
                    type="straight",
                    params={"length": 50, "heading": 0, "start": (0, 0)},
                    lane_width=3.5, num_lanes=2,
                ),
                RoadSegment(
                    type="arc",
                    params={"radius": 50, "angle": math.pi / 2,
                            "center": (50, -50), "start_angle": math.pi / 2},
                    lane_width=3.5, num_lanes=2,
                ),
                RoadSegment(
                    type="straight",
                    params={"length": 100, "heading": math.pi / 2,
                            "start": (100, 0)},
                    lane_width=3.5, num_lanes=2,
                ),
            ],
            lane_width=3.5, num_lanes=2,
        )
        return ScenarioConfig(
            name="curve_90deg",
            description="90度弯道",
            road=road,
            ego_start_x=10, ego_start_y=1.75,
            ego_start_phi=0, ego_start_speed=8.0,
            target_speed=30.0,
            controller="LQR_controller",
        )

    # =========================================================================
    # 主循环
    # =========================================================================

    def run(self):
        """主循环"""
        print("=" * 50)
        print("  Lightweight Simulator - 操作说明")
        print("  Q: 切换 手动/自动    R: 重置")
        print("  WASD/方向键: 驾驶    P: 暂停")
        print("  M: 切换 LQR/MPC      Space: 刹车")
        print("  滚轮/+/-: 缩放       ESC: 退出")
        print(f"  Controller: {self._controller_type}")
        print("=" * 50)

        # 用于按键去抖的状态
        prev_q = prev_p = prev_r = prev_m = False
        running = True

        while running:
            # ---- 第1步: 强制刷新事件队列 (Windows关键) ----
            pygame.event.pump()

            # ---- 第2步: 只取出QUIT/滚轮 (不消费KEYDOWN, 留给get_pressed) ----
            for _ in pygame.event.get(pygame.QUIT):
                running = False
            for ev in pygame.event.get(pygame.MOUSEWHEEL):
                self.camera.zoom(ev.y * 0.08)

            if not running:
                break

            # ---- 第3步: 读取键盘状态 (一次性获取, 用于所有按键) ----
            keys = pygame.key.get_pressed()

            # ESC: 退出
            if keys[pygame.K_ESCAPE]:
                running = False
                break

            # Q: 切换自动/手动 (去抖)
            q_now = keys[pygame.K_q]
            if q_now and not prev_q:
                self.auto_mode = not self.auto_mode
                print(f"[Q] Mode: {'AUTO' if self.auto_mode else 'MANUAL'}")
            prev_q = q_now

            # P: 暂停 (去抖)
            p_now = keys[pygame.K_p]
            if p_now and not prev_p:
                self.paused = not self.paused
                print(f"[P] {'PAUSED' if self.paused else 'RESUMED'}")
            prev_p = p_now

            # R: 重置 (去抖)
            r_now = keys[pygame.K_r]
            if r_now and not prev_r:
                self._reset()
            prev_r = r_now

            # M: 切换控制器 LQR↔MPC (去抖)
            m_now = keys[pygame.K_m]
            if m_now and not prev_m:
                self._switch_controller()
            prev_m = m_now

            # +/-: 缩放 (去抖)
            if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]:
                self.camera.zoom(0.05)
            if keys[pygame.K_MINUS]:
                self.camera.zoom(-0.05)

            # ---- 第4步: 暂停检查 ----
            if self.paused:
                self.clock.tick(30)
                # 暂停时也要渲染一帧(更新画面)
                self._render(self.engine.get_state(),
                            ControlCommand(steer=self.manual_steer,
                                          throttle=self.manual_throttle,
                                          brake=self.manual_brake))
                continue

            # ---- 第5步: 控制 ----
            if self.auto_mode:
                control = self._auto_control()
            else:
                control = self._manual_control(keys)

            # ---- 第5.5步: 路径规划 (低频, 多进程) ----
            if (self.auto_mode and not self.engine.collision_occurred
                    and self._plan_counter % self._plan_interval == 0):
                state = self.engine.get_state()
                # 预测规划起点的位置 (补偿规划延迟)
                pred_ts = 0.2
                pred_x = state.x + state.vx * pred_ts * math.cos(state.phi) \
                         - state.vy * pred_ts * math.sin(state.phi)
                pred_y = state.y + state.vy * pred_ts * math.cos(state.phi) \
                         + state.vx * pred_ts * math.sin(state.phi)

                # 向规划子进程发送请求
                self.planner.plan(
                    ego_state=state,
                    obstacles=self.engine.obstacles.get_all(),
                    vehicle_v=(state.vx, state.vy),
                    vehicle_a=(state.accel, 0.0),
                    pred_loc=(pred_x, pred_y),
                    vehicle_loc=(state.x, state.y),
                )
                self._plan_pending = True

            # 非阻塞接收规划结果
            if self._plan_pending:
                if self.planner.poll_result():
                    planned = self.planner.get_result()
                    if planned is not None:
                        if planned and len(planned) > 0:
                            self.planned_traj = planned
                            # 平滑融合: 旧路径→新路径, 避免跳变
                            blended = self._blend_paths(
                                self.controller.ref_path, planned)
                            self.controller.update_ref_path(blended)
                        else:
                            # 空结果=无障碍物, 回到车道中心线
                            self.planned_traj = []
                            self.controller.update_ref_path(self.original_ref_path)
                        if not self._first_plan_done:
                            print("[App] First planning result received!")
                        self._first_plan_done = True
                    self._plan_pending = False
                elif self._plan_counter > self._plan_interval + 200:
                    print("[App] Planning timed out, resetting planner state")
                    self._plan_pending = False

            self._plan_counter += 1

            # ---- 第6步: 物理步进 ----
            dt = 0.05
            state = self.engine.step(control, dt)
            self.sim_time += dt

            # ---- 第7步: 误差计算 ----
            err_state = self.engine.get_error_state()
            if err_state is not None:
                self._last_ed = err_state[0]
                self._last_ephi = err_state[2]
            self.hud.update_history(self._last_ed, self._last_ephi)

            # ---- 第8步: 状态检查 ----
            if self.engine.collision_occurred:
                print("[!] Collision detected!")
            if self.engine.reached_destination:
                print("[✓] Destination reached!")
                running = False

            # ---- 第9步: 相机跟随 ----
            self.camera.follow(state.x, state.y)

            # ---- 第10步: 日志 ----
            self.log_entries.append(LogEntry(
                timestamp=self.sim_time,
                x=state.x, y=state.y, phi=state.phi,
                vx=state.vx, vy=state.vy,
                speed_kmh=state.speed_kmh,
                steer=control.steer,
                throttle=control.throttle,
                brake=control.brake,
                ed=self._last_ed, ephi=self._last_ephi,
                target_speed=self.engine.target_speed,
            ))

            # ---- 第11步: 渲染 ----
            self._render(state, control)

            # ---- 第12步: 帧率控制 ----
            self.clock.tick(60)

        # 结束
        self._on_exit()

    # =========================================================================
    # 控制
    # =========================================================================

    def _auto_control(self) -> ControlCommand:
        """自动驾驶控制 — 使用 LQR/MPC + PID"""
        state = self.engine.get_state()

        # 控制器参考线: 优先用规划轨迹 → 否则用车道路中心偏移线
        if self.planned_traj:
            ref_path = self.planned_traj
        else:
            ref_path = self._shift_ref_path(
                self.engine.world.ref_path_as_tuples, self._lane_offset)

        if not ref_path:
            return ControlCommand()

        # 仅在无规划轨迹时更新参考线
        if not self.planned_traj:
            self.controller.update_ref_path(ref_path)

        # 更新目标速度
        self.controller.set_target_speed(self.engine.target_speed)

        # 计算控制量
        steer, throttle, brake = self.controller.step(
            x=state.x, y=state.y, phi=state.phi,
            vx=state.vx, vy=state.vy, r=state.r,
        )

        # 保存参考线用于渲染
        self.planned_path = ref_path

        # 保存debug标记
        self._last_ctrl_debug = {
            'x_pre': self.controller.lat.x_pre,
            'y_pre': self.controller.lat.y_pre,
            'x_pro': self.controller.lat.x_pro,
            'y_pro': self.controller.lat.y_pro,
        }

        return ControlCommand(steer=steer, throttle=throttle, brake=brake)

    def _manual_control(self, keys) -> ControlCommand:
        """手动键盘控制 (keys 由主循环的 get_pressed() 传入)"""
        # 油门
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            self.manual_throttle = min(1.0, self.manual_throttle + 0.05)
            self.manual_brake = 0.0
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            self.manual_throttle = 0.0
            self.manual_brake = min(1.0, self.manual_brake + 0.1)
        else:
            self.manual_throttle = 0.0
            self.manual_brake = 0.0

        # 转向
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            self.manual_steer = max(-1.0, self.manual_steer - 0.05)
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            self.manual_steer = min(1.0, self.manual_steer + 0.05)
        else:
            self.manual_steer *= 0.8  # 回正

        # 刹车
        if keys[pygame.K_SPACE]:
            self.manual_brake = 1.0
            self.manual_throttle = 0.0

        return ControlCommand(
            steer=self.manual_steer,
            throttle=self.manual_throttle,
            brake=self.manual_brake,
        )

    # =========================================================================
    # 事件处理
    # =========================================================================

    def _handle_events(self) -> bool:
        """处理pygame事件, 返回False表示退出

        同时使用 KEYDOWN 事件(一次性动作) 和 get_pressed(连续动作).
        切换类按键(Q/P/R)用事件+去抖双重保障.
        """
        # --- 事件处理 (一次性动作: 退出, 切换, 重置) ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_q:
                    self.auto_mode = not self.auto_mode
                    self._prev_q_pressed = True
                    print(f"[Q] Mode: {'AUTO' if self.auto_mode else 'MANUAL'}")
                elif event.key == pygame.K_p:
                    self.paused = not self.paused
                    print(f"[P] {'PAUSED' if self.paused else 'RESUMED'}")
                elif event.key == pygame.K_r:
                    self._reset()
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    self.camera.zoom(0.15)
                elif event.key == pygame.K_MINUS:
                    self.camera.zoom(-0.15)
            elif event.type == pygame.MOUSEWHEEL:
                self.camera.zoom(event.y * 0.08)

        # --- get_pressed 兜底 (去抖: 防止一帧触发多次) ---
        keys = pygame.key.get_pressed()

        # Q: 自动/手动切换
        q_now = keys[pygame.K_q]
        if q_now and not getattr(self, '_prev_q_pressed', False):
            self.auto_mode = not self.auto_mode
            print(f"[Q] Mode: {'AUTO' if self.auto_mode else 'MANUAL'}")
        self._prev_q_pressed = q_now

        # P: 暂停
        p_now = keys[pygame.K_p]
        if p_now and not getattr(self, '_prev_p_pressed', False):
            self.paused = not self.paused
            print(f"[P] {'PAUSED' if self.paused else 'RESUMED'}")
        self._prev_p_pressed = p_now

        # R: 重置
        r_now = keys[pygame.K_r]
        if r_now and not getattr(self, '_prev_r_pressed', False):
            self._reset()
        self._prev_r_pressed = r_now

        return True

    def _reset(self):
        """重置仿真"""
        self.engine = SimulationEngine(self.config)
        self.sim_time = 0.0
        self.manual_steer = 0.0
        self.manual_throttle = 0.0
        self.manual_brake = 0.0
        self.log_entries.clear()
        self.hud.ed_history.clear()
        self.hud.ephi_history.clear()
        self.planned_path.clear()
        self.planned_traj.clear()
        self._last_ed = 0.0
        self._last_ephi = 0.0
        self._last_ctrl_debug = {}
        self._plan_pending = False
        self._plan_counter = 0
        self._first_plan_done = False

        # 保留当前控制器类型，更新车道中心参考线
        self.original_ref_path = self._shift_ref_path(
            self.engine.world.ref_path_as_tuples, self._lane_offset)
        self.controller.update_ref_path(self.original_ref_path)
        self.planned_path = []
        self.planned_traj.clear()
        self.controller.set_target_speed(self.engine.target_speed)
        self.controller.lat.min_index = 0  # 重置匹配点索引
        self.controller.lon.reset()        # 重置PID误差缓冲

        self.start_real_time = time.time()
        print(f"[R] Reset complete (controller: {self.controller.controller_type})")

    # =========================================================================
    # 渲染
    # =========================================================================

    def _render(self, state: VehicleState, control: ControlCommand):
        """渲染一帧"""
        self.renderer.clear()
        self.renderer.draw_grid()

        # 道路
        self.renderer.draw_road(self.engine.world)

        # 原始车道中心线 (始终绘制, 青色虚线, 作为"原路线"参考)
        if self.original_ref_path:
            self.renderer.draw_path(self.original_ref_path, (0, 200, 200), 1, dashed=True)

        # 控制器当前跟踪路径 (绿色实线)
        if self.planned_path and self.planned_path != self.original_ref_path:
            self.renderer.draw_path(self.planned_path, REF_PATH_SMOOTH, 2)
        # 规划器避障轨迹 (白色)
        if self.planned_traj:
            self.renderer.draw_path(self.planned_traj, PLANNED_TRAJ, 2)

        # 障碍物
        self.renderer.draw_obstacles(self.engine.obstacles.get_all())

        # 自车
        self.renderer.draw_vehicle(state)

        # 控制器debug标记 (预测点 + 投影点)
        if self.auto_mode and self._last_ctrl_debug:
            dbg = self._last_ctrl_debug
            self.renderer.draw_debug_marker(dbg['x_pre'], dbg['y_pre'],
                                            PREDICT_POINT, size=4, label='pre')
            self.renderer.draw_debug_marker(dbg['x_pro'], dbg['y_pro'],
                                            PROJ_POINT, size=4, label='proj')

        # HUD
        real_time = time.time() - self.start_real_time

        # ---- 屏幕顶部模式指示器 ----
        ctrl_name = "LQR" if "LQR" in self._controller_type else "MPC"
        mode_text = f"AUTO ({ctrl_name})" if self.auto_mode else "MANUAL"
        mode_color = (0, 255, 100) if self.auto_mode else (255, 200, 50)
        try:
            big_font = pygame.font.Font(None, 36)
            mode_surf = big_font.render(mode_text, True, mode_color, (0, 0, 0))
            self.screen.blit(mode_surf, (self.screen.get_width() // 2 - mode_surf.get_width() // 2, 5))
        except Exception:
            pass

        # 帧率+状态
        fps = self.clock.get_fps()
        control_info = {
            'steer': control.steer,
            'throttle': control.throttle,
            'brake': control.brake,
        }
        self.hud.render(
            state=state,
            target_speed=self.engine.target_speed,
            control_info=control_info,
            auto_mode=self.auto_mode,
            fps=fps,
            sim_time=self.sim_time,
            real_time=real_time,
            collision=self.engine.collision_occurred,
            map_name=f"{self.config.name} [{self.controller.controller_type}]",
            ed=self._last_ed,
            ephi=self._last_ephi,
        )

        pygame.display.flip()

    def _on_exit(self):
        """退出时清理"""
        if self.planner:
            self.planner.stop()
        print(f"Simulation ended. {len(self.log_entries)} steps logged.")
        pygame.quit()
