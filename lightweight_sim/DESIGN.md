# 简易规划控制仿真器 — 技术选型与架构设计

## Context

当前项目的规划/控制算法（LQR, MPC, DP+QP, A*, PID, Frenet 坐标变换）全部**强依赖 CARLA API**（`carla.Vehicle`, `carla.World`, `carla.Map` 等），导致：
- 算法必须启动 CARLA 才能运行（GPU/配置门槛高）
- 无法快速迭代调试单个算法模块
- 测试场景搭建繁琐

需要一个**轻量级、纯 Python、可脱离 CARLA 独立运行**的仿真器来承载现有算法。

---

## 1. 技术选型

### 1.1 总体方案

| 维度 | 选择 | 理由 |
|------|------|------|
| **语言** | Python 3.8+ | 与现有代码一致 |
| **可视化** | **pygame** 2D 俯视图 | 项目已熟悉，轻量跨平台，支持实时键盘交互 |
| **后处理分析** | matplotlib | 速度曲线、轨迹偏差等出图 |
| **数值计算** | numpy | 已在使用 |
| **QP求解** | cvxopt | 已在使用（MPC/路径QP/参考线平滑） |
| **图搜索** | networkx | 已在使用（A*） |
| **场景配置** | YAML | 人类可读，易编辑 |
| **车辆模型** | **运动学自行车模型** + 可选**动力学自行车模型** | 先用运动学（简单稳定），保留动力学接口 |

### 1.2 为什么选择 Pygame 而非其他方案？

| 方案 | 优点 | 缺点 |
|------|------|------|
| **pygame ✅** | 轻量(~5MB)、已熟悉、2D俯视图足够 | 手动绘制道路/车辆 |
| matplotlib.animation | 无需额外依赖 | 交互性差、帧率低 |
| OpenCV | 图像处理方便 | 交互性差 |
| PyQt/PySide | UI控件丰富 | 太重，学习成本 |
| mini-CARLA (CarlaUE4) | 真实物理 | 重量级，需GPU |

---

## 2. 架构设计

### 2.1 分层架构

```
┌─────────────────────────────────────────────────┐
│                  SimulatorApp                     │  ← 主循环 + pygame 渲染
│  ┌───────────┐  ┌───────────┐  ┌─────────────┐  │
│  │ Visualizer│  │  UI/HUD   │  │  Keyboard   │  │
│  │ (俯视图)  │  │ (信息面板)│  │  Controller │  │
│  └─────┬─────┘  └─────┬─────┘  └──────┬──────┘  │
├────────┼───────────────┼───────────────┼─────────┤
│        │               │               │          │
│  ┌─────▼───────────────▼───────────────▼──────┐  │
│  │             SimulationEngine                │  │  ← 时间步进 + 状态更新
│  │  ┌──────────┐ ┌──────────┐ ┌───────────┐  │  │
│  │  │  World   │ │ EgoCar   │ │ Obstacles │  │  │
│  │  │  (道路)  │ │ (自车)   │ │ (障碍物)  │  │  │
│  │  └────┬─────┘ └────┬─────┘ └─────┬─────┘  │  │
│  └───────┼────────────┼─────────────┼────────┘  │
├──────────┼────────────┼─────────────┼───────────┤
│          │            │             │            │
│  ┌───────▼────────────▼─────────────▼───────┐   │
│  │            Algorithm Layer                │   │  ← 从 CARLA 解耦的纯算法
│  │  ┌──────┐ ┌──────┐ ┌──────┐ ┌───────┐  │   │
│  │  │  A*  │ │ DP   │ │ QP   │ │LQR/MPC│  │   │
│  │  │Global│ │Local │ │Local │ │Control │  │   │
│  │  └──────┘ └──────┘ └──────┘ └───────┘  │   │
│  │  ┌──────┐ ┌──────┐                       │   │
│  │  │Frenet│ │PID   │                       │   │
│  │  │Trans │ │Lon   │                       │   │
│  │  └──────┘ └──────┘                       │   │
│  └──────────────────────────────────────────┘   │
├─────────────────────────────────────────────────┤
│              Data Types / Interfaces             │  ← 统一数据结构
│  VehicleState, PathPoint, Obstacle, Scenario...  │
└─────────────────────────────────────────────────┘
```

### 2.2 模块划分与职责

```
lightweight_sim/
├── main.py                  # 入口：解析参数，启动 SimulatorApp
├── config/
│   └── scenarios/           # YAML 场景文件
│       ├── straight.yaml    # 直线道路
│       ├── lane_change.yaml # 换道场景
│       └── curve.yaml       # 弯道场景
├── simulator/
│   ├── __init__.py
│   ├── app.py               # SimulatorApp: 主循环 + pygame 窗口管理
│   ├── engine.py            # SimulationEngine: 时间步进 dt=0.05s
│   ├── world.py             # World: 道路模型 + 参考线管理
│   ├── vehicle.py           # EgoVehicle: 运动学/动力学自行车模型
│   ├── obstacle.py          # Obstacle: 静态/动态障碍物
│   └── data_types.py        # 统一数据结构定义
├── algorithms/              # ★ 从现有代码迁移，去除 CARLA 依赖
│   ├── __init__.py
│   ├── planner/
│   │   ├── global_plan.py       # A* (从 planner/global_path_plan.py 解耦)
│   │   ├── dp_path_plan.py      # DP (从 planner/motion_plan_path_planning.py)
│   │   ├── qp_path_plan.py      # QP (同上)
│   │   └── speed_plan.py        # S-T图 (从 planner/motion_plan_speed_planning.py)
│   ├── controller/
│   │   ├── lat_lqr.py           # LQR (从 controller/Controller.py)
│   │   ├── lat_mpc.py           # MPC (同上)
│   │   ├── lon_pid.py           # PID (同上)
│   │   └── combined.py          # VehicleControl 门面类
│   └── utils/
│       ├── frenet.py            # Frenet 坐标变换 (从 planner/planner_utiles.py)
│       ├── quintic.py           # 五次多项式
│       ├── reference_line.py    # 参考线平滑 QP
│       └── geometry.py          # 航向角/曲率计算
├── visualization/
│   ├── __init__.py
│   ├── renderer.py          # 俯视图渲染器：道路、车辆、障碍物、轨迹
│   ├── hud.py               # HUD: 速度、误差、控制器状态显示
│   └── colors.py            # 颜色常量
└── analysis/
    ├── __init__.py
    ├── logger.py             # 数据记录 (CSV)
    └── plotting.py           # matplotlib 曲线绘制
```

---

## 3. 核心数据结构设计

### 3.1 统一数据类型 (`data_types.py`)

```python
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

# --- 路径相关 ---
@dataclass
class FrenetPoint:
    """Frenet 坐标系下的路径点"""
    s: float       # 弧长 (m)
    l: float       # 横向偏移 (m)
    dl: float      # dl/ds
    ddl: float     # d²l/ds²

@dataclass
class PathPoint:
    """笛卡尔坐标系下的路径点 (x, y, theta, kappa)"""
    x: float
    y: float
    theta: float   # 切向角 (rad)
    kappa: float   # 曲率 (1/m)

# --- 车辆状态 ---
@dataclass
class VehicleState:
    """自车完整状态向量"""
    x: float       # 位置 X (m)
    y: float       # 位置 Y (m)
    phi: float     # 横摆角 (rad)
    vx: float      # 纵向速度 (m/s)
    vy: float      # 横向速度 (m/s)
    r: float       # 横摆角速度 (rad/s)  = phi_dot
    steer: float   # 当前前轮转角 (rad)
    accel: float   # 当前加速度 (m/s²)
    timestamp: float

# --- 障碍物 ---
@dataclass
class Obstacle:
    """障碍物描述"""
    id: int
    x: float
    y: float
    length: float
    width: float
    speed: float           # 速度标量 (m/s)，0=静态
    heading: float = 0.0   # 运动方向 (rad)
    type: str = "vehicle"  # vehicle / pedestrian

# --- 场景 ---
@dataclass
class Scenario:
    """完整场景定义"""
    name: str
    road: 'RoadDef'
    ego_start: VehicleState
    obstacles: List[Obstacle]
    target_speed: float      # km/h
    destination: Tuple[float, float]  # (x, y) 终点
```

### 3.2 道路模型 (`world.py`)

```python
@dataclass
class RoadDef:
    """道路定义 — 由多段线段组成"""
    segments: List['RoadSegment']

@dataclass
class RoadSegment:
    """单段道路"""
    type: str        # "straight" | "arc" | "clothoid" | "waypoints"
    # 直线: length, heading
    # 圆弧: radius, angle
    # waypoints: list of (x, y)
    params: dict
    lane_width: float = 3.5
    num_lanes: int = 2  # 单向车道数
```

**实现策略**：
- 优先实现 `waypoints` 模式（直接给 (x,y) 列表，灵活通用）
- 提供 `straight` 和 `arc` 的辅助构造函数
- 用 `planner_utiles.cal_heading_kappa()` 计算 $(\theta, \kappa)$
- 用 `planner_utiles.smooth_reference_line()` 对路点做 QP 平滑 → 得到参考线

---

## 4. 车辆模型设计

### 4.1 运动学自行车模型（默认）

离散时间运动学更新（以 $[x, y, \phi, v]$ 为状态，$[\delta, a]$ 为控制）：

$$x_{k+1} = x_k + v_k \cos\phi_k \cdot dt$$

$$y_{k+1} = y_k + v_k \sin\phi_k \cdot dt$$

$$\phi_{k+1} = \phi_k + \frac{v_k}{L} \tan\delta_k \cdot dt$$

$$v_{k+1} = v_k + a_k \cdot dt$$

优于动力学模型的方面：简化、稳定、不需要轮胎参数。

### 4.2 动力学自行车模型（可选）

复用现有的 4 状态动力学模型（$e_d, \dot{e}_d, e_\phi, \dot{e}_\phi$），可直接对接 LQR/MPC。

### 4.3 接口设计

```python
class EgoVehicle:
    """自车模型"""
    def __init__(self, state: VehicleState, params: VehicleParams):
        ...
    def step(self, steer: float, accel: float, dt: float) -> VehicleState:
        """运动学更新一步"""
        ...
    def get_state(self) -> VehicleState:
        ...
    def get_error_state(self, ref_path: List[PathPoint]) -> np.ndarray:
        """计算 e_rr = [ed, ėd, eφ, ėφ] 供 LQR/MPC 使用"""
        ...
```

**关键设计决策**：`get_error_state()` 替代 CARLA 的 `vehicle.get_location()` 等方法，通过内部调用 `find_match_points()` 和 `cal_s_l_deri_fun()` 计算 $e_{rr}$，**完全不依赖 CARLA**。

---

## 5. 算法层解耦策略

### 5.1 需要修改的内容

现有算法的 CARLA 依赖集中在获取车辆状态的环节：

| 现有调用 | 替换为 |
|----------|--------|
| `self._vehicle.get_location()` | `state.x, state.y` |
| `self._vehicle.get_velocity()` | `state.vx, state.vy` (需转换为世界坐标) |
| `self._vehicle.get_angular_velocity()` | `state.r` |
| `self._vehicle.get_transform().rotation.yaw` | `state.phi` |
| `self._vehicle.get_acceleration()` | `state.accel` (需估算) |
| `self._vehicle.get_control()` | `state.steer` |

**改造方式**：给 `Lateral_LQR_controller` 等类新增一个 `_control_from_state(state: VehicleState)` 方法，从 `VehicleState` 直接提取状态值，替代 `cal_vehicle_info()` 中的 CARLA API 调用。

### 5.2 不需要修改的内容

以下算法**完全不依赖 CARLA**，可直接迁移：
- `planner_utiles.py`: `cal_heading_kappa()`, `smooth_reference_line()`, `cal_quintic_coefficient()`, `cal_s_l_fun()`, `cal_s_l_deri_fun()`, `cal_s_map_fun()`, `cal_projection_s_fun()`
- `motion_plan_path_planning.py`: `DP_algorithm()`, `Quadratic_planning()`, `cal_lmin_lmax()`, `frenet_2_x_y_theta_kappa()`
- `global_path_plan.py`: `_A_star()`, `_build_graph()` — 但需要自己构建 Graph 替代 CARLA topology

### 5.3 全局路径规划适配

不再使用 CARLA 的 `map.get_topology()`，改为：
- 直接提供 waypoint 列表作为"全局路径"（Simple mode）
- 或用 NetworkX 手动构建路网图（Advanced mode）

---

## 6. 仿真主循环设计

```python
class SimulatorApp:
    def __init__(self, scenario: Scenario, controller_type: str):
        self.engine = SimulationEngine(scenario)   # 世界+车辆+障碍物
        self.visualizer = Visualizer(pygame_screen)
        self.planner = MotionPlanner(...)           # DP+QP
        self.controller = VehicleControl(...)       # LQR/MPC + PID
        self.logger = DataLogger()

    def run(self):
        clock = pygame.time.Clock()
        plan_counter = 0

        while not self.quit:
            dt = clock.tick(60) / 1000.0  # 实际帧间隔

            # 1. 规划层 (低频, 每~50步)
            if plan_counter % 50 == 0:
                ref_path = self.planner.plan(
                    ego_state=self.engine.ego.get_state(),
                    obstacles=self.engine.get_obstacles()
                )
                self.controller.update_path(ref_path)

            # 2. 控制层 (每步)
            control = self.controller.run_step(target_speed)
            self.engine.ego.step(control.steer, control.accel, 0.05)

            # 3. 障碍物更新
            self.engine.update_obstacles(dt)

            # 4. 渲染
            self.visualizer.render(
                self.engine.world,
                self.engine.ego,
                self.engine.obstacles,
                ref_path,
                planned_traj
            )

            plan_counter += 1
```

**关于时间同步**：
- 仿真物理使用**固定步长** $dt = 0.05s$（与 CARLA 的 `fixed_delta_seconds` 一致）
- 渲染帧率独立于物理步长（Pygame `clock.tick(60)` 控制渲染 60fps）
- 控制频率 = 物理频率 = 20Hz
- 规划频率 = 控制频率 / 50 ≈ 0.4Hz

---

## 7. 可视化设计

### 7.1 俯视图渲染 (`renderer.py`)

- **道路**：灰色多边形（参考线 + 车道边界），支持多车道
- **参考线**：绿色虚线（原始）→ 红色实线（规划后）
- **自车**：橙色矩形 + 方向箭头
- **障碍物**：蓝色矩形（静态）/ 红色矩形（动态）
- **规划轨迹**：黄色点序列（DP 结果）+ 白色实线（QP 结果）
- **匹配点/投影点**：青色/深红小圆点（debug用）

### 7.2 HUD 面板 (`hud.py`)

复用现有 `code_4.py` 的 `show_infomation()` 风格：
- 左栏：FPS、实时速度、目标速度、加速度
- 控制信息：Throttle/Brake/Steer/当前模式
- 误差信息：$e_d$ (横向偏差), $e_\phi$ (航向角偏差)
- 碰撞历史曲线

### 7.3 键盘交互

| 按键 | 功能 |
|------|------|
| Q | 切换 手动/自动 模式 |
| W/S | 手动油门/倒车 |
| A/D | 手动转向 |
| Space | 刹车 |
| 1/2/3 | 切换显示层（规划路径/速度曲线/误差曲线） |
| R | 重置场景 |
| P | 暂停/继续 |
| ESC | 退出 |

---

## 8. 场景文件格式 (YAML)

```yaml
# scenarios/lane_change.yaml
name: "换道避障"
description: "直道三车道，前方有静止车辆，自车需换道超越"

road:
  type: straight
  length: 200
  lanes: 3
  lane_width: 3.5

ego:
  start_x: 20
  start_y: 1.75      # 中间车道
  start_phi: 0
  start_speed: 12.5  # m/s = 45 km/h
  target_speed: 50   # km/h

obstacles:
  - id: 1
    x: 60
    y: 1.75
    length: 4.5
    width: 2.0
    speed: 0          # 静止
    type: vehicle
  - id: 2
    x: 65
    y: 5.25
    length: 4.5
    width: 2.0
    speed: 0
    type: vehicle

controller: "LQR_controller"  # 或 "MPC_controller"
planner:
  dp_rows: 12
  dp_cols: 6
  dp_sample_s: 15
  qp_enabled: true
```

---

## 9. 数据记录与分析

### 9.1 Logger

每步记录一行 CSV：
```
timestamp, x, y, phi, vx, vy, speed_kmh, steer, throttle, brake, ed, ephi, target_speed
```

### 9.2 Plotting

仿真结束后用 matplotlib 生成：
1. **轨迹图**：XY 平面轨迹 + 参考线 + 障碍物位置
2. **速度曲线**：实际速度 vs 目标速度
3. **误差曲线**：$e_d$ 和 $e_\phi$ 随时间变化
4. **控制量曲线**：Steer / Throttle / Brake

---

## 10. 实施计划（6 阶段）

### 阶段 1：基础框架 (Foundation)
- [ ] 搭建项目骨架 (`lightweight_sim/` 目录结构)
- [ ] 实现 `data_types.py`（所有数据结构）
- [ ] 实现 `world.py`（Waypoint 道路 + 参考线生成 + QP 平滑）
- [ ] 实现 `vehicle.py`（运动学自行车模型）
- [ ] 实现 `engine.py`（时间步进 + 状态更新）
- [ ] 实现 `renderer.py`（pygame 俯视图绘制道路+车辆）
- [ ] 实现 `app.py`（主循环，先手动控制验证）

**验证**：手动驾驶小车在道路上行驶，pygame 正常渲染

### 阶段 2：控制算法集成
- [ ] 将 `controller/Controller.py` 迁移到 `algorithms/controller/`
- [ ] 新增 `_control_from_state(state)` 方法，去除 CARLA 依赖
- [ ] 集成到仿真主循环（自动模式）
- [ ] 控制器参数可通过 YAML 配置

**验证**：车辆沿参考线自动驾驶（定速 + LQR 横向控制）

### 阶段 3：规划算法集成
- [ ] 迁移 `planner/planner_utiles.py` → `algorithms/utils/`
- [ ] 迁移 `planner/motion_plan_path_planning.py` → `algorithms/planner/`
- [ ] 集成 DP+QP 局部路径规划
- [ ] 多进程架构（规划在子进程，控制在主进程）

**验证**：车辆自动避让静态障碍物

### 阶段 4：全局路径 + 速度规划
- [ ] 实现简单全局路径生成（waypoint 直连 / A* 路网）
- [ ] 补全 S-T 图速度规划
- [ ] 支持动态障碍物场景

**验证**：动态障碍物穿越 + 减速避让

### 阶段 5：场景系统完善
- [ ] YAML 场景解析器
- [ ] 场景选择菜单（pygame UI）
- [ ] 一键重置功能
- [ ] 5+ 个预设场景（直道巡航、换道超车、弯道、动态穿行、多障碍物）

### 阶段 6：分析工具 + 文档
- [ ] CSV Logger + matplotlib 分析脚本
- [ ] MPC vs LQR 对比模式
- [ ] 参数调节面板（键盘实时调整 Q/R 权重）
- [ ] README + 使用说明

---

## 11. 关键技术风险与缓解

| 风险 | 缓解 |
|------|------|
| DP+QP 计算量大，实时性差 | 多进程分离规划/控制；降采样策略 |
| 运动学模型在高速/大曲率下精度不足 | 保留动力学模型接口，可切换 |
| Frenet 法向量方向不一致导致路径震荡 | 复用现有代码中的投影点计算，验证符号约定 |
| cvxopt QP 求解偶尔失败 | 捕获异常，回退到 DP 路径 |

---

## 12. 与现有 CARLA 项目的代码复用

| 现有文件 | 复用方式 |
|----------|----------|
| `controller/Controller.py` | 提取控制类，新增 `_control_from_state()` |
| `planner/motion_plan_path_planning.py` | **直接复用** (无 CARLA 依赖) |
| `planner/planner_utiles.py` | **直接复用** |
| `planner/global_path_plan.py` | 提取 A* 核心（解耦 CARLA topology） |
| `planner/motion_plan_speed_planning.py` | 提取并补全（当前未完成） |
| `code_4.py` 的 HUD 部分 | **参考**重写 |

---

## Verification

完成后通过以下方式验证：

1. **手动驾驶测试**：键盘 W/S/A/D 控制车辆，确认运动学和渲染正确
2. **LQR 巡航测试**：直道 50km/h 巡航，$e_d$ 稳态偏差 < 0.1m
3. **MPC 对比测试**：弯道场景，MPC vs LQR 轨迹偏差对比
4. **DP 避障测试**：前方有静止障碍物，DP 生成避障路径，车辆跟随不碰撞
5. **DP+QP 联合测试**：多障碍物场景，QP 平滑后的避障轨迹
6. **端到端测试**：从起点到终点，全局规划 + 局部 DP+QP + LQR/MPC + PID
7. **性能测试**：DP+QP 规划耗时 < 500ms（单次）、控制周期 < 5ms
