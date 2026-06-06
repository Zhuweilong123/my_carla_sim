# 项目功能点归档

> 项目: Lightweight Planning & Control Simulator (轻量级规划控制仿真器)  
> 日期: 2026-06-06  
> 版本: Phase 1 — 基础框架

---

## 一、项目概述

脱离 CARLA 独立运行的轻量级自动驾驶仿真器。将原 CARLA 项目中的规划控制算法解耦为纯 Python 实现，配合 pygame 2D 俯视图进行可视化和交互。

---

## 二、仿真器核心功能

### 2.1 车辆模型

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 运动学自行车模型 | 4状态(x,y,phi,v)，控制量(steer,accel)，20Hz更新 | `simulator/vehicle.py:kinematic_step()` |
| 动力学自行车模型 | 4状态误差动力学(ed,ėd,eφ,ėφ)，可对接LQR/MPC | `simulator/vehicle.py:dynamic_step()` |
| 误差状态计算 | 完全替代CARLA API，通过Frenet投影计算e_rr | `simulator/vehicle.py:get_error_state()` |
| 车辆参数配置 | Tesla Model 3参数(a,b,m,Cf,Cr,Iz) | `simulator/data_types.py:VehicleParams` |
| SAT碰撞检测 | 分离轴定理实现矩形精确碰撞 | `simulator/vehicle.py:check_collision_with_obstacle()` |

### 2.2 道路模型

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 直线道路 | 参数化生成(length, heading) | `simulator/world.py:_generate_straight()` |
| 圆弧道路 | 参数化生成(radius, angle, center) | `simulator/world.py:_generate_arc()` |
| Waypoint道路 | 自定义路点列表 | `simulator/world.py:_generate_segment()` |
| 多段拼接 | 多段RoadSegment组合成完整道路 | `simulator/world.py:_generate_road()` |
| QP参考线平滑 | 三项代价函数(平滑+紧凑+相似)二次规划 | `algorithms/utils/reference_line.py` |
| 弧长映射 | s_map自动计算，支持车辆原点定位 | `simulator/world.py:_build_s_map()` |
| 多车道支持 | 车道边界渲染 + 车道中心线查询 | `simulator/world.py:get_lane_center()` |

### 2.3 障碍物系统

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 静态障碍物 | 位置固定的矩形障碍物 | `simulator/data_types.py:Obstacle` |
| 动态障碍物 | 带速度+运动方向的障碍物 | `simulator/data_types.py:Obstacle.step()` |
| YAML配置加载 | 从场景配置字典批量创建障碍物 | `simulator/obstacle.py:add_from_config()` |
| AABB快速剔除 | 轴对齐包围盒预筛选 | `simulator/obstacle.py:check_collision()` |
| SAT精确检测 | 分离轴定理矩形碰撞 | `simulator/obstacle.py:_sat_collision()` |

### 2.4 仿真引擎

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 固定步长 | dt=0.05s (20Hz) 物理更新 | `simulator/engine.py` |
| 碰撞检测 | 每步自动检测自车与所有障碍物 | `simulator/engine.py:step()` |
| 终点判断 | 距离目标点<2m自动触发到达 | `simulator/engine.py` |
| 时间管理 | sim_time + step_count 追踪 | `simulator/engine.py` |

### 2.5 可视化 (`visualization/`)

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 俯视图渲染 | 道路(灰)、车道线(白)、参考线(绿)、轨迹(彩色) | `visualization/renderer.py` |
| 车辆绘制 | 旋转矩形 + 方向箭头 | `visualization/renderer.py:draw_vehicle()` |
| 障碍物绘制 | 蓝色(静态)/红色(动态)矩形 + 速度方向线 | `visualization/renderer.py:draw_obstacles()` |
| 相机系统 | 平滑跟随 + 鼠标滚轮缩放(2x-20x) + 平移 | `visualization/renderer.py:Camera` |
| 网格背景 | 10m间距参考网格 | `visualization/renderer.py:draw_grid()` |
| HUD面板 | FPS/速度/位置/误差/控制量/碰撞状态 | `visualization/hud.py` |
| 误差曲线 | 右下角实时e_d(白)/e_φ(黄)历史曲线 | `visualization/hud.py:_draw_error_graph()` |
| 颜色常量 | 统一管理所有可视化颜色 | `visualization/colors.py` |

### 2.6 交互控制

| 功能 | 按键 | 实现位置 |
|------|------|----------|
| 手动/自动切换 | Q | `simulator/app.py:_handle_events()` |
| 油门 | W / ↑ | `simulator/app.py:_manual_control()` |
| 倒车 | S / ↓ | `simulator/app.py:_manual_control()` |
| 左转 | A / ← | `simulator/app.py:_manual_control()` |
| 右转 | D / → | `simulator/app.py:_manual_control()` |
| 刹车 | Space | `simulator/app.py:_manual_control()` |
| 重置 | R | `simulator/app.py:_reset()` |
| 暂停 | P | `simulator/app.py:_handle_events()` |
| 缩放 | +/- 或鼠标滚轮 | `simulator/app.py:_handle_events()` |
| 退出 | ESC | `simulator/app.py:_handle_events()` |

### 2.7 自动模式

| 功能 | 描述 |
|------|------|
| 比例导航控制 | 前视参考线方向 + 横向误差比例转向 + 速度误差比例油门 |
| 可替换接口 | `_auto_control()` 方法预留，后续替换为 LQR/MPC + PID |

---

## 三、算法工具库 (`algorithms/utils/`)

| 功能 | 描述 | 来源 |
|------|------|------|
| 航向角/曲率计算 | 中点欧拉法O(h²)，sin近似避免多值 | `geometry.py` (迁移自 planner_utiles.py) |
| 五次多项式 | 6边界条件→6系数矩阵求逆 + evaluate | `quintic.py` (迁移自 planner_utiles.py) |
| 参考线QP平滑 | 三项代价cvxopt求解，±0.2m约束 | `reference_line.py` (迁移自 planner_utiles.py) |
| Frenet坐标变换 | 匹配点/投影点/s/l/导数全套 | `frenet.py` (迁移自 planner_utiles.py) |

---

## 四、数据结构 (`data_types.py`)

| 类型 | 字段 |
|------|------|
| `PathPoint` | x, y, theta, kappa |
| `FrenetPoint` | s, l, dl, ddl |
| `VehicleState` | x, y, phi, vx, vy, r, steer, accel, timestamp |
| `VehicleParams` | a, b, m, Cf, Cr, Iz |
| `ControlCommand` | steer, throttle, brake, gear |
| `Obstacle` | id, x, y, length, width, speed, heading, type |
| `RoadDef / RoadSegment` | 道路几何定义 |
| `ScenarioConfig` | 场景完整配置 |
| `LogEntry` | 单步仿真数据记录 |

---

## 五、预设场景

| 场景 | 道路 | 障碍物 | 终点 |
|------|------|--------|------|
| `_default_config()` | 200m直道, 2车道 | 无 | (190, 1.75) |
| `straight_with_obstacle()` | 200m直道, 2车道 | 1个静态车@60m | (150, 1.75) |
| `curve_scenario()` | 50m直道 + 90°圆弧 + 100m直道 | 无 | 无(手动终点) |

---

## 六、待实现 (Phase 2-6)

| 阶段 | 内容 |
|------|------|
| Phase 2 | LQR/MPC 横向控制器 + PID 纵向控制器集成 |
| Phase 3 | DP+QP 局部路径规划 + 多进程分离 |
| Phase 4 | A* 全局路径 + S-T 图速度规划 + 动态障碍物 |
| Phase 5 | YAML 场景解析器 + 场景菜单 + 一键重置 |
| Phase 6 | CSV Logger + matplotlib 分析 + MPC vs LQR 对比 |
