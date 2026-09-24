# ROS 2 节点与接口

本文描述轻量级仿真的当前 ROS 2 执行图。默认 launch 使用 `lightweight_sim/config/launch/lightweight_sim.launch.py`；可调参数集中在 `config/default.yaml`。消息定义属于独立包 `lightweight_sim_msgs/`。

## 节点与数据流

```text
simulator_node --vehicle/state, obstacles, sim/context--> planner_node
simulator_node --routing/request-----------------------> routing_node
routing_node --routing/route---------------------------> reference_line_node
reference_line_node --routing/reference_line-----------> planner_node, controller_node, safe_stop_node
planner_node --planned_path----------------------------> controller_node, safe_stop_node
controller_node --control_command/cruise---------------+
GUI/manual --control_command/manual--------------------+--> controller_manager
safe_stop_node --safety/stop_request-------------------+          |
                                                                control_command
                                                                     |
                                                               simulator_node
```

| 节点 | 职责 |
| --- | --- |
| `simulator_node` | 唯一推进仿真引擎；发布车辆状态、障碍物、场景上下文和 Routing 请求；提供 reset/pause/step 服务 |
| `routing_node` | 加载内置或指定 JSON 地图，响应事件型路线请求和 `routing/compute_route` 服务 |
| `reference_line_node` | 验证 RoutePlan 并生成连续车道参考线与道路边界 |
| `planner_node` | 周期触发局部规划，消费 Routing 参考线和障碍物，发布最新 `planned_path` |
| `controller_node` | 跟踪参考线/局部路径，发布巡航候选指令和 tracking metrics |
| `controller_manager` | 仲裁巡航、手动、泊车候选；唯一发布最终 `control_command` |
| `safe_stop_node` | 监视当前 run 的 Routing 参考线和规划新鲜度，异常时请求停车 |
| `gui_node` | 可选 Pygame ROS 客户端；显示状态/地图/路径并通过服务控制同一个 simulator |

Routing 请求是场景切换或重置时更新的事件，不是固定频率轨迹流。Routing 结果及参考线用于后续规划；ROS 2 节点通过请求/run 标识拒绝旧场景结果。缺少匹配参考线或新鲜局部路径时由安全监督请求制动，不回退到独立的 `/reference_path` topic。

## 主要 Topics

名称为相对名称时会受 launch namespace 影响；`/clock` 始终是全局话题。

| Topic | 消息 | 方向/用途 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | 仿真器发布仿真时钟 |
| `vehicle/state` | `lightweight_sim_msgs/VehicleState` | 仿真器发布车辆位姿、速度、加速度和执行状态 |
| `obstacles` | `lightweight_sim_msgs/ObstacleArray` | 仿真器发布障碍物状态 |
| `sim/context` | `std_msgs/String` | 当前 run、场景、任务及车辆/道路上下文 |
| `routing/request` | `lightweight_sim_msgs/RouteRequest` | 场景适配器发出的路线计算请求 |
| `routing/route` | `lightweight_sim_msgs/RoutePlan` | Routing 发布的拓扑路线及路线几何 |
| `routing/reference_line` | `lightweight_sim_msgs/ReferenceLine` | 参考线节点发布的车道中心线、边界及元数据 |
| `planned_path` | `lightweight_sim_msgs/Path` | 局部规划器发布的当前短路径 |
| `control_command/cruise` | `lightweight_sim_msgs/ControlCommand` | 巡航控制候选 |
| `control_command/manual` | `lightweight_sim_msgs/ControlCommand` | 手动控制候选 |
| `control_command/parking` | `lightweight_sim_msgs/ControlCommand` | 泊车控制候选 |
| `safety/stop_request` | `std_msgs/Bool` | 安全监督的停车请求 |
| `control_command` | `lightweight_sim_msgs/ControlCommand` | 仲裁后的最终执行指令 |
| `control_mode`, `control_mode/status` | `lightweight_sim_msgs/ControlMode` | GUI/外部控制选择及仲裁状态 |
| `tracking/metrics` | `std_msgs/String` | 控制跟踪诊断指标 |
| `sim/status` | `lightweight_sim_msgs/SimulationStatus` | 仿真生命周期及终止状态 |

实际发布/订阅数量可用 `ros2 topic info -v <topic>` 检查；Routing 的服务型调用可用 `ros2 service list` 查看。

## Services

- `routing/compute_route`（`lightweight_sim_msgs/srv/ComputeRoute`）：按请求同步计算路线。
- `sim/reset`（`std_srvs/srv/Empty`）：重置当前仿真及场景运行标识。
- `sim/pause`（`std_srvs/srv/SetBool`）：暂停或继续仿真。
- `sim/step`（`std_srvs/srv/Trigger`）：推进单个仿真步。

## 时钟、QoS 与运行标识

默认物理步长为 0.05 s，控制与规划周期为 0.05 s（20 Hz）；GUI 默认渲染 60 FPS。仿真器用 wall timer 推进固定步长，并发布 `/clock`。规划、控制和 GUI 节点使用仿真时间；simulator 与 safe-stop 监督使用 wall-time 以避免停钟时安全监督也停止更新。

车辆状态和障碍物采用 BEST_EFFORT、depth 1 的传感器 QoS。执行命令使用 RELIABLE、depth 1。场景上下文、路线/参考线、路径及状态快照使用可靠且适合 late-joiner 的锁存 QoS。匹配当前仿真 run 是消费者接受路线、参考线和规划结果的前提。

SI 单位用于物理量：位置 m、速度 m/s、角度 rad、加速度 m/s²；目标速度配置及 HUD 显示使用 km/h。车辆状态中的 `x/y/yaw` 在 map 坐标系，`vx/vy` 在车体坐标系。

## 构建与启动

在已安装 ROS 2 的 Linux/WSL 终端：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /mnt/d/AI_tools/vehicle_motion
python3 -m pip install -r lightweight_sim/requirements.txt
colcon build
source install/setup.bash
ros2 launch lightweight_sim lightweight_sim.launch.py gui:=true
```

主要 launch 参数：

- `scenario`：初始场景，默认 `obstacle`。
- `gui`：是否启动 GUI，默认 `false`。
- `routing_enabled` / `reference_line_enabled`：默认均为 `true`。
- `controller_enabled`：是否启动内置前向巡航控制器，默认 `true`。
- `steering_profile`：`ideal` 或未标定的 `assumed`。
- `namespace`：隔离节点/topic/service 命名空间；`/clock` 仍为全局 topic。

只运行无 GUI 仿真可省略 `gui:=true`。在 Windows 挂载目录 `/mnt/d` 上建议普通 `colcon build`，避免使用可能触发链接清理问题的 `--symlink-install`。

常用检查：

```bash
ros2 topic list
ros2 topic info -v /routing/route
ros2 topic echo /vehicle/state
ros2 topic echo /planned_path
ros2 topic echo /sim/status --once
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: true}"
```

## GUI 与泊车集成

`gui:=true` 启动的是同一仿真节点的 ROS 客户端，不会启动第二个 `SimulationEngine`。GUI 依赖 Pygame 和可用 Linux 图形显示（例如 WSLg）。具体按键和场景切换以 GUI 当前实现为准。

`parking_module/` 是独立的泊车规划/控制包。集成启动入口为：

```bash
ros2 launch parking_module unified_vehicle.launch.py gui:=true
```

`controller_manager` 统一仲裁不同控制候选，最终 topic 仍只有一个写入者。倒车泊车场景可关闭仿真器内置前向控制器，并单独启动泊车控制节点；没有与当前 run 匹配的泊车规划结果时保持停车。泊车模块本身不属于 lightweight_sim 内核依赖。

## 测试

无 ROS 的 Python 测试通过 pytest 运行；依赖 rclpy/ROS 消息的节点和 lifecycle 测试及 launch smoke test 由独立 ROS 集成工作流运行。CI 工作流筛选规则见仓库 `.github/workflows/ci.yml` 和 `.github/workflows/ros-integration.yml`。
