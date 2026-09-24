# Vehicle Motion

[English](README.md) | [简体中文](README.zh-CN.md)

Vehicle Motion 是一个轻量级二维车辆仿真与 ROS 2 规划控制工作空间，用于可重复地开发和验证车辆动力学、车道级 Routing、局部避障、路径跟踪及泊车接口。仿真器不依赖 CARLA；`carla_legacy/` 保存归档的 CARLA 代码，不属于当前 ROS 2 运行链路。

## 当前能力

- 固定步长仿真，可配置物理周期、车辆尺寸、限速和执行器参数；支持运动学及简化动力学自行车模型。
- ROS 2 节点覆盖仿真、A* 车道拓扑 Routing、路线参考线生成、局部规划、巡航控制、模式仲裁，以及 Routing/规划数据缺失或过期时的安全停车。
- 二维 GUI 作为 ROS 图的客户端运行，不会创建第二个仿真实例；可显示道路、Routing/参考线、局部规划路径、车辆状态和控制状态，并支持运行时切换场景和模式。
- 独立的 `parking_module` ROS 2 包，提供倒车入库基线规划器和控制器适配节点。
- 回归测试与拆分后的 GitHub Actions：每次 push/PR 运行无 ROS 的 Python 测试；相关改动或手动触发时运行 ROS 构建、集成测试和 launch smoke test。

默认仿真与控制周期为 0.05 秒（20 Hz）。车辆、规划器、Routing、控制器、GUI 和安全相关参数主要配置在 `lightweight_sim/config/default.yaml`；共享时序默认值位于 `lightweight_sim/engine/runtime_config.py`。设计文档统一位于 `lightweight_sim/`：[设计总览](lightweight_sim/design/DESIGN.md)、[算法说明](lightweight_sim/design/ALGORITHMS.md)、[ROS 2 架构](lightweight_sim/design/ROS2.md)、[Routing](lightweight_sim/design/ROUTING.md) 和[参考线](lightweight_sim/design/REFERENCE_LINE.md)。

## 构建与启动（ROS 2 / WSL2）

当前维护的工作流是在 WSL2 中使用 ROS 2 Lyrical。打开新终端后执行：

```bash
source /opt/ros/lyrical/setup.bash
cd /mnt/d/AI_tools/vehicle_motion
python3 -m pip install -r lightweight_sim/requirements.txt
colcon build
source install/setup.bash
ros2 launch lightweight_sim lightweight_sim.launch.py gui:=true
```

在 Windows 挂载的 `/mnt/d` 工作区中，使用普通 `colcon build`，不要加 `--symlink-install`。无图形界面时使用 `gui:=false`。可以通过 launch 参数选择初始场景和转向执行器模型：

```bash
ros2 launch lightweight_sim lightweight_sim.launch.py \
  scenario:=figure_eight gui:=true steering_profile:=ideal
```

`steering_profile:=assumed` 会启用带延迟和转角速率限制的模拟转向执行器。其参数是工程假设，并非真实车辆的标定数据。

## 内置场景

GUI 使用数字键 `1`–`7` 切换场景：

| 按键 | 场景 | 说明 |
| --- | --- | --- |
| 1 | `default` | 双车道直路巡航。 |
| 2 | `obstacle` | 带静态障碍物的直路。 |
| 3 | `three_lane` | 三车道连续障碍场景。 |
| 4 | `curve` | 含 90 度弯道的道路。 |
| 5 | `figure_eight` | 三车道闭环 8 字路线。 |
| 6 | `reverse_parking` | 垂直车位倒车入库场景；自动泊车需使用泊车模块。 |
| 7 | `demo_grid` | 双向城市路网，每个方向两条车道，包含多个路口。 |

标准仿真 launch 默认启动 Routing。内置 JSON 地图位于 `lightweight_sim/config/maps/`；Routing 节点默认加载该地图目录，各场景会请求对应地图和车道。可以通过 Routing 节点参数 `map_dir` 或 `map_file` 使用自定义地图。地图格式、A* 行为及参考线校验/平滑详见 [Routing 文档](lightweight_sim/design/ROUTING.md) 和[参考线文档](lightweight_sim/design/REFERENCE_LINE.md)。

## GUI 操作

- 点击 `CRUISE`、`PARKING` 或 `E-STOP` 选择任务；点击控制来源按钮或按 `Q` 切换 `AUTO`/`MANUAL`。
- `C`、`K`、`E` 分别切换巡航、泊车和紧急停车；`1`–`7` 切换场景。
- 手动模式下，`W/S` 或上下方向键控制前进/倒车，`A/D` 或左右方向键控制转向，`SPACE` 为制动。
- `P` 暂停/继续，`R` 重置，鼠标滚轮或 `+`/`-` 缩放，`ESC` 关闭 GUI。

GUI 需要 Linux 图形显示环境（WSL2 下可使用 WSLg）。无头仿真、ROS 话题/服务和测试不依赖 GUI。

## 巡航与泊车统一控制接口

启动带独立泊车控制器和共享控制模式接口的 GUI：

```bash
ros2 launch parking_module unified_vehicle.launch.py gui:=true
```

也可以直接以泊车模式启动：

```bash
ros2 launch parking_module unified_vehicle.launch.py \
  scenario:=reverse_parking mode:=PARKING gui:=false
```

巡航和泊车控制器发布候选控制指令；`controller_manager` 根据当前模式/控制来源进行选择，是最终的控制仲裁器。泊车规划器和控制器是倒车入库基线方案，并非量产级规划器或车辆安全系统。详见 [`parking_module/README.md`](parking_module/README.md)。

## ROS 接口与诊断

常用话题：

| 话题 | 作用 |
| --- | --- |
| `/vehicle/state`、`/obstacles` | 当前自车状态和障碍物。 |
| `/routing/request`、`/routing/route` | 路线请求和 A* 拓扑路线。 |
| `/routing/reference_line` | 经过地图校验和平滑的车道级参考线。 |
| `/planned_path` | 当前局部规划轨迹。 |
| `/control_command` | 仲裁后发送给仿真器的控制指令。 |
| `/sim/status`、`/tracking/metrics` | 仿真生命周期/安全结果和跟踪诊断。 |

常用检查与控制命令：

```bash
ros2 topic list
ros2 topic echo /vehicle/state
ros2 topic echo /routing/route
ros2 topic echo /routing/reference_line
ros2 topic echo /planned_path
ros2 topic echo /sim/status --once
ros2 service call /sim/reset std_srvs/srv/Empty '{}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/step std_srvs/srv/Trigger '{}'
```

`/control_command` 是最终执行器指令。巡航、手动和泊车控制器使用不同的候选话题，由 `controller_manager` 仲裁。若当前 run 缺少匹配的 Routing/参考线/局部规划结果，或规划数据过期，`safe_stop_node` 会请求制动。节点图、QoS、服务和 launch 参数详见 [ROS 2 架构文档](lightweight_sim/design/ROS2.md)。

## 测试与 CI

在已加载 ROS 环境的仓库根目录构建并运行仿真器测试：

```bash
source /opt/ros/lyrical/setup.bash
cd /mnt/d/AI_tools/vehicle_motion
colcon build --packages-up-to lightweight_sim
source install/setup.bash
python3 -m pytest
```

ROS launch smoke test：

```bash
LIGHTWEIGHT_SIM_INSTALL="$PWD/install" \
  bash lightweight_sim/scripts/ros2_smoke_test.sh
```

独立泊车模块测试可用 `python3 -m pytest parking_module/tests` 运行。GitHub Actions 在每次 push 和 pull request 上运行无需 ROS 的 Python 测试；只有 ROS 接口、launch/配置或相关包发生变化时，才运行单独的 ROS 2 集成工作流（也支持手动触发）。`lightweight_sim/records/` 中的实验过程数据保留在本地，并由 Git 忽略。

## 仓库结构

```text
lightweight_sim/       仿真器、ROS 2 节点、规划/控制、地图、GUI 和测试
lightweight_sim_msgs/  ROS 2 消息与服务定义
parking_module/        独立的倒车入库规划/控制 ROS 2 包
carla_legacy/          归档的 CARLA 实现（lightweight_sim 不依赖）
```
