# Vehicle Motion

[English](README.md) | [简体中文](README.zh-CN.md)

Vehicle Motion 是一个轻量级二维车辆仿真与 ROS 2 规划控制工作空间，用于可重复地开发和验证车辆动力学、车道级 Routing、局部避障、路径跟踪及泊车接口。仿真器不依赖 CARLA；`carla_legacy/` 保存归档的 CARLA 代码，不属于当前 ROS 2 运行链路。

## 当前能力

- 固定步长仿真，可配置物理周期、车辆尺寸、限速和执行器参数；支持运动学及简化动力学自行车模型。
- ROS 2 节点覆盖仿真、A* 车道拓扑 Routing、路线参考线生成、局部规划、巡航控制、模式仲裁，以及 Routing/规划数据缺失或过期时的安全停车。
- 二维 GUI 作为 ROS 图的客户端运行，不会创建第二个仿真实例；可显示道路、Routing/参考线、局部规划路径、车辆状态和控制状态，并支持运行时切换场景和模式。
- 独立的 `parking_module` ROS 2 包，默认使用 Hybrid A* 倒车入库规划器，同时保留基线规划器用于对比，并提供控制器适配节点。
- 回归测试与拆分后的 GitHub Actions：每次 push/PR 运行无 ROS 的 Python 测试；相关改动或手动触发时运行 ROS 构建、集成测试和 launch smoke test。

默认仿真与控制周期为 0.05 秒（20 Hz）。常用设置在 `lightweight_sim/config/default.yaml`；车辆、算法、系统接口与备用参数分层配置，详见[配置说明](lightweight_sim/config/README.md)。共享时序默认值位于 `lightweight_sim/engine/runtime_config.py`。设计文档统一位于 `lightweight_sim/`：[设计总览](lightweight_sim/design/DESIGN.md)、[算法说明](lightweight_sim/design/ALGORITHMS.md)、[ROS 2 架构](lightweight_sim/design/ROS2.md)、[Routing](lightweight_sim/design/ROUTING.md) 和[参考线](lightweight_sim/design/REFERENCE_LINE.md)。

## 构建与启动（ROS 2 / WSL2）

仿真器仅通过 ROS 2 运行；原独立 Pygame 主程序已下线。当前维护的工作流是在 WSL2 中使用 ROS 2 Lyrical。打开新终端后执行：

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
| 6 | `reverse_parking` | 垂直车位倒车入库场景，由集成的泊车模块规划和控制。 |
| 7 | `demo_grid` | 双向城市路网，每个方向两条车道，包含多个路口。 |

标准仿真 launch 默认启动 Routing。内置 JSON 地图位于 `lightweight_sim/config/maps/`；Routing 节点默认加载该地图目录，各场景会请求对应地图和车道。可以通过 Routing 节点参数 `map_dir` 或 `map_file` 使用自定义地图。地图格式、A* 行为及参考线校验/平滑详见 [Routing 文档](lightweight_sim/design/ROUTING.md) 和[参考线文档](lightweight_sim/design/REFERENCE_LINE.md)。

## GUI 操作

- 点击 `CRUISE`、`PARKING` 或 `E-STOP` 选择任务；点击控制来源按钮或按 `Q` 切换 `AUTO`/`MANUAL`。
- `C`、`K`、`E` 分别切换巡航、泊车和紧急停车；`1`–`7` 切换场景。
- 手动模式下，`W/S` 或上下方向键控制前进/倒车，`A/D` 或左右方向键控制转向，`SPACE` 为制动。
- `P` 暂停/继续，`R` 重置，`ESC` 关闭 GUI。
- 触摸板双指上下/左右滑动平移地图，也可按住鼠标中键拖动。平移后暂停自动跟随；按 `F` 回到车辆中心并恢复跟随，切换场景也会恢复跟随。
- `Ctrl + 滚动` 或 `+`/`-` 缩放。普通鼠标滚轮也用于平移，因为 WSLg 可能将触摸板和滚轮输入映射为相同事件。
- 点击 `EDIT SCENE` 可暂停仿真并编辑自车初始位置和障碍物。选择 `EGO` 后点击地图设置车辆位置，按 `[`/`]` 调整朝向；`ADD` 添加矩形障碍物，`MOVE` 移动障碍物，`DELETE` 删除障碍物。选中障碍物后可用 `ROT +/-` 或 `SIZE +/-` 调整，点击 `APPLY` 应用并重置场景。场景会保持暂停，按 `E`、`P` 或点击 `RESUME` 后继续；`CANCEL` 放弃本次编辑。
- 倒车入库场景可通过车位按钮或 `F1`–`F3` 选择三个车位之一。仿真开始后如需换位，请先暂停；选择新车位会从场景初始位置重新开始。

车辆位置和朝向在相邻状态间按时间插值，相机使用同一显示位置并按经过时间平滑跟随。显示延迟约一个状态周期（通常 50 ms）；遥测和跟踪误差仍使用真实状态。暂停/重置时显示真实位置，消息中断时不外推未知运动。

GUI 需要 Linux 图形显示环境（WSL2 下可使用 WSLg）。无头仿真、ROS 话题/服务和测试不依赖 GUI。

## 巡航与泊车统一控制接口

标准 launch 同时启动泊车控制器和共享控制模式接口：

```bash
ros2 launch lightweight_sim lightweight_sim.launch.py gui:=true
```

无 GUI 时可以先加载泊车场景，再通过 `control_mode` 话题选择 `PARKING` 模式：

```bash
ros2 launch lightweight_sim lightweight_sim.launch.py \
  scenario:=reverse_parking gui:=false
```

泊车模块默认使用 Hybrid A*，同时保留基线规划器供对比。规划器根据车辆当前位置和朝向规划到所选车位，并受可行驶区域、车辆转向限制和障碍物约束。巡航和泊车控制器发布候选控制指令；`controller_manager` 根据当前模式/控制来源进行选择，是最终的控制仲裁器。这些算法用于仿真，不构成量产车辆安全系统。

在 `lightweight_sim/config/default.yaml` 中切换泊车算法：

```yaml
/**/parking_controller_node:
  ros__parameters:
    planner_type: hybrid_astar  # 或 baseline
```

修改源配置后，重新构建并加载工作区，再重启 launch。算法详情和对比说明见 [`parking_module/README.md`](parking_module/README.md)。

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
