# ROS 2 运行架构

当前 ROS 2 迁移采用“保留轻量仿真内核 + ROS 2 节点适配层”的方式：

```text
simulator_node --/vehicle/state, /obstacles--> planner_node
simulator_node --/reference_path-------------> controller_node
planner_node   --/planned_path---------------> controller_node
controller_node --/control_command-----------> simulator_node
```

## 节点

- `simulator_node`：以 20 Hz 推进 `SimulationEngine`，发布车辆状态、障碍物和参考线，并提供 reset/pause/step 服务。
- `planner_node`：以低频周期调用现有 `MotionPlanner`，只保留最新规划请求和结果。
- `controller_node`：以 20 Hz 调用现有 `VehicleController`，没有有效状态、路径或状态超时时自动发布全制动。

## 消息接口

当前节点使用独立的 `lightweight_sim_msgs` 消息包：

- `VehicleState`：车辆位姿、车体速度、横摆角速度和当前执行状态。
- `Path` / `PathPoint`：带航向角和曲率的路径点，并带有规划序号。
- `ObstacleArray`：障碍物几何、速度、航向和类型。
- `ControlCommand`：前轮转角、油门、制动和挡位。

其中 `x/y/yaw` 位于 `map` 坐标系，`vx/vy` 使用车体坐标系，所有物理量使用 SI 单位；目标速度参数仍按现有控制器接口使用 km/h。

## Linux 构建与运行

在 Linux ROS 2 环境中，从仓库根目录执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /mnt/d/AI_tools/vehicle_motion
python3 -m pip install -r lightweight_sim/requirements.txt  # 如果项目环境尚未安装 numpy/pytest
colcon build
source install/setup.bash
ros2 launch lightweight_sim lightweight_sim.launch.py

# Run an isolated instance; /clock remains global and other topics/services use /sim_01.
ros2 launch lightweight_sim lightweight_sim.launch.py namespace:=sim_01

# Start the Pygame GUI as a ROS 2 client of the same simulator instance.
ros2 launch lightweight_sim lightweight_sim.launch.py gui:=true
```

仓库根目录现在包含两个独立 ROS 2 包：`lightweight_sim/` 和 `lightweight_sim_msgs/`，可以直接使用标准 `colcon build` 构建。后续如果迁移到标准 `ros2_ws/src/` 布局，只需将这两个包整体放入 `src/`。

QoS 约定：`/vehicle/state` 和 `/obstacles` 使用 BEST_EFFORT、depth 1；`/control_command` 使用 RELIABLE、depth 1；路径话题使用 RELIABLE + TRANSIENT_LOCAL，保证晚启动节点能够获取最新路径；`/clock` 使用 BEST_EFFORT。

常用检查：

```bash
ros2 topic echo /vehicle/state
ros2 topic echo /planned_path
ros2 topic echo /sim/status --once
ros2 service call /sim/reset std_srvs/srv/Empty {}
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: true}"
```

`/sim/status` 发布当前仿真生命周期状态，包含运行/暂停/结束标志、碰撞/越界/到达结果、仿真步数、仿真时间、场景名和终止原因；该话题使用 RELIABLE + TRANSIENT_LOCAL，晚启动的监控节点也能读取最近状态。

当前节点默认使用仿真时间戳并发布 `/clock`。如果其他节点需要 ROS 仿真时间，应在 launch 或参数中设置 `use_sim_time: true`。

### ROS 2 GUI client

`gui:=true` 启动的是 ROS 2 GUI 客户端，不会创建第二个 `SimulationEngine`。仿真仍由 `simulator_node` 推进，GUI 只订阅状态、障碍物、参考路径、规划路径和 `/sim/status`，并通过服务控制仿真。

GUI 依赖 Pygame；首次使用前执行 `python3 -m pip install -r requirements.txt`。WSL 下会自动优先使用 WSLg 的 X11 桥接；也可以通过 `SDL_VIDEODRIVER` 显式覆盖。GUI 需要 WSLg 或其他 Linux 图形环境；如果只做后台测试，可以不传 `gui:=true`。快捷键：`R` 重置，`P` 暂停/继续，`N` 单步，`+/-` 或鼠标滚轮缩放，`ESC` 退出 GUI。
## Runtime parameter alignment

Both execution paths use the same core `VehicleController`, `MotionPlanner` and `SimulationEngine`. Shared timing and safety defaults are maintained in `engine/runtime_config.py`: physics/control period 0.05 s, planning period 0.5 s, prediction horizon 0.2 s, and the common timeout values. Scenario-specific vehicle, road and target-speed values are carried by `sim/context` in ROS 2; ROS transport and stale-data braking remain adapter behavior.
## Current WSL2 quick start

The current environment uses ROS 2 `lyrical` inside WSL2. Every new terminal must source the ROS environment before `ros2`, `colcon`, or the Python ROS packages are available.

```bash
cd /mnt/d/AI_tools/vehicle_motion
source /opt/ros/lyrical/setup.bash

# The repository is on /mnt/d. Use a regular build here; --symlink-install
# can fail while colcon cleans Python package links on the Windows-mounted drive.
colcon build
source install/setup.bash
```

Start the three-lane figure-eight scenario:

```bash
ros2 launch lightweight_sim lightweight_sim.launch.py \
  scenario:=figure_eight \
  gui:=true \
  steering_profile:=ideal
```

Use `gui:=false` when WSLg or a Linux display is unavailable. The simulation can then be inspected from another WSL terminal:

```bash
source /opt/ros/lyrical/setup.bash
cd /mnt/d/AI_tools/vehicle_motion
source install/setup.bash

ros2 topic list
ros2 topic echo /vehicle/state
ros2 topic echo /planned_path
ros2 topic echo /control_command
ros2 topic echo /sim/status --once
ros2 topic hz /vehicle/state
```

If `ros2: command not found` appears, run `source /opt/ros/lyrical/setup.bash` in that same terminal. If a previous build failed, remove only the generated package artifacts and rebuild:

```bash
rm -rf build/lightweight_sim install/lightweight_sim log
colcon build
source install/setup.bash
```
## Reverse parking scene

The `reverse_parking` scene provides a perpendicular parking slot bounded by
two side walls and a rear wall. It is intended for an external parking
controller rather than the built-in forward tracking controller.

```bash
ros2 launch lightweight_sim lightweight_sim.launch.py \
  scenario:=reverse_parking \
  controller_enabled:=false \
  gui:=true
```

The external controller should publish `lightweight_sim_msgs/msg/ControlCommand`
with `gear: -1` for reverse motion. The simulator publishes the resulting
signed longitudinal velocity in `VehicleState.vx`. Use `controller_enabled:=true`
only when testing the standard forward tracking controller.
