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

## 初始消息协议

为了让第一阶段保持纯 Python `ament_python` 包，路径和障碍物暂时使用 `std_msgs/msg/Float64MultiArray`，协议已在 `ros_nodes/protocol.py` 中版本化：

- path: `[version, sequence, point_count, x, y, theta, kappa, ...]`
- obstacle: `[id, x, y, length, width, speed, heading, ...]`
- control: `[steer_rad, throttle_0_to_1, brake_0_to_1]`

车辆状态使用 `nav_msgs/msg/Odometry`，其中 `x/y/yaw` 在 `map`，`vx/vy` 在 `base_link`，单位为 SI。

这是一层过渡协议。下一阶段应将它们替换为 `lightweight_sim_msgs` 自定义消息，同时保持 topic 语义不变。

## Linux 构建与运行

在 Linux ROS 2 环境中，从仓库根目录执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 -m pip install -r requirements.txt  # 如果项目环境尚未安装 numpy/pytest
colcon build --symlink-install
source install/setup.bash
ros2 launch lightweight_sim lightweight_sim.launch.py
```

常用检查：

```bash
ros2 topic echo /vehicle/state
ros2 topic echo /planned_path
ros2 service call /sim/reset std_srvs/srv/Empty {}
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: true}"
```

当前节点默认使用仿真时间戳并发布 `/clock`。如果其他节点需要 ROS 仿真时间，应在 launch 或参数中设置 `use_sim_time: true`。
