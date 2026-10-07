# 配置入口

日常修改从 `default.yaml` 开始。标准 launch 自动加载以下六个 ROS 参数文件，修改源码配置后需重新构建并重启节点：

| 文件 | 参数数 | 用途 |
| --- | ---: | --- |
| `default.yaml` | 23 | 场景、算法选择、速度策略、地图、窗口及初始交互模式 |
| `vehicle.yaml` | 15 | 车辆物理模型、积分子步和转向执行器 |
| `algorithms.yaml` | 46 | 路径与 ST 速度 DP+QP、路径接管、横纵向控制、Routing 和参考线几何 |
| `system.yaml` | 48 | 时钟、接口名称、调度周期、仲裁与安全超时 |
| `compatibility.yaml` | 11 | 启动/旧上下文备用值及未启用的预留参数 |
| `baseline.yaml` | 5 | baseline 对比规划器的专用参数 |

加载顺序是 `system → vehicle → algorithms → baseline → compatibility → default`。同名节点参数以后加载的值为准；当前各文件没有重复的节点参数。节点代码声明的默认值仍作为缺省值，部分共享默认值位于 `engine/runtime_config.py`。

所有节点配置使用 `/**/节点名` 选择器，例如 `/**/controller_node`，同时匹配根命名空间和任意层级的 namespace。通配只作用于命名空间，不会把一个节点的参数加载到其他节点。launch 读取场景默认值时也使用这一选择器。

## 常用调整

- 场景和执行器模式：修改 `default.yaml` 的 `scenario`、`steering_profile`。launch 默认值读取这两项，显式命令行参数优先，例如 `scenario:=figure_eight steering_profile:=assumed`。
- 速度：修改仿真端的场景/路段限速、`target_speed_ratio` 和 `max_lateral_accel_mps2`。独立 `speed_planner_node` 将限速、曲率、障碍物及终点转为 ST 约束，纵向控制跟踪速度参考；GUI 的目标速度仅是显示备用值。
- ST 速度规划：`algorithms.yaml` 的 `/**/speed_planner_node` 设置时域、格点、舒适加减速度、jerk、停车余量和求解预算；`system.yaml` 设置提交周期和时效。关闭控制端 `speed_planning_enabled` 对比旧标量速度策略时，同步关闭安全监督的 `require_speed_plan`。
- 算法：在 `default.yaml` 选择 `local_planner_algorithm`、`controller` 和泊车 `planner_type`，再调整 `algorithms.yaml` 中对应参数。
- 地图：Routing 和参考线节点的 `map_file` / `map_dir` 应保持一致；`map_file` 非空时优先使用单个地图，目录配置不参与该次加载。
- 窗口和初始模式：修改 GUI 的尺寸、帧率、`initial_mode`、`initial_control_source`；渲染帧率不改变物理步长。仲裁器的启动备用模式在 `system.yaml` 中。

## 生效条件

`vehicle.yaml` 中的三个动态执行器参数只在 `steering_profile=assumed` 时影响响应。`algorithms.yaml` 的 `dynamic_lateral_r` 仅用于动态执行器补偿；理想执行器使用 `lateral_r`。纵向积分增益为零时，积分分离阈值不影响积分输出。

`baseline.yaml` 的五项不参与当前 `dp_qp` 路径计算，但始终加载以支持切换到 baseline。baseline 的两个 `local_obstacle_longitudinal_*` 当前比较参考点索引差，只有约 1 m 的参考点间距时才近似米数。

`compatibility.yaml` 中的车道、车辆尺寸、控制步长和目标速度主要是启动/旧上下文备用值，正常运行由仿真端 `sim/context` 覆盖。参考线的 `max_lateral_deviation_m` 和 `boundary_margin_m` 当前仅校验/保存，不改变生成结果；保留原值以兼容已有配置。

调整仿真端 `physics_dt` 时，控制端备用步长无需同时修改：新运行激活后，LQR/MPC 和 PID 从场景上下文采用实际物理周期。`assumed` 执行器的 `steering_delay_s` 仍必须为实际周期的整数倍。

测试从上述六层配置读取参数；实际 ROS 启动验收读取构建后的安装配置，并核对 `sim/context`。调整速度策略无需同步修改 `RuntimeConfig` 的缺省值。速度跟踪质量预算位于 `tests/acceptance.yaml`：起步 4 s 后，RMS、95 分位和最大误差分别按配置路段目标速度的 1.5%、2.5% 和 5% 计算，同时受 0.6、1.0 和 2.0 km/h 的绝对上限约束，取两者较小值。提高限速不会自动放宽原有质量标准。障碍场景使用直线路段限速；目标速度为 60 km/h 时最终预算为 0.6、1.0 和 2.0 km/h。预算只控制测试判定，不参与车辆控制；质量政策需要调整时，应单独修改该文件并运行 ROS 验收。

## 系统配置与单独启动

标准 launch 最后固定各节点的 `use_sim_time`，以及泊车输出 `control_command/parking`。这些同名值在 `system.yaml` 中供单独启动节点使用，修改该文件不会覆盖标准 launch 的固定设置。

各层超时检查的对象和时钟不同，不要合并：仿真端检查最终命令，仲裁端检查候选命令与安全心跳，控制端检查状态/路径，安全监督检查规划接收年龄。物理步长、规划/控制周期、结果轮询周期分别控制不同阶段。

直接使用 `ros2 run ... --ros-args --params-file config/default.yaml` 只会加载常用设置。要复现完整配置，需要按上述顺序重复指定六个 `--params-file`，并使用构建后的安装路径；推荐使用标准 launch。
