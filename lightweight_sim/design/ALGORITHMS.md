# 轻量级仿真算法与模块职责

本文说明当前轻量级仿真实际使用的算法链。仓库根目录的 [`ALGORITHMS.md`](../../ALGORITHMS.md) 是 CARLA 历史算法说明，不代表本仿真器当前的默认执行路径。，不代表本仿真器当前的默认执行路径。

## 运行链路

```text
场景与地图
  -> Routing：车道拓扑 A*
  -> 参考线节点：拼接、采样和几何约束
  -> 局部规划：基于参考线与障碍物生成短路径
  -> 横纵向控制
  -> controller_manager 仲裁 + safe_stop 安全监督
```

核心算法在 `engine/` 中；ROS 2 节点负责消息、定时器和生命周期适配。整体节点关系见 [ROS2.md](ROS2.md)，模块边界见 [DESIGN.md](DESIGN.md)。

## Routing

`engine/routing/` 用有向车道边和显式 successor 关系表达可行驶拓扑。边包含车道/道路身份、中心线、边界、限速和 maneuver 元数据。A* 在车道边图上搜索；转弯、换道、掉头代价可配置。请求可指定起终点、车道、策略以及是否按闭环路线规划。

Routing 只负责拓扑路径，不负责动态障碍物避让。内置 JSON 地图位于 `config/maps/`；场景切换会创建新的 request/run 标识，避免旧路线被新场景误用。详见 [ROUTING.md](ROUTING.md)。

## 参考线生成

`engine/reference_line/` 根据 RoutePlan 中的有序边序列回查地图，校验 lane identity 和拓扑连接，拼接并重采样路线几何，处理路口转角并限制曲率，再计算航向、曲率及车道/道路可行驶边界。此模块生成连续、地图约束的车道参考线，不负责障碍物决策。详见 [REFERENCE_LINE.md](REFERENCE_LINE.md)。

## 局部路径规划

ROS `planner_node` 通过 `local_planner_algorithm` 选择算法，默认 `dp_qp`，可改为 `baseline` 对比。两者继承 `CorridorMotionPlanner`，共用异步请求队列、参考线边界和最终矩形碰撞检查，不互相继承。原 `RouteAwareMotionPlanner` 名称保留为基线算法的兼容别名。

`BaselinePathPlanner` 保留原车道中心候选算法：生成固定目标横向偏移的五次多项式过渡，在候选中选择安全路径。`local_transition_distance_m` 仅控制这一算法的过渡距离。

`DPQPPathPlanner` 的执行链为：

1. 以实际车辆位置投影作为规划起点，按参考线实际弧长建立 S-L 坐标，并从车辆航向、速度和横摆角速度计算起始横向导数。
2. `engine/algorithms/planner/dp_qp.py::DP_algorithm` 在可行驶边界内建立横向格点（默认 0.5 m），纵向最大间隔 8 m，起点和障碍物附近加密至 4 m。状态包含横向偏移和斜率（-0.2、0、0.2），节点二阶导数为零；相邻状态用匹配两端导数的五次多项式连接。先排除不可达状态对，再批量计算代价与安全约束。偏移、一二阶导数及靠近障碍物的软余量构成代价，不硬编码车道中心或左侧偏好。
3. DP 与 QP 共用斜率、道路边界及车身五个纵向位置的约束；障碍物按对应车身位置的纵向占用收紧边界，避免把前后车身余量同时施加在整个膨胀区。根据 DP 绕障方向构造凸走廊。QP 显式保留 l、dl、ddl 和每段 jerk，通过稀疏连续性等式精确积分，得到 C2 横向曲线，避免全时域积分矩阵的病态问题。起点前 12 m 加密至 0.5 m，障碍物附近加密至 1 m，远端最大间隔由 `qp_station_step_m` 控制。上一帧有效路径在新弧长坐标中作为软连续性目标；OSQP 从 DP 曲线初始化，执行不可行检测、迭代限制和默认 0.08 s 求解时间限制，之后独立检查所有约束残差。QP 额外约束二阶空间导数以改善可跟踪性；起点已超过舒适阈值时允许逐渐恢复，不要求状态瞬间跳变。这仍不保证任意可行 DP 路径均有可行 QP 解。
4. 输出按 `local_path_sampling_resolution_m` 增密，默认最大参考线弧长间距 0.5 m；重算笛卡尔航向和曲率。再次检查障碍物碰撞及车身四角是否超出物理道路边界。

`last_qp_diagnostics` 提供 OSQP 状态、迭代数、求解耗时及变量数；失败日志同时记录自车位置和状态。`last_status` 提供 `solved`、`dp_infeasible`、`corridor_infeasible`、`qp_failed`、`validation_failed` 等诊断。任一阶段失败都返回空路径，触发现有安全停车；不会用走廊中线冒充 QP，也不会自动切换基线算法。历史 CARLA 兼容工具 `dp_path_plan.py`、`qp_path_plan.py` 不参与这条链；其点障碍物、固定道路边界和失败回退不适合作为正式实现直接调用。

两种算法都需要参考线；正式算法在提供可行驶左右边界时不依赖明确车道线。缺少边界时仍采用名义车道宽度推导道路范围。上游 Routing 仍使用车道拓扑，因此这次改动不等于完整的无结构道路导航。参考线分支匹配仍使用最近投影，交叉或重叠参考线需要后续加入有状态的分支选择。

这是横向几何路径 DP+QP；障碍物按当前矩形处理，不包含动态目标预测和纵向 ST 速度优化。空间 jerk 优化不等同于变速车辆的时间 jerk 最优。QP 限制相对参考线的斜率（约 0.35 rad），较大起始航向偏差会失败停车。边界和碰撞约束采用密集采样及保守矩形近似，并非连续空间的形式化安全证明。

## 控制与车辆模型

横向控制器采用离散动态自行车误差模型、Riccati LQR 反馈和曲率前馈；通过单调弧长进度及局部参考投影维持分支连续性，适用于闭环交叉处的路径跟踪。转向执行器支持理想模式，以及显式假设的延迟/惯性/速率限制模式；后者不是实车标定。

纵向控制采用 PID。速度限值、目标限速比例、弯道/横向加速度约束和车辆参数由 `config/default.yaml` 配置；共享周期与默认值集中在 `engine/runtime_config.py`。默认仿真物理步长和规划/控制周期均为 0.05 s（20 Hz）。

### 控制器扩展结构

横向和纵向分别遵循 `LateralController`、`LongitudinalController` 抽象接口（`engine/algorithms/controller/base.py`）。`VehicleController` 组合两个接口实例，统一输出前轮转角、油门和刹车；通过 `lateral_controller=`、`longitudinal_controller=` 可以注入新的控制算法，默认仍为 LQR + PID。

```text
LateralController
└── ProjectedLateralController（公共路径投影、误差计算）
    ├── LateralLQRController（lat_lqr.py，直接实现 Riccati LQR）
    └── LateralMPCController

LongitudinalController
└── LongitudinalPIDController
```

横向接口提供 `control()`、`set_path()`、`reset_tracking()`、`configure_actuator()` 和 `configure_tracking()`；纵向接口提供 `control()`、`set_target()` 和 `reset()`。横向输出单位为 rad，纵向输出为 m/s²，目标速度接口使用 km/h。自定义实现需遵守车辆限值及统一周期；`VehicleController` 会设置周期、车辆限值、目标速度和公共跟踪/执行器配置，算法专有配置由实例自身负责。

`ProjectedLateralController` 提取公共参考线处理，具体算法实现 `control_from_error()`。MPC 与 LQR 属于同级横向实现，共享 `lateral_model.py` 的车辆模型，不互相继承或调用对方的控制算法。

### CARLA MPC 迁移

`lat_mpc.py` 迁移自 `carla_legacy/controller/Controller.py` 的 `Lateral_MPC_controller`：保留预测矩阵、仿射曲率扰动、阶段/终端代价和约束 QP 的组织方式，去除 CARLA actor 依赖。参数统一为 `(a,b,m,Cf,Cr,Iz)`，转角使用 rad，离散周期使用 `ts`；修正原曲率扰动中的 `a*Cf+b*Cr` 为 `a*Cf-b*Cr`。

`N` 为预测步数、`P` 为控制步数，满足 `1 <= P <= N`，每个控制步只有一个转角变量；超过控制时域保持最后一个转角，且每个预测步的控制代价均计入。`Q`、`F` 分别为阶段和终端误差权重，`R` 为转角权重。默认 `N=6/P=2`，`F=Q`；预测时域内速度与当前投影点曲率保持常数。

`discretization="plant"` 使用共享的仿真子步模型，MPC 默认按运行配置的 0.0025 s 最大子步离散以避免低速显式 Euler 不稳定（可通过 `max_substep_s` 配置）；`"bilinear"` 使用 CARLA 的双线性离散方法。动态执行器模式增加实际前轮角及延迟队列状态，要求使用 `plant`，保留角度约束 `[-max_steer,max_steer]`。预测模型包含线性的转向惯性与传输延迟；仿真器的非线性转向速率限制仍由执行器实际执行，并不是 QP 中的显式约束。

`solver="auto"` 在安装 `cvxopt` 时使用其 QP 求解器，否则使用内置 NumPy 活跃集求解器求解相同的转角上下界 QP；也可指定 `"numpy"` 或 `"cvxopt"`。MPC 不再回退成 LQR。求解失败不更新命令历史，ROS 控制节点发布制动；动态执行器模式锁定故障并等待新运行重置，避免继续使用失去时间对齐的历史。

可通过 `VehicleController(controller_type="MPC_controller", mpc_params={"N": 12, "P": 4, "solver": "numpy"})` 配置。ROS 控制节点提供 `mpc_prediction_steps`、`mpc_control_steps`、`mpc_solver` 参数，默认值分别为 6、2、auto。`solver_status`、`solver_backend`、`last_solution` 和 `predicted_states` 提供求解诊断；曲率补偿包含在预测中，不单独叠加前馈转角。


## 仲裁、安全与验证

`controller_manager` 是最终 `control_command` 的唯一发布者，负责巡航、手动和泊车控制候选的仲裁，并在切换或命令缺失/过期时制动。`safe_stop_node` 独立监视当前 run 的 Routing 参考线与局部规划结果是否就绪、新鲜。该机制是仿真安全监督，不构成安全认证。

指标实现位于 `engine/analysis/`。测试分为无需 ROS 的 Python 单元测试和依赖 ROS 的集成/生命周期测试；CI 默认每次变更运行前者，ROS 工作流由相关路径变化触发，也可手动触发。

## 当前边界

车辆动力学为简化模型，地图为合成地图，A* 不考虑动态障碍物。通用行为决策、动态目标预测、完整纵向 ST 规划和实车标定不属于当前基线。仿真通过不等同于实车可部署。
