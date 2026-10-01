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

`engine/algorithms/planner/motion_planner.py` 根据当前 run 匹配的 Routing 参考线和障碍物，生成短时域车道级避障路径。ROS planner 节点以异步方式提交规划任务，优先保留最新请求与结果；局部候选受路廊及碰撞间距参数约束。

全局 Routing 选择拓扑车道序列；局部规划可以为绕开障碍物暂时偏离目标车道。当前局部规划以几何路径生成为主，并非完整行为规划或 ST 时空优化：不提供通用动态障碍物轨迹预测、信号灯规则处理或完整纵向速度规划。历史 `dp_path_plan.py` 与 `qp_path_plan.py` 不在默认执行链中。

若当前 run 缺少匹配的 Routing 参考线或新鲜局部路径，`safe_stop_node` 请求制动；系统不以仿真器道路路径代替缺失的 Routing 参考线。

## 控制与车辆模型

横向控制器采用离散动态自行车误差模型、Riccati LQR 反馈和曲率前馈；通过单调弧长进度及局部参考投影维持分支连续性，适用于闭环交叉处的路径跟踪。转向执行器支持理想模式，以及显式假设的延迟/惯性/速率限制模式；后者不是实车标定。

纵向控制采用 PID。速度限值、目标限速比例、弯道/横向加速度约束和车辆参数由 `config/default.yaml` 配置；共享周期与默认值集中在 `engine/runtime_config.py`。默认仿真物理步长和规划/控制周期均为 0.05 s（20 Hz）。

### 控制器扩展结构

横向和纵向分别遵循 `LateralController`、`LongitudinalController` 抽象接口（`engine/algorithms/controller/base.py`）。`VehicleController` 组合两个接口实例，统一输出前轮转角、油门和刹车；通过 `lateral_controller=`、`longitudinal_controller=` 可以注入新的控制算法，默认仍为 LQR + PID。

```text
LateralController
└── ProjectedLateralController（公共路径投影、误差计算）
    ├── LateralLQRController（包含内部 LQR 实现层）
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
