# 轻量级车辆规划控制仿真器设计文档

## 1. 文档范围

本文件描述 lightweight_sim 当前已经实现的架构、数据接口、仿真时序和可扩展边界。

该目录是一个脱离 CARLA 的二维车辆仿真环境，适合快速验证车辆模型、路径跟踪、局部避障和规划控制接口。根目录中的 controller、planner 和 code_*.py 仍属于 CARLA 侧历史实现，轻量环境不直接依赖 CARLA API。

## 2. 当前实现状态

| 子系统 | 当前实现 | 说明 |
| --- | --- | --- |
| 道路 | 直线、圆弧、waypoints | 多段道路检查连接间隙 |
| 参考线 | 航向角、曲率、可选平滑 | 未安装 cvxopt 时自动几何回退 |
| 自车模型 | 运动学自行车，动力学自行车可选 | 默认使用运动学模型 |
| 控制器 | 有界几何横向控制 + PID 纵向控制 | LQR/MPC 类接口保留 |
| 局部规划 | 最新请求优先的异步车道级避障 | 线程不阻塞主循环 |
| 碰撞检测 | 旋转矩形 SAT | 每个物理子步检查 |
| 可视化 | pygame 2D 俯视图和 HUD | 渲染频率独立于物理步长 |
| 测试 | lightweight_sim/tests 核心回归测试 | CARLA 测试需要单独环境 |
| 运行日志 | 进程级线程安全日志 | 每次运行写入 temp/logs/run_*.log |
| 配置 | Python ScenarioConfig 和场景工厂 | 当前没有 YAML 解析器 |

## 3. 目录结构

    lightweight_sim/
    ├── main.py                         命令行入口和场景选择
    ├── simulator/
    │   ├── data_types.py               状态、道路、障碍物和场景配置
    │   ├── world.py                    道路生成、参考线和道路边界
    │   ├── vehicle.py                  运动学/动力学自车模型
    │   ├── obstacle.py                 障碍物运动和 SAT 碰撞
    │   ├── engine.py                   固定步长仿真引擎
    │   └── app.py                      pygame 主循环、场景工厂和交互
    ├── algorithms/
    │   ├── controller/                 横纵向控制器
    │   ├── planner/                    局部规划器和历史 DP/QP 模块
    │   └── utils/                      Frenet、几何、曲率和参考线工具
    ├── visualization/                  pygame 渲染和 HUD
    ├── tests/                          轻量环境回归测试
    └── temp/logs/                      运行诊断日志（自动创建）

## 4. 分层架构

    SimulatorApp
        ├── 输入、控制调用、异步规划请求
        ├── SimulationEngine
        │   ├── World
        │   ├── EgoVehicle
        │   └── ObstacleManager
        ├── VehicleController
        │   ├── LateralLQRController / LateralMPCController
        │   └── LongitudinalPIDController
        ├── MotionPlanner（后台线程，最新请求优先）
        └── Renderer / HUD

依赖方向保持单向：

- 数据类型不依赖 pygame 或 CARLA；
- 仿真器核心不依赖 GUI 才能完成状态推进；
- 控制器和规划器只接收纯 Python 状态、路径和障碍物；
- pygame 只位于 app、renderer 和 hud 相关模块。

## 5. 数据接口和单位

### 5.1 VehicleState

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| x、y | m | 世界坐标 |
| phi | rad | 车身航向角 |
| vx、vy | m/s | 车体坐标系纵向/横向速度 |
| r | rad/s | 横摆角速度 |
| steer | rad | 前轮实际转角 |
| accel | m/s² | 当前纵向加速度 |
| timestamp | s | 仿真时间戳 |

VehicleState 提供 speed 和 speed_kmh 属性，分别返回 m/s 和 km/h。

### 5.2 ControlCommand

| 字段 | 范围 | 含义 |
| --- | --- | --- |
| steer | [-0.5, 0.5] rad，最终由参数限制 | 前轮物理转角 |
| throttle | [0, 1] | 油门归一化值 |
| brake | [0, 1] | 刹车归一化值 |

引擎使用以下映射：

    acceleration = throttle * max_accel - brake * max_decel

目标速度在场景配置和 PID 接口中使用 km/h；车辆物理状态始终使用 m/s。

### 5.3 RoadDef 和 ScenarioConfig

道路由 RoadSegment 列表组成，支持：

- straight：length、heading、start、resolution；
- arc：radius、angle、center、start_angle、resolution；
- waypoints：points。

ScenarioConfig 当前包含 road、ego_start_x、ego_start_y、ego_start_phi、ego_start_speed、target_speed、obstacles、controller、destination 和 vehicle_model。vehicle_model 取值为 kinematic 或 dynamic。

## 6. 仿真时序

默认物理步长为 0.05 s，即 20 Hz。渲染通常以 60 FPS 独立运行。

每次 SimulationEngine.step：

1. 限制转角、油门和刹车范围；
2. 根据车辆速度计算子步数量，避免高速时单步位移过大；
3. 调用选定的运动学或动力学模型；
4. 更新障碍物位置；
5. 使用旋转矩形 SAT 检测碰撞；
6. 检查道路边界和终点；
7. 更新时间戳、步计数和终止状态。

engine.is_done 在发生碰撞、越界或到达终点时为真。

## 7. 车辆模型

### 7.1 运动学自行车模型

使用轴距 L = a + b：

    x_next = x + v_next * cos(phi_next) * dt
    y_next = y + v_next * sin(phi_next) * dt
    phi_next = phi + v_next / L * tan(steer) * dt
    v_next = max(0, v + accel * dt)

该模型稳定、计算量小，适合默认巡航和路径跟踪调试。

### 7.2 动力学自行车模型

动力学模型保留 vx、vy 和 r 状态，并使用前后轮侧偏刚度计算横向速度和横摆角速度变化。低速时自动退回运动学更新，避免除以过小纵向速度。

该模型是简化动力学模型，不等同于高保真轮胎模型；极限工况验证仍需使用 CARLA 或实车数据校准。

## 8. 控制器

### 8.1 横向控制

LateralLQRController 当前采用有界几何误差反馈：

- 预测一个控制周期后的车辆位置；
- 从当前匹配索引开始寻找最近参考点；
- 计算有符号横向误差 ed 和包角后的航向误差；
- 加入参考曲率前馈；
- 将输出限制在最大前轮转角内。

该类保留 LQR 控制器的调用接口，但当前不是完整的矩阵 Riccati LQR。LateralMPCController 当前作为兼容接口复用这一稳定基线，便于先完成闭环仿真和接口联调。

### 8.2 纵向控制

PID 的目标速度使用 km/h，实际速度进入误差计算前转换为 km/h。PID 输出为 m/s²，并限制在最大加速度 3.0 m/s² 和最大减速度 6.0 m/s²。门面控制器再将其映射为归一化油门和刹车。

## 9. 局部规划器

当前 MotionPlanner 的设计目标是实时性和可恢复性：

- 请求队列最多保留一个任务；
- 新请求到达时丢弃旧请求；
- 结果队列最多保留最新结果；
- 规划在后台线程执行，主循环只轮询结果；
- 以 Frenet 横向偏移近似选择可行车道；
- 在障碍物附近对车道中心做平滑过渡；
- 失败时返回空路径，调用侧回退到车道参考线。

当前规划器是车道级避障基线，尚未实现完整的纵向速度规划、时间碰撞预测、道路拓扑搜索或多目标代价优化。

dp_path_plan.py 和 qp_path_plan.py 是保留的历史算法模块。QP 模块对 cvxopt 使用可选导入；未安装时返回受边界约束的回退结果，但这些历史模块不是当前 MotionPlanner 的默认执行链。

## 10. 几何和碰撞

- Frenet 最近点采用线段投影，而不是只匹配离散节点；
- 航向角计算使用角度展开，避免跨越 pi 和 -pi 时跳变；
- 道路段之间的间隙超过 0.5 m 时直接报错；
- 自车和障碍物均按旋转矩形处理；
- SAT 先做 AABB 粗筛，再做分离轴检测。

## 11. GUI 和场景

GUI 初始化时默认启用自动模式，自动控制链负责横向跟踪、纵向 PID 和后台局部规划；Q 键仍可切换手动模式。

SimulatorApp 内置四个场景工厂：

| 名称 | 内容 |
| --- | --- |
| default | 200 m 两车道直道 |
| obstacle | 两车道直道静态障碍物 |
| three_lane | 三车道双障碍连续避障 |
| curve | 直道接 90 度弯道 |

交互、启动命令和依赖请见根目录 README.md。

## 12. 测试和验证策略

轻量环境测试位于 lightweight_sim/tests/test_core.py，覆盖：

- 固定步长和控制量限制；
- 动力学模型数值有限性；
- 旋转矩形碰撞；
- 断开道路检测；
- 多车道避障规划。

推荐验证顺序：

1. 运行 AST 检查和核心单元测试；
2. 运行默认直道自动驾驶；
3. 运行障碍物和三车道场景；
4. 运行弯道和动力学模型；
5. 使用日志中的 ed、ephi、速度和碰撞状态评估闭环质量。

### 12.1 运行日志

日志由 simulator/logging_utils.py 统一创建，路径为仓库根目录下的 temp/logs。每次进程启动生成一个带微秒时间戳的文件，日志等级包括 DEBUG、INFO、WARNING 和 ERROR。主循环、仿真引擎和后台规划线程共用同一个线程安全 logger。

引擎第 1 步和每 100 步记录一次状态；规划请求记录障碍物位置，规划结果记录轨迹首/中/末点，控制器在规划结果生效后的短窗口记录实际参考点和局部误差；碰撞记录障碍物 ID/位置，越界记录道路距离和边界阈值。碰撞、越界、终点、规划失败和规划超时只在关键事件发生时记录，避免每个物理子步写盘影响实时性。
## 13. 已知限制和后续路线

1. 增加统一的 YAML/JSON 场景加载器；
2. 增加纵向 ST 速度规划和时距安全约束；
3. 将车道边界约束直接纳入规划器；
4. 恢复真正的线性化 LQR 和 QP-MPC，并补充求解失败监控；
5. 增加传感器抽象、观测噪声和 Gymnasium 风格环境接口；
6. 增加 CSV logger、轨迹回放和 matplotlib 分析；当前已有诊断日志，但尚未提供 CSV 导出；
7. 使用 CARLA 或实车轨迹标定动力学参数；
8. 将控制器和规划器参数外置，支持批量实验。

设计原则是：先保证单位一致、时间确定、失败可恢复，再逐步增加高保真模型和复杂算法。