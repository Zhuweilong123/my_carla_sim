# 轻量级仿真器功能清单

## 当前定位

lightweight_sim 是一个脱离 CARLA 的二维车辆规划控制仿真环境，重点是快速调试、确定性步进和可恢复的闭环运行。

## 已实现功能

| 功能 | 状态 | 实现位置 |
| --- | --- | --- |
| 直线、圆弧和 waypoints 道路 | 已实现 | simulator/world.py |
| 多段道路连接检查 | 已实现 | simulator/world.py |
| 参考线航向角和曲率 | 已实现 | algorithms/utils/geometry.py |
| 可选参考线平滑 | 已实现 | algorithms/utils/reference_line.py |
| Frenet 线段投影 | 已实现 | algorithms/utils/frenet.py |
| 运动学自行车模型 | 已实现，默认 | simulator/vehicle.py |
| 简化动力学自行车模型 | 已实现，可选 | simulator/vehicle.py |
| 固定 20 Hz 物理步进 | 已实现 | simulator/engine.py |
| 高速子步进 | 已实现 | simulator/engine.py |
| 旋转矩形 SAT 碰撞检测 | 已实现 | simulator/obstacle.py |
| 越界和终点终止状态 | 已实现 | simulator/engine.py |
| 有界几何横向跟踪 | 已实现 | algorithms/controller/lat_lqr.py |
| PID 纵向控制 | 已实现 | algorithms/controller/lon_pid.py |
| LQR/MPC 兼容控制器切换 | 已实现 | algorithms/controller/combined.py |
| 最新请求优先异步规划 | 已实现 | algorithms/planner/motion_planner.py |
| 车道级横向避障 | 已实现 | algorithms/planner/motion_planner.py |
| pygame 俯视图和 HUD | 已实现 | visualization/ |
| 键盘手动驾驶 | 已实现 | simulator/app.py |
| 四个内置场景 | 已实现 | simulator/app.py |
| 核心回归测试 | 已实现 | lightweight_sim/tests/ |
| 运行诊断日志 | 已实现 | simulator/logging_utils.py、temp/logs |

## 当前控制链

    场景配置
        -> World 生成参考线
        -> MotionPlanner 后台车道级避障
        -> LateralLQRController / LateralMPCController
        -> LongitudinalPIDController
        -> ControlCommand
        -> SimulationEngine
        -> VehicleState 和碰撞状态

默认物理步长为 0.05 s。目标速度使用 km/h，车辆速度、加速度和位移使用 SI 单位。

## 控制器说明

当前横向控制器使用预测位置、横向误差、航向误差和参考曲率前馈，并进行最大转角限制。

当前的 MPC 类保留接口兼容性，但内部复用稳定的有界几何横向控制基线，不是完整的矩阵 QP-MPC。历史 QP 模块仍保留，cvxopt 为可选依赖。

纵向 PID 输出物理加速度，再由门面控制器转换成归一化油门和刹车。

## 规划器说明

当前规划器使用后台线程，不会阻塞 pygame 主循环。请求队列和结果队列都只保留最新任务，避免规划延迟导致旧轨迹覆盖新轨迹。

规划器根据障碍物相对参考线的位置选择可行车道，并使用平滑横向过渡生成局部轨迹。它目前不包含：

- 纵向 ST 速度规划；
- 基于时间的碰撞预测；
- 道路拓扑搜索；
- 完整多目标代价优化；
- 传感器观测噪声。

## 内置场景

| 场景名 | 内容 |
| --- | --- |
| default | 200 m 两车道直道 |
| obstacle | 两车道直道和静态障碍物 |
| three_lane | 三车道双障碍连续避障 |
| curve | 直道连接 90 度弯道 |

启动示例：

    python -m lightweight_sim.main --scenario three_lane

## GUI 按键

| 按键 | 功能 |
| --- | --- |
| Q | 手动和自动模式切换 |
| W、S | 油门和刹车 |
| A、D | 手动转向 |
| Space | 全刹车 |
| M | LQR 和 MPC 兼容控制器切换 |
| R | 重置 |
| P | 暂停 |
| 鼠标滚轮、加减号 | 缩放 |
| ESC | 退出 |

## 已知边界

当前没有 YAML 或 JSON 场景加载器，场景通过 Python 的 ScenarioConfig 和 app 中的工厂方法创建。

当前没有独立传感器层、Gymnasium 环境接口、CSV 日志导出和 matplotlib 分析脚本。运行诊断日志会自动写入 temp/logs，每次进程生成一个带时间戳的 run_*.log 文件。pygame 运行需要图形环境；SimulationEngine 可以在无窗口环境中直接使用。

## 后续优先级

1. 统一场景文件加载和参数校验；
2. 增加纵向速度规划和安全时距；
3. 恢复真正的线性化 LQR 和 QP-MPC；
4. 增加传感器、噪声和 Gymnasium 接口；
5. 增加日志、回放、批量实验和性能指标；
6. 使用 CARLA 或实车数据标定动力学参数。

安装和操作细节见仓库根目录 README.md，架构细节见 lightweight_sim/DESIGN.md。