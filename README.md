# Vehicle Motion 仿真环境使用指南

## 1. 项目用途

本仓库包含两套环境：

- lightweight_sim：脱离 CARLA 的二维规划控制仿真器，适合快速调试；
- `carla_legacy/`：归档的 CARLA 原始高保真仿真实现，需要 CARLA 运行时和对应 Python API。

本文主要介绍 lightweight_sim。

CARLA 历史源码和测试已统一归档到 `carla_legacy/`，当前轻量仿真与 ROS 2 开发不依赖该目录。

## 2. 安装依赖

推荐使用 Python 3.10 至 3.13。Python 3.14 下建议使用 pygame-ce，因为它通常比原版 pygame 更容易获得预编译包。

在仓库根目录执行：

    python -m pip install -r requirements.txt

如果使用 Conda：

    conda run -n hello_agents python -m pip install -r requirements.txt

运行日志会自动写入仓库根目录的 temp/logs，每次启动生成一个带时间戳的 run_*.log 文件。

cvxopt 是可选依赖。当前默认规划链不要求它；只有直接使用历史 QP 模块或启用 QP 参考线平滑时才需要安装。

## 3. 启动仿真器

从仓库根目录执行：

    python -m lightweight_sim.main --scenario three_lane

可用场景：

| 参数 | 说明 |
| --- | --- |
| default | 200 m 两车道直道 |
| obstacle | 两车道直道静态障碍物 |
| three_lane | 三车道双障碍连续避障，默认场景 |
| curve | 直道接 90 度弯道 |

示例：

    python -m lightweight_sim.main --scenario default
    python -m lightweight_sim.main --scenario obstacle
    python -m lightweight_sim.main --scenario curve

如果从 lightweight_sim 目录内部运行，建议仍然使用模块方式，并确保仓库根目录在 Python 搜索路径中。

## 4. GUI 操作

程序启动后默认进入 AUTO 自动驾驶模式；按 Q 可切换到手动模式。

| 按键 | 功能 |
| --- | --- |
| Q | 切换手动 / 自动模式 |
| W 或上方向键 | 手动增加油门 |
| S 或下方向键 | 手动刹车 |
| A、D 或左右方向键 | 手动转向 |
| Space | 全刹车 |
| M | 切换 LQR / MPC 兼容控制器 |
| R | 重置当前场景 |
| P | 暂停 / 继续 |
| 鼠标滚轮、加减号 | 缩放视图 |
| ESC | 退出 |

进入自动模式后，控制器使用横向路径跟踪和纵向 PID；局部规划器在后台线程周期性处理障碍物并更新参考轨迹。

## 5. 自定义场景

当前场景使用 Python 数据类，不是 YAML 文件。最小示例：

    from lightweight_sim.simulator.data_types import RoadDef, RoadSegment, ScenarioConfig
    from lightweight_sim.simulator.engine import SimulationEngine

    road = RoadDef(
        segments=[
            RoadSegment(
                type="straight",
                params={"length": 300, "heading": 0, "start": (0, 0)},
            )
        ],
        lane_width=3.5,
        num_lanes=3,
    )

    config = ScenarioConfig(
        name="my_test",
        road=road,
        ego_start_x=20.0,
        ego_start_y=0.0,
        ego_start_phi=0.0,
        ego_start_speed=8.0,
        target_speed=40.0,
        obstacles=[
            {
                "id": 1,
                "x": 80.0,
                "y": 0.0,
                "length": 4.5,
                "width": 2.0,
                "speed": 0.0,
                "heading": 0.0,
            }
        ],
        controller="LQR_controller",
        vehicle_model="kinematic",
    )

    engine = SimulationEngine(config)
    state = engine.step()

道路段的连接点必须连续；相邻段间隙超过 0.5 m 会直接抛出异常。车辆初始速度使用 m/s，目标速度使用 km/h。

## 6. 直接使用核心仿真

核心引擎不要求 pygame：

    from lightweight_sim.simulator.data_types import ControlCommand, ScenarioConfig
    from lightweight_sim.simulator.engine import SimulationEngine

    engine = SimulationEngine(ScenarioConfig())
    for _ in range(100):
        state = engine.step(
            ControlCommand(throttle=0.4, steer=0.0),
            dt=0.05,
        )
        if engine.is_done:
            break

可读取：

- engine.get_state()：当前 VehicleState；
- engine.get_error_state()：相对于参考线的误差；
- engine.collision_occurred：是否碰撞；
- engine.offroad_occurred：是否越界；
- engine.reached_destination：是否到达终点；
- engine.sim_time：仿真时间；
- engine.step_count：物理步数。

## 7. 运动学和动力学模型

默认配置使用运动学模型：

    config = ScenarioConfig(vehicle_model="kinematic")

需要验证横向侧滑时可切换动力学模型：

    config = ScenarioConfig(vehicle_model="dynamic")

动力学模型是简化自行车模型，适合接口和趋势验证，不应直接视为经过实车标定的高保真车辆模型。

## 8. 运行测试

轻量环境测试：

    python -m pytest

测试配置只收集 lightweight_sim/tests，避免在没有 CARLA Python API 的机器上误收集根目录 CARLA 测试。

也可以只运行核心测试文件：

    python -m pytest lightweight_sim/tests/test_core.py

`carla_legacy/test_code*.py` 属于 CARLA 测试，需要先安装并配置 CARLA 0.9.12 及其 Python API。

## 9. 运行日志和问题排查

日志文件位置：

    temp/logs/run_YYYYMMDD_HHMMSS_xxxxxx.log

日志包含以下关键节点：

- 引擎和规划器初始化、重置和退出；
- 控制量被限幅、物理步进异常；
- 规划请求、规划耗时、输出点数、障碍物位置、轨迹首/中/末点、超时和异常；
- 控制器实际参考点、局部横向/航向误差、全局参考误差和控制输出；
- 碰撞对象 ID/位置、越界距离/边界阈值和到达终点；
- 控制器切换和 GUI 生命周期。

排查时优先搜索 WARNING、ERROR、collision、offroad、planning failed 和 timed out。

## 10. 常见问题

### pygame 导入失败

确认依赖已经安装。如果使用 Python 3.14，优先安装 pygame-ce：

    python -m pip install pygame-ce

代码仍然使用 import pygame，pygame-ce 会提供兼容的模块名。

### 窗口启动后立即退出

检查是否从仓库根目录启动，并确认 pygame 能创建窗口。远程服务器或无桌面环境不能运行 GUI，但可以直接使用 SimulationEngine 做无窗口测试。

### 规划轨迹没有换道

当前 MotionPlanner 是车道级启发式规划器，要求道路有足够车道宽度和可行空间。两车道中间障碍物可能没有足够横向安全余量，此时规划器会返回最接近的可行基线，而不是保证强行换道。

### 想使用完整 MPC 或 DP+QP

当前默认链路优先保证实时性和稳定性。历史 DP/QP 文件仍保留，但完整 QP-MPC、纵向 ST 速度规划和 YAML 场景加载尚未纳入默认执行链，详见 lightweight_sim/DESIGN.md。

## 11. 建议的调试顺序

1. 运行 default 场景，确认道路、车辆和控制器正常；
2. 运行 obstacle 场景，观察规划结果和碰撞状态；
3. 运行 three_lane 场景，检查连续换道；
4. 运行 curve 场景，观察曲率和航向误差；
5. 最后切换 vehicle_model=dynamic，检查侧滑和横摆状态；
6. 调参时优先记录速度、ed、ephi、steer、throttle、brake 和碰撞标志。

设计细节、接口约定和后续路线见 lightweight_sim/DESIGN.md。
