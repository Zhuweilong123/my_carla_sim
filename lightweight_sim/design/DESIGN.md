# 轻量级仿真器设计总览

> 文档导航：算法执行链见 [ALGORITHMS.md](ALGORITHMS.md)，ROS 2 节点与接口见 [ROS2.md](ROS2.md)，拓扑地图见 [ROUTING.md](ROUTING.md)，参考线生成见 [REFERENCE_LINE.md](REFERENCE_LINE.md)。

## 范围与定位

`lightweight_sim/` 是独立于 CARLA 的 ROS 2 轻量级车辆运动仿真包，覆盖道路/车辆仿真、Routing、参考线生成、局部规划、控制、GUI 和测试。ROS 2 消息定义位于配套包 `lightweight_sim_msgs/`；独立泊车规划模块位于 `parking_module/`，通过消息接口与仿真集成，而非仿真器内核依赖。

仿真面向算法快速开发与回归验证，不是高保真轮胎/交通仿真，也不构成实车安全或部署证明。

## 当前系统数据流

```text
simulator_node
  ├─ vehicle/state + obstacles + sim/context
  └─ routing/request
          ↓
routing_node -> routing/route -> reference_line_node
                                    ↓ routing/reference_line
planner_node -> planned_path -> controller_node -> control_command/cruise ─┐
GUI/manual -------------------------------> control_command/manual          ├-> controller_manager
safe_stop_node --------------------------> safety/stop_request               ┘
                                                                                ↓
                                                                         simulator_node
```

Routing 提供拓扑级行驶路线；参考线节点把车道边序列转成平滑、带道路边界的连续参考线；局部规划器基于该线和障碍物输出短时域路径；控制器跟踪路径。GUI 是 ROS 2 客户端，不创建第二个仿真引擎。完整节点、topic、QoS 和服务合同见 [ROS2.md](ROS2.md)。

## 模块边界

| 模块 | 目录 | 职责 |
| --- | --- | --- |
| 仿真核心 | `engine/simulator/` | 场景、车辆/障碍物模型、道路、固定步长仿真与状态判定 |
| Routing | `engine/routing/` | JSON 地图模型、拓扑验证、A* 和 ROS 节点适配 |
| 参考线 | `engine/reference_line/` | RoutePlan 校验、几何拼接、圆角/采样、车道和可行驶边界 |
| 规划/控制 | `engine/algorithms/` | 局部路径、横向 LQR、纵向 PID、几何工具 |
| ROS 2 适配 | `engine/ros_nodes/` | 定时器、消息转换、运行标识、控制仲裁和安全停车监督 |
| 可视化 | `visualization/` | ROS 2 GUI、道路/路线/规划路径和 HUD 绘制 |
| 配置与地图 | `config/` | 节点默认参数、launch 文件及 JSON 地图 |
| 测试与脚本 | `tests/`, `scripts/` | 单元/集成测试、smoke test 和地图生成/实验工具 |

内核算法不直接依赖 GUI；GUI 仅通过 ROS 接口读取状态并调用仿真服务。Routing 与参考线几何核心可脱离 ROS 消息使用，节点负责 ROS 转换。

## 时序与配置

共享默认值定义在 `engine/runtime_config.py`，ROS 节点参数集中在 `config/default.yaml`。默认物理步长和规划/控制周期为 0.05 s（20 Hz）；GUI 渲染默认 60 FPS。规划输出短时域局部路径，控制器以更高频率跟踪。周期和超时属于仿真接口默认值，不代表实车调度承诺。

场景、车道和地图数据位于 `engine/simulator/` 与 `config/maps/`；车辆、限速类别、目标限速比例、横向加速度、Routing 代价、参考线曲率限制、局部规划走廊和控制器参数均通过配置/场景数据传递。具体参数以运行时配置文件及节点声明为准。

## 运行场景

内置巡航/道路场景包括直道、直道障碍物、三车道连续障碍物、90 度弯道、figure-eight 闭环和 demo_grid 路网；另有 reverse_parking 场景供外部泊车控制器集成测试。场景切换和地图编号由 GUI/仿真器提供。Routing 路线默认随场景请求生成；障碍物是否存在不决定是否启用 Routing。

巡航场景由 Routing、参考线、局部规划、控制器和安全监督组成。reverse_parking 使用独立的泊车任务流程；在没有与当前运行匹配的泊车计划时保持制动。泊车模块的接口说明见 [ROS2.md](ROS2.md) 中的集成章节。

## 安全行为与失败处理

`controller_manager` 是最终 `control_command` 的唯一发布者。模式切换、候选命令缺失或过期时会制动。`safe_stop_node` 监视当前 run 的 Routing 参考线及局部规划结果；未就绪、run 不匹配或数据过期时发出停车请求。Routing/参考线不可用不再以仿真器的基础道路路径作为静默回退。

上述措施用于仿真中的异常处理，不是功能安全实现。控制器/规划器计算时限、模型准确度及安全完整性仍需独立验证。

## 测试与 CI

`lightweight_sim/tests/` 包含纯 Python 核心单元测试，以及需要 ROS 2 的节点生命周期、消息和 launch 集成测试。默认 CI 工作流在每次 push/PR 运行不依赖 ROS 的 Python 测试；独立 ROS 集成工作流在相关 ROS、launch、接口或配置路径变更时触发，并支持手动运行。具体测试边界以 `.github/workflows/` 中的筛选列表为准。

## 设计约束与后续边界

- 地图是合成测试数据；真实道路应用需要地图质量和车道语义校验。
- Routing 搜索静态拓扑，不处理动态障碍物；局部规划能力不等于完整行为决策。
- 车辆模型为简化模型，执行器假设参数不是实车标定。
- 仿真成功或测试通过不能外推为实车可用性。
- 需要扩展模块时优先保持 Routing、参考线、局部规划、控制、仿真模型和 ROS 适配之间的单向接口，避免场景专属硬编码。
