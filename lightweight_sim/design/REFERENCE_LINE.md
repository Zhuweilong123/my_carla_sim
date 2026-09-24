# 参考线模块

`reference_line_node` 将 Routing 输出的车道拓扑序列转换为局部规划器和控制器可用的连续参考线。它独立于 A* 搜索和障碍物避让。核心逻辑位于 `engine/reference_line/`。

## 数据流与有效性

```text
routing/route (RoutePlan)
        -> reference_line_node
        -> routing/reference_line (ReferenceLine)
        -> planner_node -> planned_path -> controller_node
```

节点按 map_id 回查 RoutePlan 中的有序 lane edges，校验边存在、车道元数据匹配以及相邻边 successor 连通。结果携带 request_id、route_id 和 map_id；下游还需与当前 `sim/context` 的 run/request 标识匹配。失败会发布失败状态，不应把其他 run 的旧参考线当作有效路线。

## 几何处理

实现会去除相邻段重复连接点，按配置间距重采样，对路口折线转角做有界圆角处理，并计算路径点的 `x/y/theta/kappa`。圆角曲率受 `max_reference_curvature_1pm` 及路口角阈值约束。生成的车道左右边界和道路可行驶外边界与参考线点对应；道路外边界供 planner 进行候选过滤/裁剪。

地图边可提供明确的中心线/左右边界。缺少边界时，地图加载器依据 lane width 派生。车道宽度、平行车道数及 lane index 决定所选车道与整条道路走廊的位置。

## 规划器使用

`planner_node` 只使用与活动 run 匹配的 Routing ReferenceLine。局部候选以所选车道参考为基准，可在障碍物附近临时避让，但必须留在配置的道路走廊内。没有匹配参考线或规划结果过期时，由 `safe_stop_node` 请求制动；当前系统不以仿真器独立基础参考路径作为回退。

GUI 将 Routing 拓扑结果、连续参考线和局部规划路径分别绘制，便于区分路线层级。绘制颜色属于可视化实现，不改变消息语义。

## 参数

ROS 参数在 `config/default.yaml` 的 `reference_line_node` 和 `planner_node` 命名空间配置，常用项包括：

| 参数 | 用途 |
| --- | --- |
| `sample_spacing_m` | 参考线采样间距 |
| `max_lateral_deviation_m` | 平滑允许的最大横向偏移 |
| `join_tolerance_m` | 相邻拓扑边允许的连接误差 |
| `boundary_margin_m` | 参考线几何处理时使用的边界余量 |
| `max_reference_curvature_1pm` | 圆角几何曲率上限 |
| `junction_angle_threshold_rad` | 识别需要处理的路口转角阈值 |
| `routing_corridor_margin_m` | planner 对道路外边界保留的安全余量 |

参数默认值以 `config/default.yaml` 为准。标准 launch 默认启动参考线节点；禁用 Routing 或参考线节点会使安全监督进入停车状态，而不会自动切换到另一条无关路线。
