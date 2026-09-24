# Routing 模块

Routing 将场景起终点转换为车道级拓扑路线。核心实现位于 `engine/routing/`，不依赖 ROS 消息；`routing_node` 负责 ROS 适配。障碍物避让由后续局部规划负责，不属于 Routing 的搜索状态。

## 地图模型

地图是 `config/maps/*.json` 下的 JSON 文件，包含 `map_id`、`lane_width`、`num_lanes`、节点和有向车道边。每条边用 `from_node`、`to_node`、中心线、车道索引、限速及 successor 列表定义拓扑；可提供左右边界和 maneuver 标签。路口连接器及车道变换也表示为普通边，搜索器按显式 successors 扩展。

默认地图目录包含 `straight_cruise`、`straight_obstacle`、`three_lane_double_obs`、`curve_90deg`、`figure_eight` 和 `demo_grid`。Routing 节点默认加载目录中的地图；可用 `map_dir` 覆盖目录，或用 `map_file` 只加载一个地图。

## 搜索与闭环

A* 在有向 lane-edge 图上搜索，支持按最快或最短策略计边代价，并可配置转弯、换道、掉头惩罚。RouteRequest 可指定 map、起终点 pose、起终车道、策略、是否允许掉头及 `closed_loop`。当 `closed_loop` 为 true 时，起点和终点必须解析到同一条 lane edge；专用搜索会寻找经过该边并沿 successor 有向闭合回到起始节点的最短环。闭环地图必须在 JSON successor 拓扑中显式连通；figure-eight 场景用此模式请求循环路线。

搜索输出有序 RouteSegment 列表、总长度、目标车道、路线标识及连续几何点。搜索失败时返回带原因的失败 RoutePlan，而不是虚构路线。

## ROS 接口

- topic `routing/request`：仿真场景适配器在初始化、reset 或场景切换时发布请求，是事件驱动输入，不是周期状态流。
- topic `routing/route`：发布计算结果，供 GUI 和参考线节点使用。
- service `routing/compute_route`：供外部节点按需同步请求路线。

请求和结果携带 request/run 关联信息。消费者只应接受与当前仿真 run 匹配的结果。标准 launch 默认启动 Routing。

## 独立启动和自定义地图

```bash
ros2 run lightweight_sim routing_node --ros-args -p map_dir:=/path/to/maps
ros2 run lightweight_sim routing_node --ros-args -p map_file:=/path/to/map.json
```

独立运行时仍需确保 RouteRequest 的 `map_id` 与加载地图一致。常用诊断：

```bash
ros2 topic info -v /routing/request
ros2 topic info -v /routing/route
ros2 service list | grep routing
ros2 topic echo /routing/route
```

## 与参考线和局部规划的边界

Routing 的结果是拓扑路线/有序车道边，不是最终控制器的局部轨迹。参考线节点对其验证并生成连续车道参考；局部规划器再结合障碍物产生短路径。分层细节见 [REFERENCE_LINE.md](REFERENCE_LINE.md) 和 [ALGORITHMS.md](ALGORITHMS.md)。
