# 轻量级仿真设计文档

本目录集中存放 lightweight_sim 的系统设计和接口说明。文档描述当前代码实现；参数默认值以 ../config/default.yaml 和 ../engine/runtime_config.py 为准。

| 文档 | 内容 |
| --- | --- |
| [DESIGN.md](DESIGN.md) | 系统定位、组件边界、数据流、时序、安全处理和验证范围 |
| [ALGORITHMS.md](ALGORITHMS.md) | Routing、参考线、局部规划、控制算法及当前限制 |
| [ROS2.md](ROS2.md) | ROS 2 节点、topics/services、QoS、launch 和集成 |
| [ROUTING.md](ROUTING.md) | 车道拓扑地图格式、A* 策略、闭环搜索和 ROS 接口 |
| [REFERENCE_LINE.md](REFERENCE_LINE.md) | 路线几何拼接、采样、边界和 planner 使用合同 |
| [UML 工程](uml/lightweight_sim_engine.umlproj) | 可视化模块设计工程文件 |
