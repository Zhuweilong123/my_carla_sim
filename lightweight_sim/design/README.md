# 轻量级仿真设计文档

本目录集中存放 lightweight_sim 的系统设计和接口说明。文档描述当前代码实现；参数值以 ../config/ 下的分层配置和 ../engine/runtime_config.py 为准，加载顺序见[配置说明](../config/README.md)。

| 文档 | 内容 |
| --- | --- |
| [DESIGN.md](DESIGN.md) | 系统定位、组件边界、数据流、时序、安全处理和验证范围 |
| [ALGORITHMS.md](ALGORITHMS.md) | Routing、参考线、局部规划、控制算法及当前限制 |
| [DP_QP.md](DP_QP.md) | DP 格点、五次多项式、凸走廊、QP 平滑及输出验证的完整流程 |
| [ROS2.md](ROS2.md) | ROS 2 节点、topics/services、QoS、launch 和集成 |
| [PARAMETER_AUDIT.md](PARAMETER_AUDIT.md) | 参数失配检查、物理参数来源、同步规则与验证 |
| [ROUTING.md](ROUTING.md) | 车道拓扑地图格式、A* 策略、闭环搜索和 ROS 接口 |
| [REFERENCE_LINE.md](REFERENCE_LINE.md) | 路线几何拼接、采样、边界和 planner 使用合同 |
| [UML 工程](uml/lightweight_sim_engine.umlproj) | 可视化模块设计工程文件 |
