# lightweight_sim

ROS 2 轻量级二维车辆运动仿真包，包含仿真引擎、车道级 Routing、连续参考线、局部路径规划、控制、安全监督及 GUI 客户端。

## 文档

架构、算法、节点接口、Routing 与参考线设计统一收录于 [design/](design/README.md)。常用地图在 config/maps/，ROS 节点默认参数在 config/default.yaml。

## 快速启动

在已加载 ROS 2 环境的仓库根目录构建：

```bash
colcon build --packages-up-to lightweight_sim
source install/setup.bash
ros2 launch lightweight_sim lightweight_sim.launch.py gui:=true
```

更多依赖、launch 参数和 GUI 说明请查看仓库根目录的 [README](../README.md)。
