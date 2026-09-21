# P1：统一评估与多圈验证

本目录与此前 `records/figure_eight_*.json/csv` 归档并存。旧数据未修改。
本轮采用 `route_projection_v2`：以实际车辆状态对实际参考折线投影，
使用线段切线计算横向、车身航向误差。控制器一周期预测误差单独记录，
不能与旧版全局离散点误差直接比较。`p1_baseline_30s` 是新口径的 30 秒基线。

## 结果

以下 RMS 使用 10 秒后的稳态窗口。所有六组均满足本轮的路线完整性、
完成圈数、无碰撞/越界、Riccati 收敛检查。

| 工况 | 圈数 | 横向 RMS m | 航向 RMS ° | 峰值速度 km/h |
| --- | ---: | ---: | ---: | ---: |
| 标准 50 km/h | 10 | 0.12950 | 1.99581 | 51.49954 |
| 初始横向 0.5 m、航向 5° | 3 | 0.12845 | 2.01030 | 51.45636 |
| 状态延迟 50 ms | 3 | 0.14286 | 3.88592 | 53.86403 |
| 位置噪声 σ=0.05 m | 3 | 0.11173 | 2.10158 | 51.55358 |
| 质量 +10%、前后轮侧偏刚度幅值 -10% | 3 | 0.08107 | 1.73628 | 50.84499 |
| 保留原折线顶点，细分至段长 ≤1 m | 3 | 0.12931 | 2.02534 | 51.46079 |

标准工况完成 7071 步 / 353.55 秒；独立解析 8 字相位检查确认 10 圈。
两处交叉口、接缝、起步和稳态均分别统计。扰动组只代表这些明确配置，
其中随机噪声只使用 seed=2026，不能据此声称覆盖所有扰动或获得统计显著改善。

## 重要发现

标准工况转向饱和比例约 68.56%，最大前轮转角变化率约 1145.92 °/s。
因此报告中的 `passed` 只表示上述路线/运行检查通过，不代表舒适性、
执行器可实现性或高车速控制品质达标。转向抖动、执行器约束、动力学离散化
与延迟鲁棒性应作为 P2 的优先项；P1 没有通过降低速度或修改控制律掩盖这些问题。

`lateral_accel_m_s2` 使用车体坐标横向加速度
`(vy_next-vy_previous)/dt + vx_next*r_next`，是控制周期尺度的近似诊断值。
控制耗时来自墙钟，在不同机器或后台负载下不能要求逐样本复现。

## 归档格式与复现

每组包含 JSON（完整配置、模型参数、控制参数、Git 版本、源码哈希、
参考线哈希、环境、分组统计和峰值时刻/区域）、CSV.gz（逐步明细）、
reference.csv（实际控制参考线）和 Markdown 报告。
运行时处于 P0 提交之后的 P1 工作区，因此 Git 字段指向 P0；
`source_sha256` 和 `source_tree_sha256` 标识实际参与运行的代码，
后续 P1 提交保存这些源文件。早期 30 秒基线在饱和比例字段加入前生成，
其源码哈希与完整套件不同；几何误差定义一致。

在 WSL 的 lightweight_sim 目录运行（每次使用新的 label，禁止覆盖现有 JSON）：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/figure_eight_benchmark.py \
  --suite --label p1_next --output-dir records/p1_next

# 单个工况；实际延迟 = delay-steps * dt
PYTHONDONTWRITEBYTECODE=1 python3 scripts/figure_eight_benchmark.py \
  --laps 10 --speed 50 --delay-steps 1 --seed 2026 \
  --label delay_next --output-dir records/p1_next

# 仍可运行历史口径，另起 label 保存
PYTHONDONTWRITEBYTECODE=1 python3 scripts/figure_eight_benchmark.py \
  --protocol legacy --duration 30 --label legacy_next --output-dir records/p1_next
```

GUI 优先消费 `tracking/metrics`，与离线使用同一个 TrackingMonitor；
控制器预测误差没有混入 GUI 的实际误差。无 ROS 诊断消息时，GUI 使用相同
几何定义独立计算。解析相位校验器不读取控制器索引，仅用于验证 8 字路线
的分支和圈数；它不是任意道路的通用真值定位器。

## 测试

验证结果：普通环境 23 项通过；加载 ROS 环境后共 25 项通过。
现有 WSL 工作区 `colcon build --packages-select lightweight_sim` 构建通过，
安装后的独立进程启动测试通过。归档源码哈希已与最终源文件逐一核对一致。

普通环境覆盖几何、控制器、跨圈/密度、状态保留、统一误差、独立分支检测、
峰值统计、固定随机种子复现和归档防覆盖。
ROS 环境另外覆盖真实 DDS 十圈/重置/场景切换，以及安装后的独立进程 launch、
`/clock`、50 km/h/三车道配置和诊断 topic。

```bash
source /opt/ros/lyrical/setup.bash
source ../install/setup.bash
ROS_DOMAIN_ID=87 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -s
```
