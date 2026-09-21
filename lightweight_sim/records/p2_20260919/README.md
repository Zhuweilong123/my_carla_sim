# P2 第一轮：修复采样闭环不一致并抑制转向抖动

全部实验目标速度 50 km/h，动力学模型、原道路几何和纵向 PID 保持不变。
评估仍使用 P1 的 `route_projection_v2`，起步窗口固定为前 10 秒。
本轮是 P2 的一次优化，不表示执行器/轮胎约束和全部速度范围已验证完毕。

## 原因与修改

1. 旧反馈把预测位置/航向与当前 vy/r 混在同一个误差向量中。
   改成同时刻状态；旧的 0.05 秒预测仅保留作对照。
2. 原控制器使用双线性离散化，仿真器实际采用输入保持的 Euler 子步，且
   航向使用更新后的 r。现在先在 z=[y,vy,phi,r] 坐标构造真实采样转移，
   累积子步，再用 e=[y,vy+v*phi,phi,r] 变换到 LQR 误差坐标。
3. 控制参考航向在相邻点之间连续插值，避免直接跟随每段常值切线发生突变。
   独立评估继续使用原折线几何，未因控制参考平滑而更改误差口径。
4. 经过 R=1/4/10/25/100 对照后，选用 R=100；Q 保持 diag(200,1,50,1)。

对实际仿真状态转移做中央差分线性化，旧版在 50 km/h 的局部闭环谱半径
约 1.98；最终配置约 0.716。该检查是直线、零误差附近的小扰动分析，
不是对大扰动/轮胎饱和/任意速度的全局稳定性证明。

## 所有实验

- `ablation_01_*`：4 组 30 秒对照，分别分离反馈时刻和离散模型的影响。
- `weights_01_*`：4 组 30 秒权重实验，R=4/10/25/100，仍使用分段切线。
- `smooth_01_*`：3 组 30 秒实验，连续参考航向配合 R=1/25/100。
- `p2_candidate_*`：最终配置的 6 组复测，标准工况 10 圈，其余各 3 圈。
- 三个 `*_linearization.json`：共 55 个速度/配置组合的矩阵、特征值、
  谱半径和 K；速度覆盖 21.6/30/40/50/60 km/h。

共 17 次闭环实验均保存 CSV.gz 原始明细、JSON 参数/统计、reference.csv
实际参考线和 Markdown 报告。未采用的配置同样保留。
CSV 新增 vx、vy、r、前馈、反馈和限幅前转角，便于复查抖动来源。

`sources/*.zip` 保存每轮实际源代码快照，JSON 中的 `source_bundle` 指向
对应快照；每份文件有 SHA-256。不能只按 JSON 的 Git HEAD 重跑，因为
实验是在 P1 提交之后的工作区进行，部分中间配置尚未提交。

## 与 P1 同口径的十圈比较

| 指标 | P1 标准 10 圈 | P2 标准 10 圈 |
| --- | ---: | ---: |
| 稳态横向 RMS | 0.12950 m | 0.01691 m |
| 稳态横向最大绝对误差 | 0.20826 m | 0.07134 m |
| 稳态航向 RMS | 1.99581° | 1.61119° |
| 稳态速度误差 RMS | 0.76361 km/h | 0.22408 km/h |
| 峰值速度 | 51.49954 km/h | 50.52136 km/h |
| 转向饱和比例 | 68.562% | 0% |
| 稳态转角变化率 RMS | 1017.53 °/s | 19.56 °/s |
| 全程最大转角变化率 | 1145.92 °/s | 135.16 °/s |
| 全程速度误差 RMS（含起步） | 1.15891 km/h | 1.46606 km/h |

横向 RMS 下降约 86.94%。全程速度误差 RMS 没有改善；起步段速度变化
也保留在原始记录和分组报告中，不能把稳态改善理解为每项指标都改善。
标准工况完成 7101 步 / 355.05 秒；无错支路、异常进度跳变、碰撞或越界。

## 扰动复测（稳态窗口）

| 工况 | 圈数 | 横向 RMS m | 航向 RMS ° | 峰值速度 km/h |
| --- | ---: | ---: | ---: | ---: |
| 初始 0.5 m / 5° 偏差 | 3 | 0.01665 | 1.62916 | 50.51580 |
| 状态延迟 50 ms | 3 | 0.01647 | 1.53348 | 50.54464 |
| 位置噪声 σ=0.05 m，seed=2026 | 3 | 0.02561 | 1.72605 | 50.62966 |
| 质量 +10%、侧偏刚度幅值 -10% | 3 | 0.01239 | 1.49011 | 50.34041 |
| 原折线细分至段长 ≤1 m | 3 | 0.01678 | 1.60071 | 50.51767 |

五组均完成，无错支路/碰撞/越界。定位噪声下最大转角变化率仍约 326.32 °/s，
标准工况横向加速度诊断峰值约 20.81 m/s²。因此本轮未宣称执行器可实现性
或轮胎附着限制达标；后续应验证执行器动态、转角速率约束和模型适用边界。
随机扰动只有一个固定种子，不能据此声称统计意义上的全面鲁棒性。

## 复现（WSL，在 lightweight_sim 目录）

每次指定新的输出 label，不覆盖现有数据：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/p2_controller_experiments.py \
  --output-dir records/p2_repeat --label ablation --mode ablation
PYTHONDONTWRITEBYTECODE=1 python3 scripts/p2_controller_experiments.py \
  --output-dir records/p2_repeat --label weights --mode weights
PYTHONDONTWRITEBYTECODE=1 python3 scripts/p2_controller_experiments.py \
  --output-dir records/p2_repeat --label smooth --mode smooth
PYTHONDONTWRITEBYTECODE=1 python3 scripts/figure_eight_benchmark.py \
  --suite --label final --output-dir records/p2_repeat \
  --feedback-horizon-s 0 --lqr-discretization plant --lqr-r 100 --smooth-reference-heading
```

数值回归测试将 A/B 与实际车辆积分的中央差分结果比较，并验证小扰动
衰减和参考航向连续性；同时保留原有 Riccati 残差、核心仿真和路线测试。

## ROS 验证与原始记录

WSL 构建通过，完整测试 39 项通过（普通环境 37 项，ROS 验收 2 项）。
真实 DDS 十圈验证完成 7103 步 / 355.15 秒，累计约 4936.775 米；
复位、场景切换和独立进程启动均通过。它与离线评估是不同运行，数据分开保存。

- `ros_acceptance/dds_ten_laps.json.gz`：每个物理步的车辆状态、实际施加的
  命令、收到的下一条命令、控制误差和进度，以及收到的跟踪指标、切换事件。
- `ros_acceptance/installed_launch.json.gz`：独立启动时收到的状态、控制命令、
  场景上下文、跟踪指标、状态消息，以及完整 launch 日志文本。
- 相邻 JSON 保存通过状态、各类样本数和源码快照；`tests.xml` 保存全部测试结果。

ROS topic 记录是测试订阅端实际收到的样本，不是无损 rosbag；DDS 十圈的
每个物理步另从仿真器直接读取，保证该部分逐步状态完整。
归档完整性复核通过：17 次离线实验及 2 组 ROS 记录的样本数与元数据一致，
源码 ZIP 内文件的 SHA-256 与记录一致；JUnit 报告为 39 项测试、0 失败、0 错误。
要在后续验证中继续保存数据，指定新的目录：

```bash
source /opt/ros/lyrical/setup.bash
source ../install/setup.bash
ROS_DOMAIN_ID=87 LIGHTWEIGHT_SIM_ROS_ARCHIVE=records/p2_repeat/ros_acceptance \
  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -s \
  --junitxml=records/p2_repeat/tests.xml
```
