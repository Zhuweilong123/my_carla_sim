# CARLA 自动驾驶运动规划 — 算法详解

> **项目作者**: Weilong Zhu (朱伟龙)  
> **时间**: 2021-2022  
> **框架**: EM Planner 风格的运动规划与控制系统  
> **仿真环境**: CARLA 0.9.12  
> **文档生成**: Claude Code, 2026-05-31

---

## 目录

1. [车辆动力学模型 (Bicycle Model)](#1-车辆动力学模型-bicycle-model)
2. [横向控制算法](#2-横向控制算法)
   - 2.1 [LQR 线性二次型调节器](#21-lqr-线性二次型调节器)
   - 2.2 [MPC 模型预测控制](#22-mpc-模型预测控制)
   - 2.3 [MPC + 前馈控制](#23-mpc--前馈控制)
3. [纵向控制算法 — PID](#3-纵向控制算法--pid)
4. [横纵向联合控制器](#4-横纵向联合控制器)
5. [Frenet 坐标系变换](#5-frenet-坐标系变换)
6. [全局路径规划 — A\* 算法](#6-全局路径规划--a-算法)
7. [局部路径规划 — S-L 图上的 DP + QP](#7-局部路径规划--sl-图上的-dp--qp)
   - 7.1 [动态规划 DP](#71-动态规划-dp)
   - 7.2 [二次规划 QP](#72-二次规划-qp)
   - 7.3 [SL → XY 坐标逆变换](#73-sl--xy-坐标逆变换)
8. [速度规划 — S-T 图](#8-速度规划--st-图)
9. [五次多项式插值](#9-五次多项式插值)
10. [参考线平滑 — QP 平滑](#10-参考线平滑--qp-平滑)
11. [航向角与曲率计算](#11-航向角与曲率计算)
12. [辅助算法](#12-辅助算法)
    - 12.1 [位置预测](#121-位置预测)
    - 12.2 [障碍物感知 (向量点积法)](#122-障碍物感知-向量点积法)
    - 12.3 [YOLOv3 目标检测](#123-yolov3-目标检测)
13. [算法参数汇总](#13-算法参数汇总)

---

## 1. 车辆动力学模型 (Bicycle Model)

### 1.1 状态变量

使用4维状态向量描述车辆横向运动：

$$x = \begin{bmatrix} e_d \\ \dot{e}_d \\ e_\phi \\ \dot{e}_\phi \end{bmatrix}$$

| 符号 | 含义 |
|------|------|
| $e_d$ | 横向误差 (车辆位置到参考线投影点的法向距离) |
| $\dot{e}_d$ | 横向误差变化率 |
| $e_\phi$ | 航向角误差 (车辆航向角与参考线切线方向之差) |
| $\dot{e}_\phi$ | 航向角误差变化率 |

### 1.2 车辆参数

代码中使用 Tesla Model 3 的近似参数 (`test_code9.py:306`)：

```
vehicle_para = (a, b, Cf, Cr, m, Iz)
            = (1.015, 1.895, 1412, -148970, -82204, 1537)
```

| 参数 | 值 | 含义 |
|------|-----|------|
| $a$ | 1.015 m | 质心到前轴距离 |
| $b$ | 1.895 m | 质心到后轴距离 ($2.910 - 1.015$) |
| $m$ | 1412 kg | 整车质量 |
| $C_f$ | -148970 N/rad | 前轮侧偏刚度 |
| $C_r$ | -82204 N/rad | 后轮侧偏刚度 |
| $I_z$ | 1537 kg·m² | 绕Z轴转动惯量 |

### 1.3 连续状态空间方程

控制量 $u = \delta$ (前轮转角)，系统矩阵由自行车模型推导：

$$\dot{x} = A x + B u + C \cdot k_r V_x$$

**矩阵 A (4×4):**

$$A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & \frac{C_f + C_r}{mV_x} & -\frac{C_f + C_r}{m} & \frac{a C_f - b C_r}{mV_x} \\
0 & 0 & 0 & 1 \\
0 & \frac{a C_f - b C_r}{I_z V_x} & -\frac{a C_f - b C_r}{I_z} & \frac{a^2 C_f + b^2 C_r}{I_z V_x}
\end{bmatrix}$$

**矩阵 B (4×1):**

$$B = \begin{bmatrix} 0 \\ -\frac{C_f}{m} \\ 0 \\ -\frac{a C_f}{I_z} \end{bmatrix}$$

**矩阵 C (4×1) — 曲率扰动项:**

$$C = \begin{bmatrix} 0 \\ \frac{a C_f + b C_r}{mV_x} - V_x \\ 0 \\ \frac{a^2 C_f + b^2 C_r}{I_z V_x} \end{bmatrix}$$

> 📍 **代码位置**: `controller/Controller.py:116-150` (MPC 版本), `controller/Controller.py:425-456` (LQR 版本)

### 1.4 速度分量计算

车辆质心侧偏角 $\beta$ (速度方向与车轴方向夹角):

$$\beta = \arctan 2(V_y^{world}, V_x^{world}) - \phi$$

车轴方向的纵向速度:

$$V_x = \|\vec{V}\| \cdot \cos\beta$$

垂直于车轴的横向速度:

$$V_y = \|\vec{V}\| \cdot \sin\beta$$

> 📍 **代码位置**: `controller/Controller.py:91-114`

### 1.5 离散化 — 双线性变换 (Tustin's Method)

将连续系统离散化，采样时间 $T_s = 0.1s$:

$$A_d = \left(I - \frac{T_s}{2}A\right)^{-1} \left(I + \frac{T_s}{2}A\right)$$

$$B_d = \left(I - \frac{T_s}{2}A\right)^{-1} B \cdot T_s$$

$$C_d = \left(I - \frac{T_s}{2}A\right)^{-1} C \cdot T_s \cdot k_r V_x$$

> 曲率相关项：$\dot{\theta}_r = \kappa_r \cdot \dot{s} \approx \kappa_r \cdot V_x$（无漂移假设下）

> 📍 **代码位置**: `controller/Controller.py:152-169`

---

## 2. 横向控制算法

### 2.1 LQR 线性二次型调节器

#### 2.1.1 原理

LQR 求解无限时域最优控制问题，代价函数:

$$J = \sum_{k=0}^{\infty} \left( x_k^T Q x_k + u_k^T R u_k \right)$$

最优控制律为状态反馈:

$$u_k = -K x_k$$

#### 2.1.2 黎卡提方程迭代求解

通过迭代求解离散代数黎卡提方程 (DARE):

$$P_{i+1} = A^T P_i A - A^T P_i B (R + B^T P_i B)^{-1} B^T P_i A + Q$$

收敛条件: $\max|P_{i+1} - P_i| < \varepsilon = 0.1$，最大迭代 5000 次。

反馈增益:

$$K = (R + B^T P B)^{-1} B^T P A$$

#### 2.1.3 前馈控制

为消除稳态误差，引入前馈转角:

$$\delta_{ff} = \kappa_r \left[ a + b - b K_3 - \left( \frac{b}{C_f} + \frac{a K_3}{C_r} - \frac{a}{C_r} \right) \frac{m V_x^2}{a + b} \right]$$

其中 $K_3$ 是反馈增益矩阵 K 的第3个元素（对应 $e_\phi$）。

#### 2.1.4 最终控制量

$$u = -K \cdot e_{rr} + \delta_{ff}$$

其中 $e_{rr} = [e_d, \dot{e}_d, e_\phi, \dot{e}_\phi]^T$

> 📍 **代码位置**: `controller/Controller.py:375-612`, 类 `Lateral_LQR_controller`

#### 2.1.5 Q 权重矩阵

```python
Q = diag([200, 1, 50, 1])  # e_d, ė_d, e_φ, ė_φ
R = 1                       # 控制量权重
```

---

### 2.2 MPC 模型预测控制

#### 2.2.1 原理

MPC 求解有限时域约束最优控制问题。设置:
- 预测区间 $N = 6$
- 控制区间 $P = 2$
- 状态维度 $n = 4$

#### 2.2.2 预测模型

将未来 N+1 个时刻的状态写成矩阵形式:

$$X = \begin{bmatrix} x_0 \\ x_1 \\ \vdots \\ x_N \end{bmatrix} = M x_0 + C U + C_c$$

其中:

- $M$ 维度 $((N+1)n \times n)$ — 初始状态到各时刻的自由响应
- $C$ 维度 $((N+1)n \times NP)$ — 控制序列到状态的映射
- $C_c$ 维度 $((N+1)n \times 1)$ — 曲率扰动累积项

**递推计算:**

$$M_0 = I_n, \quad M_k = A_d M_{k-1}$$

$$C[1] = B_d, \quad C[k, j] = A_d \cdot C[k-1, j]$$

$$C_{c,k} = A_d \cdot C_{c,k-1} + C_d$$

#### 2.2.3 代价函数

$$J = \sum_{k=0}^{N-1} \left( x_k^T Q x_k + u_k^T R u_k \right) + x_N^T F x_N$$

> $F$ 是终端代价权重，保证稳定性。

转化为标准二次型:

$$J = U^T H U + 2 E^T U$$

其中:

$$H = C^T \bar{Q} C + \bar{R}$$

$$E = C^T \bar{Q} C_c + C^T \bar{Q} M x_0$$

$\bar{Q}$ 是 $(N+1)n$ 维块对角矩阵，$\bar{R}$ 是 $NP$ 维块对角矩阵。

#### 2.2.4 约束与求解

控制量约束: $-1 \leq u_k \leq 1$

转化为标准形式 $G U \leq h$:

$$G = \begin{bmatrix} I_{NP} \\ -I_{NP} \end{bmatrix}, \quad h = \begin{bmatrix} \mathbf{1} \\ \mathbf{1} \end{bmatrix}$$

标准二次规划形式: $\min_U \frac{1}{2} U^T H U + f^T U$, s.t. $GU \leq h$

其中 $H \leftarrow 2H$, $f \leftarrow 2E$

使用 **cvxopt** 求解器求解。

> 📍 **代码位置**: `controller/Controller.py:66-337`, 类 `Lateral_MPC_controller`

#### 2.2.5 Q/R/F 权重矩阵

```python
Q = diag([250, 1, 50, 1])  # e_d 权重更高 (250 vs LQR的200)
R = 1
F = diag([1, 1, 1, 1])
```

---

### 2.3 MPC + 前馈控制

`Lateral_MPC__with_feedforward_controller` 类综合了 MPC 的预测优化能力和前馈控制的稳态误差消除能力：

- 预测区间 $N = 4$（比纯 MPC 的 6 短）
- 在代价函数中嵌入前馈信息

> 📍 **代码位置**: `controller/Controller.py:728-991`

---

## 3. 纵向控制算法 — PID

### 3.1 控制律

$$u_{lon} = K_P \cdot e(t) + K_I \int_0^t e(\tau) d\tau + K_D \frac{de(t)}{dt}$$

其中 $e(t) = v_{target} - v_{current}$  (单位: km/h)

### 3.2 积分分离

为防止积分饱和引起的超调和震荡，当 $|e| > 1$ km/h 时清零积分项和误差缓冲区:

```python
if abs(error) > error_threshold:  # error_threshold = 1 km/h
    integral_error = 0.0
    error_buffer.clear()  # 清空 deque(maxlen=60)
```

### 3.3 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| $K_P$ | 1.15 | 比例系数 |
| $K_I$ | 0 | 积分系数 (实际未使用) |
| $K_D$ | 0 | 微分系数 (实际未使用) |
| dt | 0.01 s | 控制周期 |
| 误差阈值 | 1 km/h | 积分分离阈值 |
| buffer | deque(60) | 误差缓冲区 |

> 📍 **代码位置**: `controller/Controller.py:615-678`, 类 `Longitudinal_PID_controller`

---

## 4. 横纵向联合控制器

`Vehicle_control` 类采用**门面模式**统一调用横向和纵向控制器:

```
Vehicle_control
  ├── Lat_control: Lateral_LQR_controller | Lateral_MPC_controller
  └── Lon_control: Longitudinal_PID_controller
```

控制输出整定:
- **转向**: clamp(`current_steering`, -1, 1)
- **油门/刹车**: 若加速度 > 0，设油门 = min(1, accel)，刹车 = 0；否则油门 = 0，刹车 = min(1, |accel|)

> 📍 **代码位置**: `controller/Controller.py:681-725`

---

## 5. Frenet 坐标系变换

### 5.1 核心概念

将笛卡尔坐标系下的位置 $(x, y)$ 投影到参考线 (由离散点序列构成) 上：

```
         n_r (法向量)
         ↑
         │  e_d (横向偏移)
    car  ●────→ 投影点
         │    ↗
         │  ds
         │↙
    ─────●──────────────────→ t_r (切向量)
    匹配点 (x_m, y_m, θ_m, κ_m)
```

### 5.2 匹配点查找

遍历参考线上所有点，找距离 $(x, y)$ 最近的点作为匹配点:

$$i^* = \arg\min_i \left[ (x_i - x)^2 + (y_i - y)^2 \right]$$

优化：非首次运行时，根据上一匹配点沿切向方向的正/反向遍历，缩短搜索范围。

> 📍 **代码位置**: `planner/planner_utiles.py:49-177`, 函数 `find_match_points()`

### 5.3 投影点计算

匹配点切向量: $\vec{t}_m = [\cos\theta_m, \sin\theta_m]^T$

匹配点法向量: $\vec{n}_m = [-\sin\theta_m, \cos\theta_m]^T$

偏移向量: $\vec{d} = [x - x_m, y - y_m]^T$

弧长偏移: $ds = \vec{d} \cdot \vec{t}_m$

投影点位置: $\vec{r}_p = \vec{r}_m + ds \cdot \vec{t}_m$

投影点航向: $\theta_p = \theta_m + \kappa_m \cdot ds$

投影点曲率: $\kappa_p \approx \kappa_m$

> 📍 **代码位置**: `planner/planner_utiles.py:100-114`

### 5.4 计算 S (弧长) 和 L (横向偏移)

**S 的计算** (折线近似):

$$S_i = \sum_{j=1}^{i} \sqrt{(x_j - x_{j-1})^2 + (y_j - y_{j-1})^2}$$

以车辆当前位置投影点为原点: $s_{map}[i] = S_i - S_{origin}$

**L 的计算**:

$$l = (\vec{r}_{实际} - \vec{r}_{投影}) \cdot \vec{n}_p$$

> 注：CARLA 使用 UE4 左手坐标系，车辆左侧为负 l 值。

> 📍 **代码位置**: `planner/planner_utiles.py:434-492`

### 5.5 L 对时间的导数 (用于动态规划)

| 变量 | 公式 | 含义 |
|------|------|------|
| $l$ | $(\vec{r}_h - \vec{r}_r) \cdot \vec{n}_r$ | 横向偏移 |
| $\dot{l}$ | $\vec{V}_h \cdot \vec{n}_r$ | l 对时间导数 |
| $\dot{s}$ | $\frac{\vec{V}_h \cdot \vec{t}_r}{1 - \kappa l}$ | s 对时间导数 |
| $\ddot{l}$ | $\vec{a}_h \cdot \vec{n}_r - \kappa(1 - \kappa l)\dot{s}^2$ | l 对时间二阶导数 |
| $l'$ | $\dot{l} / \dot{s}$ | l 对弧长导数 |
| $\ddot{s}$ | $\frac{\vec{a}_h \cdot \vec{t}_r + 2\dot{s}^2 \kappa l' + \dot{s}^2 \dot{\kappa} l}{1 - \kappa l}$ | s 对时间二阶导数 |
| $l''$ | $(\ddot{l} - l'\ddot{s}) / \dot{s}^2$ | l 对弧长二阶导数 |

> 📍 **代码位置**: `planner/planner_utiles.py:495-568`, 函数 `cal_s_l_deri_fun()`

---

## 6. 全局路径规划 — A\* 算法

### 6.1 拓扑图构建

从 CARLA 地图的 `get_topology()` 出发，重新构建有向图:

1. 提取每个路段的 entry/exit waypoint 作为节点
2. 按采样分辨率($\approx 2m$)在路段内部加密路点
3. 节点以 $(x, y, z)$ 坐标标识，建立 `id_map` 映射

边的属性:
- `entry_vector`: 入口切线方向单位向量
- `exit_vector`: 出口切线方向单位向量  
- `net_vector`: 入口到出口的弦方向
- `intersection`: 是否属于交叉路口
- `type`: LaneFollow / Left / Right 等

> 📍 **代码位置**: `planner/global_path_plan.py:43-103`

### 6.2 A\* 搜索

**启发函数** (三维欧几里得距离):

$$h(n) = \sqrt{(x_n - x_{goal})^2 + (y_n - y_{goal})^2 + (z_n - z_{goal})^2}$$

**代价函数**: $f(n) = g(n) + h(n)$，其中 $g(n)$ 是起点到节点 n 的累计路径长度（路段的路点数）。

**算法流程**:

```
1. 初始化 open_set = {start: (cost=0, parent=-1)}, closed_set = {}
2. 循环:
   a. 从 open_set 中选 f(n) 最小的节点 c_node
   b. 若 c_node == goal: 终止搜索
   c. 遍历 c_node 的所有后继 suc:
      - new_cost = g(c_node) + edge_length(c_node→suc)
      - 若 suc 在 closed_set: 跳过
      - 若 suc 在 open_set 且 new_cost < g(suc): 更新
      - 否则: 加入 open_set
   d. 将 c_node 从 open_set 移到 closed_set
3. 从 goal 回溯 parent 得到路径
4. 反转得到 start→goal 的有序路径
```

> 📍 **代码位置**: `planner/global_path_plan.py:167-211`, 方法 `_A_star()`

### 6.3 完整路径生成

A\* 返回节点 ID 序列 → 映射回 waypoint → 三段拼接:
1. **首段**: 起点投影到第一条边的 waypoint
2. **中间段**: 每条边的 path waypoints
3. **尾段**: 最后一条边投影到终点的 waypoint

> 📍 **代码位置**: `planner/global_path_plan.py:232-270`

---

## 7. 局部路径规划 — S-L 图上的 DP + QP

这是 EM Planner 风格的核心模块。

### 7.1 动态规划 DP

#### 7.1.1 采样

在 S-L 图上建立 $row \times col$ 采样网格:
- 默认: `row = 12, col = 6`
- S方向间隔: `sample_s = 15m`
- L方向间隔: `sample_l = 1.5m`
- L 范围: $-7.5m \sim +7.5m$（6+5条车道宽度）

```
      S=0  15   30   45   60   75   90
  L=+7.5  .  .  .  .  .  .  .
  L=+6.0  .  .  .  .  .  .  .
  ...     .  .  .  .  .  .  .
  L=0   b ------------------reference line
  ...     .  .  .  .  .  .  .
  L=-7.5  .  .  .  .  .  .  .
```

#### 7.1.2 五次多项式连接

相邻两列采样点之间用 **五次多项式** 连接，边界条件为两端 $dl/ds = 0$, $d^2l/ds^2 = 0$:

$$l(s) = a_0 + a_1 s + a_2 s^2 + a_3 s^3 + a_4 s^4 + a_5 s^5$$

> 详见 [第9节 五次多项式插值](#9-五次多项式插值)

#### 7.1.3 代价函数

每条候选路径的代价由三部分组成:

$$J_{total} = J_{collision} + J_{smooth} + J_{ref}$$

**碰撞代价** ($w = 10^{12}$):

$$J_{collision} = \begin{cases}
10^{12} & \text{if } d \leq d_{danger}^2 \\
\frac{5000}{d^2} & \text{if } d_{danger}^2 < d < d_{safe}^2 \\
0 & \text{otherwise}
\end{cases}$$

其中 $d$ 是五次曲线上的采样点到障碍物的欧氏距离平方，$d_{danger}=4m$, $d_{safe}=6m$。

**平滑代价** ($w_{dl}=300, w_{ddl}=1000, w_{dddl}=5000$):

$$J_{smooth} = w_{dl} \cdot \sum dl^2 + w_{ddl} \cdot \sum ddl^2 + w_{dddl} \cdot \sum dddl^2$$

在五次曲线上均匀采 10 个点计算。

**参考线代价** ($w = 20$):

$$J_{ref} = w_{ref} \cdot \sum l^2$$

促使路径靠近参考线（l=0）。

**额外规则**: 在 CARLA 坐标系中，左侧行（l < 0）车道速度更快，加 10000 代价使其倾向于右侧车道。

#### 7.1.4 DP 递推

$$cost[i][j] = \min_k \left( cost[k][j-1] + cost_{neighbor}(k \to i) \right)$$

`pre_node_index[i][j]` 记录最优前驱节点行号。

回溯得到最优路径序列。

#### 7.1.5 路径增密

DP 采样点间距较大，用五次多项式在相邻点间以 1m 分辨率插值:

```python
enriched_s = start_s + arange(0, end_s - start_s, resolution=1)
enriched_l = quintic(enriched_s)
```

> 📍 **代码位置**: `planner/motion_plan_path_planning.py:215-363`

---

### 7.2 二次规划 QP

QP 在 DP 结果的基础上进行更精细的避障路径优化。

#### 7.2.1 变量定义

优化变量 $\mathbf{x} = [l_0, l'_0, l''_0, l_1, l'_1, l''_1, \dots, l_n, l'_n, l''_n]^T$

每个 s 点有 3 个变量: $l$, $l'$, $l''$，共 $3n$ 个优化变量。

#### 7.2.2 等式约束 — 连续性

相邻两点之间的三阶连续性:

$$
\begin{cases}
l_{i+1} = l_i + l'_i \Delta s + \frac{1}{2} l''_i \Delta s^2 + \frac{1}{6} l'''_{i \to i+1} \Delta s^3 \\
l'_{i+1} = l'_i + l''_i \Delta s + \frac{1}{2} l'''_{i \to i+1} \Delta s^2
\end{cases}
$$

消去 $l'''$，得等式约束 $A_{eq} \mathbf{x} = 0$，其中 $A_{eq}$ 维度 $(2n-2) \times 3n$:

$$A_{eq}^{(i)} = \begin{bmatrix} 1 & \Delta s & \frac{\Delta s^2}{2} & -1 & 0 & \frac{\Delta s^2}{6} \\ 0 & 1 & \frac{\Delta s}{2} & 0 & -1 & \frac{\Delta s}{2} \end{bmatrix}$$

#### 7.2.3 不等式约束 — 车辆形状

考虑车辆矩形形状，而非将其视为质点:

前方点索引: $i_{front} = \min(i + \lceil \frac{d_1}{\Delta s} \rceil, n-1)$  
后方点索引: $i_{back} = \max(i - \lceil \frac{d_2}{\Delta s} \rceil, 0)$

$d_1 = 3m$ (质心到车头), $d_2 = 3m$ (质心到车尾), $w = 3m$ (车宽)

8 个约束不等式保证车辆四角不超出边界。

#### 7.2.4 代价函数

$$J = w_l \|L\|^2 + w_{dl} \|DL\|^2 + w_{ddl} \|DDL\|^2 + w_{dddl} \|DDDL\|^2 + w_{centre} \|L - L_{centre}\|^2 + w_{end} \|L_{end} - 0\|^2$$

| 权重 | 值 | 含义 |
|------|-----|------|
| $w_l$ | 1000 | 参考线代价 |
| $w_{dl}$ | 10000 | 一阶平滑代价 |
| $w_{ddl}$ | 3000 | 二阶平滑代价 |
| $w_{dddl}$ | 150 | 三阶平滑代价（jerk） |
| $w_{centre}$ | 250 | 凸空间中心代价 |
| $w_{end\_l}$ | 40 | 终点状态代价 $l \to 0$ |
| $w_{end\_dl}$ | 40 | 终点状态代价 $l' \to 0$ |
| $w_{end\_ddl}$ | 40 | 终点状态代价 $l'' \to 0$ |

矩阵化: $H = 2 \times$ (各项 H 矩阵加权求和), $f = -2 w_{centre} \cdot L_{centre}$

#### 7.2.5 边界约束 (l_min/l_max)

根据 DP 路径和障碍物位置确定可行域 $[l_{min}, l_{max}]$:

- 默认边界: $l_{min} = -6m$, $l_{max} = +6m$
- 障碍物在规划路径**右侧**: $l_{max}[j] = \min(l_{max}[j], l_{obs} - w_{obs}/2)$
- 障碍物在规划路径**左侧**: $l_{min}[j] = \max(l_{min}[j], l_{obs} + w_{obs}/2)$

> 📍 **代码位置**: 
> - 二次规划: `planner/motion_plan_path_planning.py:21-162`
> - 边界计算: `planner/motion_plan_path_planning.py:165-212`

---

### 7.3 SL → XY 坐标逆变换

将 QP 优化后的 $(s, l)$ 路径转回笛卡尔坐标:

1. 确定每个 $s$ 在参考线 $s_{map}$ 上的匹配点索引
2. 计算投影点: $\vec{r}_{proj} = \vec{r}_m + ds \cdot \vec{t}_m$
3. 计算实际点: $\vec{r}_{actual} = \vec{r}_{proj} + l \cdot \vec{n}_{proj}$
4. 对离散 XY 点平滑并计算 $(\theta, \kappa)$

> 📍 **代码位置**: `planner/motion_plan_path_planning.py:541-601`

---

## 8. 速度规划 — S-T 图

> ⚠️ **状态**: 框架已搭建，但核心逻辑**未完成**。

### 8.1 算法流程设计

1. 以路径规划输出的笛卡尔路径为参考线建立 Frenet 坐标系
2. 将动态障碍物投影到 S-T 图上
3. 速度决策 (DP 在 S-T 图上搜索)
4. 速度规划 (QP 平滑)

### 8.2 S-T 图构建

时间区间 $T = 8s$, 时隙 $\Delta t = 1s$

**非均匀采样**（提高近距离精度）:

| S 范围 | 采样间隔 |
|--------|---------|
| 0~20m | 0.2m |
| 20~40m | 0.4m |
| 40~60m | 0.8m |
| 60~80m | 1.5m |
| 80~100m | 2.5m |

总行数 `row = 100`，总列数 `col = T/Δt = 8`

### 8.3 障碍物预测

**自车**: $s_{ego}(t) = s_0 + v_{ego} \cdot t$ (匀速假设)

**障碍物**: $s_{obs}(t) = s_{obs,0} + v_{obs} \cdot t$

### 8.4 动态障碍物处理 (test_code9.py)

计算相遇时间和位置:

$$\Delta v = v_{ego} - v_{obs}$$

$$t_{meet} = \frac{dis - L_{ego}/2 - L_{obs}/2}{\Delta v}$$

$$\Delta t_{overlap} = \frac{L_{ego} + L_{obs}}{\Delta v}$$

$$s_{meet} = s_0 + dis + v_{obs} \cdot t_{meet} - L_{obs}/2$$

$$s_{leave} = s_{meet} + L_{obs} + \Delta t_{overlap} \cdot v_{obs}$$

在障碍物列表中添加虚拟静态障碍物（在相遇位置处）。

> 📍 **代码位置**: `planner/motion_plan_speed_planning.py` | `test_code9.py:137-169`

---

## 9. 五次多项式插值

### 9.1 问题描述

给定起点 $s_0$ 和终点 $s_f$，以及 6 个边界条件，求解五次多项式系数:

$$l(s) = a_0 + a_1 s + a_2 s^2 + a_3 s^3 + a_4 s^4 + a_5 s^5$$

$$l'(s) = a_1 + 2a_2 s + 3a_3 s^2 + 4a_4 s^3 + 5a_5 s^4$$

$$l''(s) = 2a_2 + 6a_3 s + 12a_4 s^2 + 20a_5 s^3$$

### 9.2 矩阵求解

构建线性方程组 $A \cdot \mathbf{c} = B$:

$$A = \begin{bmatrix}
1 & s_0 & s_0^2 & s_0^3 & s_0^4 & s_0^5 \\
0 & 1 & 2s_0 & 3s_0^2 & 4s_0^3 & 5s_0^4 \\
0 & 0 & 2 & 6s_0 & 12s_0^2 & 20s_0^3 \\
1 & s_f & s_f^2 & s_f^3 & s_f^4 & s_f^5 \\
0 & 1 & 2s_f & 3s_f^2 & 4s_f^3 & 5s_f^4 \\
0 & 0 & 2 & 6s_f & 12s_f^2 & 20s_f^3
\end{bmatrix}$$

$$B = [l_0, l'_0, l''_0, l_f, l'_f, l''_f]^T$$

$$\mathbf{c} = A^{-1} B = [a_0, a_1, a_2, a_3, a_4, a_5]^T$$

### 9.3 应用场景

| 场景 | 起点条件 | 终点条件 |
|------|---------|---------|
| DP 起点到第一列 | $(l_0, l'_0, l''_0)$ 已知 | $(l_f, 0, 0)$ |
| DP 相邻列连接 | $(l_{k-1}, 0, 0)$ | $(l_k, 0, 0)$ |
| DP 路径增密 | 同 DP 相邻列 | 同 DP 相邻列 |

> 📍 **代码位置**: `planner/planner_utiles.py:651-683`, 函数 `cal_quintic_coefficient()`

---

## 10. 参考线平滑 — QP 平滑

### 10.1 问题描述

对全局路径采样得到的局部参考线 (约 181 个点) 进行平滑，同时保持与原路径的相似性。

### 10.2 优化变量

$$\mathbf{x} = [x_0, y_0, x_1, y_1, \dots, x_n, y_n]^T \in \mathbb{R}^{2n}$$

### 10.3 代价函数

$$J = w_{smooth} J_{smooth} + w_{length} J_{length} + w_{ref} J_{ref}$$

**平滑代价** — 最小化二阶差分: $|x_i - 2x_{i+1} + x_{i+2}|^2$

$$A_1 = \begin{bmatrix}
1 & 0 & -2 & 0 & 1 & 0 & & \\
0 & 1 & 0 & -2 & 0 & 1 & & \\
& & & \ddots & & & &
\end{bmatrix}_{(2n-4) \times 2n}$$

$$J_{smooth} = \mathbf{x}^T A_1^T A_1 \mathbf{x}$$

**紧凑代价** — 最小化相邻点距离: $|x_i - x_{i+1}|^2$

$$A_2 = \begin{bmatrix}
1 & 0 & -1 & 0 & & \\
0 & 1 & 0 & -1 & & \\
& & & \ddots & &
\end{bmatrix}_{(2n-2) \times 2n}$$

$$J_{length} = \mathbf{x}^T A_2^T A_2 \mathbf{x}$$

**几何相似代价** — 保持接近原始点:

$$J_{ref} = \mathbf{x}^T I \mathbf{x} - 2 \mathbf{x}_{ref}^T I \mathbf{x}$$

### 10.4 约束

每个点的坐标偏移不超过阈值 $x_{thre} = y_{thre} = 0.2m$:

$$x_{ref} - 0.2 \leq x \leq x_{ref} + 0.2$$
$$y_{ref} - 0.2 \leq y \leq y_{ref} + 0.2$$

### 10.5 权重

| 权重 | 值 | 含义 |
|------|-----|------|
| $w_{smooth}$ | 0.4 | 平滑性权重 |
| $w_{length}$ | 0.3 | 紧凑性权重 |
| $w_{ref}$ | 0.3 | 几何相似性权重 |

### 10.6 最终 H 矩阵

$$H = 2 \left( w_{smooth} A_1^T A_1 + w_{length} A_2^T A_2 + w_{ref} I \right)$$

$$f = -2 w_{ref} \mathbf{x}_{ref}$$

> 📍 **代码位置**: `planner/planner_utiles.py:247-347`, 函数 `smooth_reference_line()`

---

## 11. 航向角与曲率计算

### 11.1 航向角 (中点欧拉法)

首先计算相邻点差分:

$$\Delta x_i = x_{i+1} - x_i, \quad \Delta y_i = y_{i+1} - y_i$$

对首尾做边界延拓，取前后差分的均值:

$$dx_i = \frac{\Delta x_{i-1} + \Delta x_i}{2}, \quad dy_i = \frac{\Delta y_{i-1} + \Delta y_i}{2}$$

航向角:

$$\theta_i = \arctan2(dy_i, dx_i)$$

### 11.2 曲率

曲率公式: $\kappa = \frac{d\theta}{ds}$

对 $\theta$ 差分: $\Delta\theta_i = \theta_{i+1} - \theta_i$

用 $\sin(\Delta\theta)$ 近似 $\Delta\theta$ (避免角度多值性):

$$d\theta_i = \sin\left(\frac{\Delta\theta_{i-1} + \Delta\theta_i}{2}\right)$$

$$ds_i = \sqrt{dx_i^2 + dy_i^2}$$

$$\kappa_i = \frac{d\theta_i}{ds_i}$$

> 📍 **代码位置**: `planner/planner_utiles.py:180-214`, 函数 `cal_heading_kappa()`

---

## 12. 辅助算法

### 12.1 位置预测

#### 笛卡尔坐标系预测

预测 $t_s$ 秒后车辆位置，用于补偿规划延迟:

$$x_{pred} = x + V_x t_s \cos\phi - V_y t_s \sin\phi$$
$$y_{pred} = y + V_y t_s \cos\phi + V_x t_s \sin\phi$$
$$\phi_{pred} = \phi + \dot{\phi} \cdot t_s$$

> 📍 **代码位置**: `planner/planner_utiles.py:571-594`, 函数 `predict_block()`

#### Frenet 坐标系预测

$$s_{pred} = s_0 + v \cdot t_s$$

在规划路径上找到对应 s 的 (x, y)。

> 📍 **代码位置**: `planner/planner_utiles.py:597-624`, 函数 `predict_block_based_on_frenet()`

---

### 12.2 障碍物感知 (向量点积法)

由于 CARLA 自带的 ObstacleDetector 只能检测直线方向单个物体，代码实现了基于**向量运算**的感知方法:

#### 步骤

1. **距离筛选**: 获取 100m 范围内所有车辆
2. **方向判断**: 计算障碍物方向向量 $\vec{v}_1$ 与自车速度 $\vec{V}_{ego}$ 的点积

   $$\vec{v}_1 \cdot \vec{V}_{ego} > 0 \Rightarrow \text{障碍物在前方}$$

3. **横向过滤**: 计算障碍物到自车航向的横向距离

   $$\vec{n}_r = [-\sin\theta_{ego}, \cos\theta_{ego}, 0]^T$$

   $$d_{lat} = \vec{v}_1 \cdot \vec{n}_r$$

   $$-10m < d_{lat} < 12m \Rightarrow \text{可能在同一条路上}$$

4. **动静态分类**:
   - 车速 > 1 m/s: 动态障碍物
   - 车速 ≤ 1 m/s: 静态障碍物

> 📍 **代码位置**: `test_code9.py:49-90`, 函数 `get_actor_from_world()`

---

### 12.3 YOLOv3 目标检测

> ⚠️ **状态**: 已集成但因 Python 推理太慢而**注释掉**。

#### 流程

1. 加载 YOLOv3 权重 (`yolov3.weights`) 和配置 (`yolov3.cfg`)
2. 将 RGB 图像转为 blob 输入网络
3. 三次下采样检测不同尺度物体 (13×13, 26×26, 52×52)
4. 非极大值抑制 (NMS, IoU 阈值 0.3)
5. 在图像上绘制检测框

#### 性能问题

- Python 下 YOLOv3-416 推理较慢
- 建议改用 C++ 实现或更轻量的检测模型

> 📍 **代码位置**: `sensors/Sensors_camera_lib.py:147-221`

---

## 13. 算法参数汇总

### 13.1 控制参数

| 参数 | LQR | MPC | 说明 |
|------|-----|-----|------|
| Q[0,0] ($e_d$) | 200 | 250 | 横向误差权重 |
| Q[1,1] ($\dot{e}_d$) | 1 | 1 | 横向速度误差权重 |
| Q[2,2] ($e_\phi$) | 50 | 50 | 航向角误差权重 |
| Q[3,3] ($\dot{e}_\phi$) | 1 | 1 | 航向角速度误差权重 |
| R (控制量) | 1 | 1 | 控制量权重 |
| F | N/A | I₄ | MPC 终端代价 |
| 预测区间 N | N/A | 6 | MPC 专用 |
| 控制区间 P | N/A | 2 | MPC 专用 |
| 离散化 $T_s$ | 0.1s | 0.1s | 控制周期 |

### 13.2 PID 参数

| 参数 | 值 |
|------|-----|
| $K_P$ | 1.15 |
| $K_I$ | 0 (未使用) |
| $K_D$ | 0 (未使用) |
| dt | 0.01s |
| 积分分离阈值 | 1 km/h |
| 误差缓冲区长度 | 60 |

### 13.3 动态规划参数

| 参数 | 值 |
|------|-----|
| row × col | 12 × 6 |
| sample_s | 15m |
| sample_l | 1.5m |
| 碰撞代价 $w_{collision}$ | $10^{12}$ |
| 平滑代价 $[w_{dl}, w_{ddl}, w_{dddl}]$ | [300, 1000, 5000] |
| 参考线代价 $w_{ref}$ | 20 |
| 危险距离 $d_{danger}$ | 4m |
| 安全距离 $d_{safe}$ | 6m |
| 五次曲线上采样点数 | 10 |
| 路径增密分辨率 | 1m |

### 13.4 二次规划参数 (路径)

| 参数 | 值 |
|------|-----|
| $w_l$ | 1000 |
| $w_{dl}$ | 10000 |
| $w_{ddl}$ | 3000 |
| $w_{dddl}$ | 150 |
| $w_{centre}$ | 250 |
| $w_{end\_l}$ | 40 |
| $w_{end\_dl}$ | 40 |
| $w_{end\_ddl}$ | 40 |
| $d_1$ (质心到车头) | 3m |
| $d_2$ (质心到车尾) | 3m |
| $w$ (车宽) | 3m |

### 13.5 参考线平滑参数

| 参数 | 值 |
|------|-----|
| $w_{smooth}$ | 0.4 |
| $w_{length}$ | 0.3 |
| $w_{ref}$ | 0.3 |
| $x_{thre}$ | 0.2m |
| $y_{thre}$ | 0.2m |
| 局部参考线长度 | ~60 点 |

### 13.6 系统配置

| 参数 | 值 |
|------|-----|
| CARLA 同步模式 fixed_delta_seconds | 0.05s (20Hz) |
| 规划/控制频率比 | 50:1 ~ 100:1 |
| 位置预测 $t_s$ | 0.2s |
| 感知范围 | 50m |
| 横向感知宽度 | ~22m (-10 ~ +12) |

---

## 附录: 文件与算法对照表

| 文件 | 包含的算法 |
|------|-----------|
| `controller/Controller.py` | LQR, MPC, MPC+前馈, PID, 双线性变换, 黎卡提方程, 二次规划(cvxopt) |
| `planner/global_path_plan.py` | A\* 搜索, 拓扑图构建 |
| `planner/motion_plan_path_planning.py` | DP, QP (cvxopt), SL↔XY变换, 五次多项式 |
| `planner/motion_plan_speed_planning.py` | S-T图 DP (未完成) |
| `planner/planner_utiles.py` | Frenet变换, 参考线QP平滑, 航向角/曲率计算, 五次多项式, 位置预测 |
| `sensors/Sensors_camera_lib.py` | YOLOv3目标检测 (已注释) |
| `test_code9.py` | 向量点积障碍物感知, 多进程规划-控制分离, 动态障碍物预测 |

---

> **文档生成日期**: 2026-05-31  
> **基于代码版本**: commit `de16b0a`
