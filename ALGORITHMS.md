# CARLA 自动驾驶运动规划 — 算法详解

> **项目作者**: Weilong Zhu (朱伟龙)
> **时间**: 2021-2022
> **框架**: EM Planner 风格的运动规划与控制系统
> **仿真环境**: CARLA 0.9.12
> **文档生成**: Claude Code, 2026-05-31 (v2 增强版)

---

## 目录

1. [车辆动力学模型](#1-车辆动力学模型)
   - 1.1 [自行车模型假设](#11-自行车模型假设)
   - 1.2 [线性轮胎模型与侧偏刚度](#12-线性轮胎模型与侧偏刚度)
   - 1.3 [误差动力学状态方程推导](#13-误差动力学状态方程推导)
   - 1.4 [离散化方法：双线性变换](#14-离散化方法双线性变换tustin变换)
2. [横向控制算法](#2-横向控制算法)
   - 2.1 [LQR — 无限时域最优控制](#21-lqr--无限时域最优控制)
   - 2.2 [MPC — 有限时域约束最优控制](#22-mpc--有限时域约束最优控制)
   - 2.3 [MPC + 前馈控制](#23-mpc--前馈控制)
3. [纵向控制：PID 原理与积分分离](#3-纵向控制pid-原理与积分分离)
4. [横纵向联合控制器](#4-横纵向联合控制器)
5. [Frenet 坐标系与坐标变换](#5-frenet-坐标系与坐标变换)
6. [全局路径规划：A\* 搜索](#6-全局路径规划a-搜索)
7. [局部路径规划：S-L 图上的 DP + QP](#7-局部路径规划s-l-图上的-dp--qp)
   - 7.1 [动态规划 (DP) 在 S-L 图上的应用](#71-动态规划-dp-在-s-l-图上的应用)
   - 7.2 [二次规划 (QP) 精细化避障](#72-二次规划-qp-精细化避障)
   - 7.3 [SL → XY 坐标逆变换](#73-sl--xy-坐标逆变换)
8. [速度规划：S-T 图方法](#8-速度规划s-t-图方法)
9. [五次多项式：最小 Jerk 最优轨迹](#9-五次多项式最小-jerk-最优轨迹)
10. [参考线平滑：QP 二次规划方法](#10-参考线平滑qp-二次规划方法)
11. [航向角与曲率的数值计算](#11-航向角与曲率的数值计算)
12. [辅助算法](#12-辅助算法)
13. [算法参数汇总](#13-算法参数汇总)

---

## 1. 车辆动力学模型

### 1.1 自行车模型假设

自行车模型（Bicycle Model，也称 Single-Track Model）是车辆横向控制中最基本的动力学模型。它将车辆简化为一个二轮模型，通过以下假设实现降维：

**假设1：左右轮合并**。将车辆的左前轮和右前轮合并为一个"虚拟前轮"，左右后轮合并为一个"虚拟后轮"。这一假设在车辆不发生严重侧倾时是合理的。

**假设2：小角度假设**。前轮转角 $\delta$ 较小（通常 $|\delta| < 10°$），使得 $\sin\delta \approx \delta$, $\cos\delta \approx 1$。该假设在城市道路正常行驶条件下通常成立，但在急转弯场景下会引入误差。

**假设3：恒定纵向速度**。在横向控制的一个控制周期内，假设纵向速度 $V_x$ 恒定。这使得横向动力学与纵向动力学解耦，可以分别设计控制器。

**假设4：无滑移/小滑移**。假设轮胎工作在线性区域，侧向力与侧偏角成正比（见下节）。

> 📍 **代码位置**: `controller/Controller.py:16-63` (MPC 注释), `controller/Controller.py:340-372` (LQR 注释)

### 1.2 线性轮胎模型与侧偏刚度

轮胎的侧向力 $F_y$ 是侧偏角 $\alpha$ 的非线性函数。当 $\alpha$ 较小时（通常 $|\alpha| < 4°\sim5°$），二者近似为线性关系：

$$F_y = C \cdot \alpha$$

其中 $C$ 为**侧偏刚度**（Cornering Stiffness），为负值（因为侧向力方向与侧偏角方向相反）。

**前后轮侧偏角定义**：

$$\alpha_f = \delta - \frac{V_y + a \cdot r}{V_x}, \quad \alpha_r = -\frac{V_y - b \cdot r}{V_x}$$

其中 $r = \dot{\phi}$ 为横摆角速度，$\delta$ 为前轮转角，$a, b$ 分别为质心到前后轴的距离。

- $\alpha_f$：前轮速度方向与车轮指向之间的夹角。前轮速度由质心速度 $V_x, V_y$ 和横摆角速度 $r$ 合成。
- $\alpha_r$：后轮侧偏角，后轮不能转向，仅由质心速度和横摆角速度产生。

**线性轮胎力的局限**：当侧偏角超过 $5°\sim6°$ 时，轮胎进入非线性饱和区，线性模型精度急剧下降。因此 LQR/MPC 控制器需要通过约束 $|\delta| \leq 1$ 来限制前轮转角幅度，保证轮胎工作在线性区。

> 📍 **代码位置**: `controller/Controller.py:134-150` (矩阵填充中的 $C_f$, $C_r$)

### 1.3 误差动力学状态方程推导

#### 1.3.1 从车辆动力学到误差动力学

**步骤1：车辆横向动力学方程**

由牛顿-欧拉方程，车辆质心的横向加速度和绕Z轴的转动：

$$m(\dot{V}_y + V_x r) = F_{yf} + F_{yr}$$

$$I_z \dot{r} = a F_{yf} - b F_{yr}$$

其中 $V_x r$ 项是旋转参考系中的科里奥利加速度。

**步骤2：代入线性轮胎力**

$$F_{yf} = C_f \cdot \alpha_f = C_f \left( \delta - \frac{V_y + a r}{V_x} \right)$$

$$F_{yr} = C_r \cdot \alpha_r = C_r \left( - \frac{V_y - b r}{V_x} \right)$$

代入动力学方程，整理得：

$$\dot{V}_y = \frac{C_f + C_r}{m V_x} V_y + \left( \frac{a C_f - b C_r}{m V_x} - V_x \right) r - \frac{C_f}{m} \delta$$

$$\dot{r} = \frac{a C_f - b C_r}{I_z V_x} V_y + \frac{a^2 C_f + b^2 C_r}{I_z V_x} r - \frac{a C_f}{I_z} \delta$$

**步骤3：从车辆状态到误差状态**

引入 Frenet 坐标系下的横向误差 $e_d$ 和航向角误差 $e_\phi$：

$$\dot{e}_d = V_y \cos e_\phi + V_x \sin e_\phi \approx V_y + V_x e_\phi$$

$$\dot{e}_\phi = \dot{\phi} - \dot{\theta}_r = r - \kappa_r \dot{s} \approx r - \kappa_r V_x$$

其中 $\kappa_r$ 是参考线在投影点的曲率，$\dot{\theta}_r = \kappa_r \dot{s}$ 是参考线切向角的旋转速率。

对 $\dot{e}_d$ 和 $\dot{e}_\phi$ 求导，整理得标准状态空间形式：

$$\frac{d}{dt} \begin{bmatrix} e_d \\ \dot{e}_d \\ e_\phi \\ \dot{e}_\phi \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & \frac{C_f + C_r}{m V_x} & -\frac{C_f + C_r}{m} & \frac{a C_f - b C_r}{m V_x} \\ 0 & 0 & 0 & 1 \\ 0 & \frac{a C_f - b C_r}{I_z V_x} & -\frac{a C_f - b C_r}{I_z} & \frac{a^2 C_f + b^2 C_r}{I_z V_x} \end{bmatrix} \begin{bmatrix} e_d \\ \dot{e}_d \\ e_\phi \\ \dot{e}_\phi \end{bmatrix} + \begin{bmatrix} 0 \\ -\frac{C_f}{m} \\ 0 \\ -\frac{a C_f}{I_z} \end{bmatrix} \delta + \begin{bmatrix} 0 \\ \frac{a C_f + b C_r}{m V_x} - V_x \\ 0 \\ \frac{a^2 C_f + b^2 C_r}{I_z V_x} \end{bmatrix} \kappa_r V_x$$

即：

$$\dot{x} = A x + B u + C \cdot \kappa_r V_x$$

**关键理解**：
- **矩阵 A** 的第一行：$\dot{e}_d$ 本身是 $\dot{e}_d$（恒等式）
- **矩阵 A** 的第三行：$\dot{e}_\phi$ 本身是 $\dot{e}_\phi$（恒等式）
- **A[1][2] = -(Cf+Cr)/m**：这项将航向角误差 $e_\phi$ 映射为横向加速度的变化。直觉上，航向角偏离越大，车辆"侧滑"越严重。
- **矩阵 C**：代表道路曲率 $\kappa_r$ 对系统产生的持续扰动。即使 $\delta=0$，弯曲的道路也会产生非零的状态导数。

> 📍 **代码位置**:
> - MPC 矩阵计算: `controller/Controller.py:116-150`
> - LQR 矩阵计算: `controller/Controller.py:425-456`

#### 1.3.2 车辆参数

```python
vehicle_para = (a, b, m, Cf, Cr, Iz)
# Tesla Model 3 近似参数:
# a = 1.015 m     (质心到前轴)
# b = 1.895 m     (质心到后轴, 轴距2.910 - 1.015)
# m = 1412 kg     (整备质量 + 驾驶员)
# Cf = -148970 N/rad  (前轮侧偏刚度，负值表示力与侧偏角反向)
# Cr = -82204 N/rad   (后轮侧偏刚度)
# Iz = 1537 kg·m²    (绕Z轴转动惯量)
```

> 📍 **代码位置**: `test_code9.py:306`

### 1.4 离散化方法：双线性变换（Tustin变换）

**为什么需要离散化？** 计算机控制是离散时间的，需要将连续状态方程 $\dot{x} = Ax + Bu$ 转化为 $\Delta t = T_s = 0.1s$ 的离散时间形式。

**三种常见离散化方法**：

| 方法 | 公式 | 精度 | 稳定性 |
|------|------|------|--------|
| 前向欧拉 | $A_d = I + A T_s$ | $O(T_s)$ | 可能不稳定 |
| 后向欧拉 | $A_d = (I - A T_s)^{-1}$ | $O(T_s)$ | 稳定但阻尼过大 |
| **双线性变换** | $A_d = (I - \frac{AT_s}{2})^{-1}(I + \frac{AT_s}{2})$ | $O(T_s^2)$ | 稳定且保结构 |

**双线性变换（Tustin变换）的优势**：

1. **二阶精度**：比欧拉方法高一阶
2. **保持稳定性**：连续系统稳定 $\Rightarrow$ 离散系统稳定
3. **保结构**：连续系统的频率响应特征在离散域中较好保留

**数学推导**：

双线性变换基于梯形积分法则，对应 s 域到 z 域的映射：

$$s = \frac{2}{T_s} \cdot \frac{z-1}{z+1}$$

将 $\dot{x} \approx \frac{x_{k+1} - x_k}{T_s}$ 和 $x \approx \frac{x_{k+1} + x_k}{2}$ 代入连续方程：

$$\frac{x_{k+1} - x_k}{T_s} = A \cdot \frac{x_{k+1} + x_k}{2} + B u_k + C \cdot \kappa_r V_x$$

整理得：

$$\left(I - \frac{T_s}{2}A\right) x_{k+1} = \left(I + \frac{T_s}{2}A\right) x_k + B T_s u_k + C T_s \kappa_r V_x$$

$$x_{k+1} = \underbrace{\left(I - \frac{T_s}{2}A\right)^{-1} \left(I + \frac{T_s}{2}A\right)}_{A_d} x_k + \underbrace{\left(I - \frac{T_s}{2}A\right)^{-1} B T_s}_{B_d} u_k + \underbrace{\left(I - \frac{T_s}{2}A\right)^{-1} C T_s \kappa_r V_x}_{C_d}$$

**代码中的实现**：

```python
ts = 0.1  # 离散化时间间隔
temp = np.linalg.inv(np.eye(4) - (ts * self.A) / 2)
self.A_bar = temp @ (np.eye(4) + (ts * self.A) / 2)
self.B_bar = temp @ self.B * ts
self.C_bar = temp @ self.C * ts * self.k_r * self._vehicle_Vx
```

**关于 $C_d$ 项的说明**：代码将 $\dot{\theta}_r = \kappa_r \dot{s}$ 视为常数处理。在无侧滑假设下 $\dot{s} \approx V_x$，因此 $C_d$ 的形式如上所示。这是对非线性项的**线性化处理**——因为 $\kappa_r$ 随路径点变化，严格来说不应视为常数，但在一个控制周期内变化很小，近似合理。

> 📍 **代码位置**: `controller/Controller.py:152-169`

---

## 2. 横向控制算法

### 2.1 LQR — 无限时域最优控制

#### 2.1.1 LQR 问题的数学表述

LQR（Linear Quadratic Regulator）求解以下无限时域最优控制问题：

$$\min_{u_0, u_1, \dots} J = \sum_{k=0}^{\infty} \left( x_k^T Q x_k + u_k^T R u_k \right)$$

$$\text{s.t. } x_{k+1} = A_d x_k + B_d u_k$$

其中：
- $Q \succeq 0$（半正定）是状态误差权重矩阵
- $R \succ 0$（正定）是控制代价权重
- $x_k \in \mathbb{R}^4$ 是误差状态向量
- $u_k \in \mathbb{R}$ 是前轮转角

**为什么选择二次型代价？**
1. 数学上易处理：最优控制律是线性的，可通过代数方程求解
2. 物理意义明确：大的误差/控制量付出平方代价，天然惩罚极端值
3. 与二次规划兼容：后续 MPC 的 QP 子问题也是二次型

#### 2.1.2 从连续 LQR 到离散 LQR

连续时间 LQR 的代价函数为：

$$J = \int_0^\infty \left( x(t)^T Q x(t) + u(t)^T R u(t) \right) dt$$

最优控制律为线性状态反馈 $u(t) = -Kx(t)$，其中 $K = R^{-1}B^T P$，$P$ 是连续代数 Riccati 方程（CARE）的解：

$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

对于离散时间系统（已通过双线性变换得到 $A_d, B_d$），最优反馈增益通过**离散代数 Riccati 方程（DARE）**求解：

$$P = A_d^T P A_d - A_d^T P B_d (R + B_d^T P B_d)^{-1} B_d^T P A_d + Q$$

$$K = (R + B_d^T P B_d)^{-1} B_d^T P A_d$$

#### 2.1.3 Riccati 方程迭代求解

代码采用**迭代法**（值迭代）求解 DARE：

$$
P_{i+1} = A_d^T P_i A_d - A_d^T P_i B_d (R + B_d^T P_i B_d)^{-1} B_d^T P_i A_d + Q
$$

初始化 $P_0 = Q$，迭代直到 $\|P_{i+1} - P_i\|_{\max} < \varepsilon = 0.1$ 或达到最大迭代次数 5000。

**收敛性保证**：当 $(A_d, B_d)$ 能控且 $(A_d, Q^{1/2})$ 能观时，迭代收敛于唯一的正定解 $P^*$。对于自行车模型，这两个条件在 $V_x > 0$ 时通常成立。

> 📍 **代码位置**: `controller/Controller.py:458-487`, 方法 `LQR_fun()`

#### 2.1.4 前馈控制（Feedforward Control）

纯状态反馈 $u = -Kx$ 只能将状态驱动到原点（$e_d=0, e_\phi=0$），但存在道路曲率 $\kappa_r$ 这一**持续扰动**时，反馈控制会产生稳态误差。

**前馈控制的物理直觉**：在弯道上，即使车辆完美地处于车道中心（$e_d=0$），仍需要一个非零的转角来维持圆周运动。这个角度就是前馈量 $\delta_{ff}$。

前馈转角公式：

$$\delta_{ff} = \kappa_r \left[ L - b K_3 - \left( \frac{b}{C_f} + \frac{a K_3}{C_r} - \frac{a}{C_r} \right) \frac{m V_x^2}{L} \right]$$

其中 $L = a + b$ 为轴距，$K_3$ 是反馈增益矩阵 $K = [K_1, K_2, K_3, K_4]$ 中对应 $e_\phi$ 的元素。

**公式解析**：
- **$\kappa_r L$**：阿克曼转向几何的基准转角，车辆以曲率 $\kappa_r$ 沿圆弧行驶所需的基本转角
- **$-\kappa_r b K_3$**：LQR 反馈对稳态转角的修正
- **第三项**：速度平方项，反映了离心力随速度增大而增大的物理事实。$m V_x^2 / L$ 本质上是车辆以曲率 $\kappa_r$ 行驶时的离心力

**总控制量**：

$$u = -K \cdot e_{rr} + \delta_{ff}$$

**关于 $e_{rr}$ 的含义**：向量 $e_{rr} = [e_d, \dot{e}_d, e_\phi, \dot{e}_\phi]^T$ 是四维误差状态。其中：
- $-K_1 e_d$：位置误差的纠正力
- $-K_2 \dot{e}_d$：阻尼项，抑制横向摆动
- $-K_3 e_\phi$：航向误差的纠正力
- $-K_4 \dot{e}_\phi$：抑制横摆角速度的过度变化
- $+\delta_{ff}$：前馈项，消除弯道的稳态误差

> 📍 **代码位置**: `controller/Controller.py:570-584`, 方法 `forward_control_fun()`

#### 2.1.5 LQR 的稳定性分析

闭环系统 $x_{k+1} = (A_d - B_d K) x_k$ 的稳定性由矩阵 $A_{cl} = A_d - B_d K$ 的特征值决定。LQR 保证 $A_{cl}$ 的所有特征值都在单位圆内，且具有以下鲁棒性保证：

- **增益裕度**: $(1/2, \infty)$
- **相位裕度**: 至少 60°

这意味着 LQR 控制器对模型不确定性有较好的鲁棒性，这也是 LQR 在自动驾驶中被广泛使用的原因之一。

---

### 2.2 MPC — 有限时域约束最优控制

#### 2.2.1 为什么需要 MPC？

LQR 虽然鲁棒性好，但有**天然局限**：
1. **无约束处理**：LQR 无法显式处理控制量约束（$|\delta| \leq 1$）
2. **无限时域简化**：无限时域的单一反馈增益无法灵活适应不同场景

MPC 的核心思想是**滚动时域优化**（Receding Horizon Control）：
- 在每个控制周期，求解一个**有限时域** ($N$ 步) 的约束优化问题
- 只实施**第一步**优化得到的控制量
- 下一周期，时间窗口向前滑动，重新优化

#### 2.2.2 预测模型矩阵推导

**问题规模**：预测区间 $N=6$，控制区间 $P=2$，状态维度 $n=4$。

**符号约定**：
- $x_k \in \mathbb{R}^4$: 第 $k$ 步的预测状态 ($k = 0, 1, \dots, N$)
- $u_k \in \mathbb{R}$: 第 $k$ 步的控制量
- 控制序列 $U = [u_0, u_1]^T \in \mathbb{R}^2$（因为 P=2，仅优化前两步，后续控制量恒为零）

**预测状态序列**：

$$x_1 = A_d x_0 + B_d u_0 + C_d$$
$$x_2 = A_d x_1 + B_d u_1 + C_d = A_d^2 x_0 + A_d B_d u_0 + B_d u_1 + (A_d + I) C_d$$
$$x_3 = A_d x_2 + B_d u_2 + C_d = A_d^3 x_0 + A_d^2 B_d u_0 + A_d B_d u_1 + (A_d^2 + A_d + I) C_d$$

注意到 $u_2 = u_3 = \dots = u_{N-1} = 0$（控制区间外），所以：

$$x_k = A_d^k x_0 + \sum_{j=1}^{\min(k, P)} A_d^{k-j} B_d u_{j-1} + \sum_{j=1}^{k} A_d^{k-j} C_d$$

写为紧凑矩阵形式：

$$\underbrace{\begin{bmatrix} x_0 \\ x_1 \\ \vdots \\ x_N \end{bmatrix}}_{X \; ((N+1)n \times 1)} = \underbrace{\begin{bmatrix} I \\ A_d \\ A_d^2 \\ \vdots \\ A_d^N \end{bmatrix}}_{M \; ((N+1)n \times n)} x_0 + \underbrace{\begin{bmatrix} 0 & 0 \\ B_d & 0 \\ A_d B_d & B_d \\ \vdots & \vdots \\ A_d^{N-1}B_d & A_d^{N-2}B_d \end{bmatrix}}_{C \; ((N+1)n \times NP)} \underbrace{\begin{bmatrix} u_0 \\ u_1 \end{bmatrix}}_{U \; (NP \times 1)} + \underbrace{\begin{bmatrix} 0 \\ C_d \\ (A_d+I)C_d \\ \vdots \\ (\sum_{j=0}^{N-1}A_d^j)C_d \end{bmatrix}}_{C_c \; ((N+1)n \times 1)}$$

即：**$X = M x_0 + C U + C_c$**

#### 2.2.3 代价函数构造

$$J = \sum_{k=0}^{N-1} \left( x_k^T Q x_k + u_k^T R u_k \right) + x_N^T F x_N$$

将 $X = M x_0 + C U + C_c$ 代入：

$$\sum_{k=0}^N x_k^T Q x_k = X^T \bar{Q} X = (M x_0 + CU + C_c)^T \bar{Q} (M x_0 + C U + C_c)$$

其中 $\bar{Q} = \text{diag}(Q, Q, \dots, Q, F) \in \mathbb{R}^{(N+1)n \times (N+1)n}$ 是块对角矩阵，最后一块为终端权重 $F$。

展开并忽略与 $U$ 无关的常数项：

$$J = U^T (C^T \bar{Q} C + \bar{R}) U + 2 (C^T \bar{Q} M x_0 + C^T \bar{Q} C_c)^T U$$

**为什么需要终端代价 $F$？** 若不施加终端约束或终端代价，MPC 在有限时域内的最优解可能偏离无限时域最优解。终端代价的作用是**近似未来无限时域的成本**，使得有限时域优化接近全局最优。理论上，$F$ 应取为 DARE 的解 $P$，这样可以保证 MPC 的**递归可行性**和**闭环稳定性**。

#### 2.2.4 约束与求解

控制量约束：$-1 \leq u_k \leq 1, \forall k = 0, 1$

转化为线性不等式 $G U \leq h$：

$$G = \begin{bmatrix} I_{2} \\ -I_{2} \end{bmatrix}_{4 \times 2}, \quad h = \begin{bmatrix} 1 \\ 1 \\ 1 \\ 1 \end{bmatrix}_{4 \times 1}$$

转换为标准 QP 形式 $\min_U \frac{1}{2} U^T H U + f^T U$, s.t. $G U \leq h$：

$$H = 2(C^T \bar{Q} C + \bar{R}), \quad f = 2(C^T \bar{Q} M x_0 + C^T \bar{Q} C_c)$$

使用 **cvxopt** 求解器求解。cvxopt 内部实现的是**内点法**（Interior-Point Method），对于此类小规模 QP（4 个不等式约束）求解速度很快。

#### 2.2.5 MPC vs LQR 对比

| 维度 | LQR | MPC |
|------|-----|-----|
| 时域 | 无限时域 | 有限时域 ($N=6$) |
| 约束 | 无法显式处理 | 显式处理 $-1 \leq u \leq 1$ |
| 求解方式 | DARE 迭代（离线） | QP 求解（在线，每步） |
| 计算量 | 低（一次矩阵运算） | 中等（QP 优化） |
| $e_d$ 权重 | 200 | 250 (收敛更快) |
| 前馈 | 显式公式 | 未使用（MPC 自身处理扰动） |

#### 2.2.6 关于 $e_\phi$ 的 sin 近似

代码中 $e_\phi = \sin(\phi - \theta_r)$ 而非直接使用 $\phi - \theta_r$，原因有二：

1. **避免角度多值性**：$\phi$ 和 $\theta_r$ 都在 $[-\pi, \pi]$ 范围内，直接相减可能产生 $2\pi$ 的跳变，导致控制器输出突变
2. **小角度等价**：正常行驶中 $|\phi - \theta_r|$ 是小量，$\sin(e_\phi) \approx e_\phi$，近似误差可忽略

> 📍 **代码位置**: `controller/Controller.py:237-240`, `controller/Controller.py:556-557`

---

### 2.3 MPC + 前馈控制

`Lateral_MPC__with_feedforward_controller` 类结合了两种思路：

- **MPC 的约束处理能力**：显式约束 $-1 \leq u \leq 1$
- **前馈的稳态误差消除**：$\delta_{ff}$ 作为前馈量

与纯 MPC 的主要区别：
- 预测区间从 $N=6$ 缩短为 $N=4$
- $e_\phi$ 直接使用差值而非 $\sin$ 近似（因为前馈计算需要精确角度值）
- 代价函数矩阵构造方式略有不同

> 📍 **代码位置**: `controller/Controller.py:728-991`

---

## 3. 纵向控制：PID 原理与积分分离

### 3.1 PID 控制律

标准 PID 控制律：

$$u(t) = K_P \cdot e(t) + K_I \int_0^t e(\tau) d\tau + K_D \frac{de(t)}{dt}$$

其中 $e(t) = v_{target} - v_{current}$（km/h）。

**离散实现**（使用后向差分和矩形积分）：

$$u_k = K_P \cdot e_k + K_I \sum_{i=1}^k e_i \Delta t + K_D \frac{e_k - e_{k-1}}{\Delta t}$$

### 3.2 三个环节的物理作用

| 环节 | 作用 | 副作用 |
|------|------|--------|
| **P (比例)** | 快速响应误差，误差越大控制力越大 | 产生稳态误差（offset） |
| **I (积分)** | 消除稳态误差，积累历史偏差 | 引起超调、震荡、积分饱和 |
| **D (微分)** | 预判误差趋势，抑制超调 | 放大测量噪声 |

### 3.3 积分分离策略

**问题**：当误差很大时（如刚启动），积分项会快速累积，导致超调和长时间震荡。

**解决方案**：积分分离（Integral Clamping/分离）：

```python
if abs(error) > error_threshold:  # 1 km/h
    integral_error = 0.0           # 清零积分项
    error_buffer.clear()           # 清空历史误差缓冲区
```

**代码实现细节**：

```python
self.error_buffer = deque(maxlen=60)  # 滑动窗口存储最近60个误差值

# 积分项：矩形积分法
integral_error = sum(self.error_buffer) * self.dt

# 微分项：后向差分
differential_error = (self.error_buffer[-1] - self.error_buffer[-2]) / self.dt
```

`deque(maxlen=60)` 实现了一个**滑动窗口**，自动丢弃超过 60 步的旧数据。这相当于对积分项施加了一个时间窗口限制，防止过老的数据影响当前控制。

### 3.4 参数设计原理

$$K_P = 1.15, \quad K_I = 0, \quad K_D = 0$$

**为什么 $K_I = K_D = 0$？** 这是一个纯比例控制器。原因分析：
- 智能驾驶场景中目标速度变化频繁（而非恒定值），积分项会导致过度平滑和响应滞后
- CARLA 中的油门/刹车输入本身就是位置式（而非增量式），比例控制已能较好跟踪
- 如果需要改进，建议先引入小微分项抑制超调，再根据需要引入积分项

> 📍 **代码位置**: `controller/Controller.py:615-678`

---

## 4. 横纵向联合控制器

`Vehicle_control` 采用**门面模式**（Facade Pattern）统一调度：

```python
class Vehicle_control:
    Lat_control: Lateral_LQR_controller | Lateral_MPC_controller
    Lon_control: Longitudinal_PID_controller

    def run_step(target_speed):
        steering = Lat_control._control()    # 横向: LQR 或 MPC
        accel     = Lon_control.PID_control(target_speed)  # 纵向: PID
        return carla.VehicleControl(steer, throttle, brake)
```

**控制量整定逻辑**：

- 加速度 $\geq 0$: `throttle = min(1, accel)`, `brake = 0`
- 加速度 $< 0$: `throttle = 0`, `brake = min(1, |accel|)`

这一逻辑基于 CARLA 的控制接口：油门和刹车不能同时施加，且正值加速度对应油门，负值对应刹车。

> 📍 **代码位置**: `controller/Controller.py:681-725`

---

## 5. Frenet 坐标系与坐标变换

### 5.1 为什么需要 Frenet 坐标系？

在结构化道路（有明确车道线）上，用笛卡尔坐标 $(x, y)$ 描述车辆位置存在两个问题：
1. **道路是弯曲的**：很难用简单的 $(x, y)$ 表达"车辆位于车道中心偏左 0.5m"
2. **运动规划维度耦合**：弯道上，横向移动和纵向移动在 $(x, y)$ 下是耦合的

Frenet 坐标系将位置分解为：
- **$s$**：沿参考线（车道中心线）的弧长 → 纵向进度
- **$l$**：垂直于参考线的横向偏移 → 横向偏差

这使得路径规划问题转化为**在 S-L 平面上的曲线规划**问题——这正是 EM Planner 思想的核心。

### 5.2 匹配点查找

给定车辆位置 $(x, y)$，在参考线（离散点序列 $\{(x_i, y_i, \theta_i, \kappa_i)\}$）上找最近的匹配点：

$$i^* = \arg\min_i \left[ (x_i - x)^2 + (y_i - y)^2 \right]$$

**优化策略**：非首次运行时，根据上一匹配点和车辆运动方向确定搜索方向：
- 若 $(x,y)$ 在上一匹配点的切向正方向 $\Rightarrow$ 正向搜索（索引递增）
- 若在切向反方向 $\Rightarrow$ 反向搜索（索引递减）
- 搜索范围限制在 5 步内（`increase_count >= 5`），大幅减少计算量

这一优化利用了车辆连续运动的特性：相邻两个控制周期之间，车辆位置变化很小，匹配点不会"跳跃"太远。

> 📍 **代码位置**: `planner/planner_utiles.py:49-177`

### 5.3 Frenet-Serret 框架与投影点计算

参考线在每个点的局部几何由 Frenet-Serret 公式描述：

$$\frac{d\vec{r}}{ds} = \vec{t}, \quad \frac{d\vec{t}}{ds} = \kappa \vec{n}, \quad \frac{d\vec{n}}{ds} = -\kappa \vec{t}$$

其中 $\vec{t}$ 是单位切向量，$\vec{n}$ 是单位法向量，$\kappa$ 是曲率。

**投影点计算**（一阶近似）：

设匹配点 $M$ 的位置矢为 $\vec{r}_m$，切向量为 $\vec{t}_m$，曲率为 $\kappa_m$。$(x,y)$ 是实际位置：

1. 弧长偏移（$d\vec{v}$ 在切向的投影）：$ds = (\vec{r} - \vec{r}_m) \cdot \vec{t}_m$
2. 投影点位置：$\vec{r}_p \approx \vec{r}_m + ds \cdot \vec{t}_m$
3. 投影点航向：$\theta_p = \theta_m + \kappa_m \cdot ds$
4. 投影点曲率：$\kappa_p \approx \kappa_m$（一阶近似）

**关键假设**：$ds \ll 1/\kappa$，即在曲率半径远大于弧长偏移的情况下，投影点处曲率与匹配点处曲率近似相同。

> 📍 **代码位置**: `planner/planner_utiles.py:100-114`

### 5.4 完整坐标变换

**S (弧长) 的计算**：

以车辆当前位置的投影点为原点（$s=0$），参考线上其他点的 s 通过折线长度累加计算：

$$s_i = S_i - S_{origin}, \quad S_i = \sum_{j=1}^{i} \sqrt{(x_j - x_{j-1})^2 + (y_j - y_{j-1})^2}$$

> 📍 **代码位置**: `planner/planner_utiles.py:434-457`

**L (横向偏移) 的计算**：

$$l = (\vec{r}_{actual} - \vec{r}_{projection}) \cdot \vec{n}_p$$

**注意符号约定**：CARLA 使用 UE4 左手坐标系。对于单向车道：
- 车辆在参考线**左侧**（前进方向的左手边）→ $l < 0$
- 车辆在参考线**右侧**→ $l > 0$

代码中多处标注了 `***************************************` 注释，提醒法向量方向需要谨慎处理。

> 📍 **代码位置**: `planner/planner_utiles.py:460-492`

### 5.5 导数变换

Frenet 坐标下的一、二阶导数用于 DP 规划起点的边界条件：

| 变量 | 公式 | 物理含义 |
|------|------|----------|
| $l$ | $(\vec{r}_h - \vec{r}_r) \cdot \vec{n}_r$ | 横向偏移 |
| $\dot{l}$ | $\vec{V} \cdot \vec{n}_r$ | 横向速度（法向分量） |
| $\dot{s}$ | $\frac{\vec{V} \cdot \vec{t}_r}{1 - \kappa_r l}$ | 纵向速度沿参考线 |
| $\ddot{l}$ | $\vec{a} \cdot \vec{n}_r - \kappa_r(1 - \kappa_r l)\dot{s}^2$ | 横向加速度（含离心补偿） |
| $l' = dl/ds$ | $\dot{l} / \dot{s}$ | 横向偏移对弧长的变化率 |
| $\ddot{s}$ | $\frac{\vec{a} \cdot \vec{t}_r + 2\dot{s}^2 \kappa_r l'}{1 - \kappa_r l}$ | 纵向加速度沿参考线 |
| $l'' = d^2l/ds^2$ | $(\ddot{l} - l'\ddot{s}) / \dot{s}^2$ | 横向偏移对弧长的二阶导 |

> 📍 **代码位置**: `planner/planner_utiles.py:495-568`

---

## 6. 全局路径规划：A\* 搜索

### 6.1 问题建模

将 CARLA 地图的道路网络建模为**有向图** $G = (V, E)$：
- **节点 $V$**：每条路段的 entry 和 exit waypoint
- **边 $E$**：路段内部的 waypoint 序列
- **边权**：路段内 waypoint 的数量（近似路段长度）

### 6.2 A\* 算法原理

A\* 是 Dijkstra 的启发式改进，使用估计函数指导搜索方向：

$$f(n) = g(n) + h(n)$$

- $g(n)$：从起点到节点 $n$ 的实际代价（已知）
- $h(n)$：从节点 $n$ 到终点的启发式估计（预估）
- $f(n)$：通过节点 $n$ 的路径的总估计代价

**启发函数** — 三维欧氏距离：

$$h(n) = \sqrt{(x_n - x_{goal})^2 + (y_n - y_{goal})^2 + (z_n - z_{goal})^2}$$

**可采纳性（Admissibility）**：$h(n)$ 永远不会高估实际代价（两点之间的直线距离 ≤ 任何路径的实际长度），因此 A\* 保证找到最优路径。

**一致性（Consistency/Monotonicity）**：三角不等式 $h(n) \leq c(n, n') + h(n')$ 对于欧氏距离也成立，这意味着 A\* 对每个节点只需访问一次。

### 6.3 复杂度分析

- **时间复杂度**：$O(|E| \log |V|)$（使用优先队列/最小堆）
- **空间复杂度**：$O(|V|)$（存储 open_set 和 closed_set）

代码中使用字典（`dict`）而非优先队列实现 open_set，每次用 `min()` 遍历所有元素找最小值，导致实际复杂度为 $O(|V|^2)$。对于 CARLA Town05（通常 $< 500$ 个节点），性能可接受。如需优化，可改用 `heapq`。

> 📍 **代码位置**: `planner/global_path_plan.py:167-211`

---

## 7. 局部路径规划：S-L 图上的 DP + QP

这是 EM Planner 的**核心模块**，采用**粗搜索 + 精优化**的两阶段策略：
1. **DP（动态规划）**：在 S-L 图上快速生成粗路径，确定避障的大致形状
2. **QP（二次规划）**：在 DP 确定的可行域内求解精细平滑路径

### 7.1 动态规划 (DP) 在 S-L 图上的应用

#### 7.1.1 最优子结构与 Bellman 方程

动态规划的理论基础是 **Bellman 最优性原理**：最优策略的子策略也是最优的。

在 S-L 图上的具体体现：如果从起点 $s_0$ 到节点 $(s_j, l_j)$ 的最优路径经过前一列的节点 $(s_{j-1}, l_{k})$，那么从 $s_0$ 到 $(s_{j-1}, l_k)$ 的子路径也是最优的。

**递推公式**：

$$cost(i, j) = \min_{k \in \{0,\dots,row-1\}} \left[ cost(k, j-1) + cost_{neighbor}(k \to i) \right]$$

#### 7.1.2 采样网格设计

在 S-L 平面上建立 $12 \times 6$ 的采样网格：

```
s 方向: 0 → 15 → 30 → 45 → 60 → 75 → 90m  (间隔 15m, 6 列)
l 方向: +7.5m ~ -7.5m  (间隔 1.5m, 12 行, 覆盖 3~4 条车道宽度)
```

**设计考量**：
- S 方向覆盖约 90m（约 7.2s @ 45km/h），足以处理城市道路的避障需求
- L 方向覆盖 ±7.5m，对应约 4 条车道宽度，可处理换道避障
- 右行规则：行号为 0~5 对应 l < 0（左侧车道），6~11 对应 l > 0（右侧车道），在代价中给左侧车道加惩罚

#### 7.1.3 代价函数设计的理论依据

每条候选路径的代价由三部分组成：

$$J_{total} = J_{collision} + J_{smooth} + J_{ref}$$

**碰撞代价 $J_{collision}$**：

$$J_{collision} = \sum_{障碍物} \begin{cases}
10^{12} & d^2 \leq 16 \text{ (4m 以内, 硬约束)} \\
\frac{5000}{d^2} & 16 < d^2 < 36 \text{ (4-6m, 软惩罚)} \\
0 & d^2 \geq 36 \text{ (6m 以外, 安全)}
\end{cases}$$

- $10^{12}$ 的数值选择：足够大使得任何有碰撞风险的路径不被选择，但不过大导致数值问题
- $\frac{5000}{d^2}$ 的反比惩罚：距离越近代价增长越快，创建"安全裕度"

**平滑代价 $J_{smooth}$**：

$$J_{smooth} = w_{dl} \sum_{k=1}^{10} \left( \frac{dl}{ds} \right)^2 + w_{ddl} \sum_{k=1}^{10} \left( \frac{d^2l}{ds^2} \right)^2 + w_{dddl} \sum_{k=1}^{10} \left( \frac{d^3l}{ds^3} \right)^2$$

- 在两条五次多项式曲线上均匀采 10 个点
- 权重分配：$w_{dl}=300$, $w_{ddl}=1000$, $w_{dddl}=5000$ — 高阶导数权重更大，抑制急转弯

**参考线代价 $J_{ref}$**：

$$J_{ref} = w_{ref} \sum l^2, \quad w_{ref} = 20$$

驱使路径在不需避障时贴近参考线（车道中心）。

#### 7.1.4 路径增密

DP 采样间距为 15m，过于稀疏，无法直接用于控制。因此在相邻 DP 点之间用五次多项式以 1m 分辨率插值：

```python
for each adjacent pair (s_prev, l_prev) → (s_curr, l_curr):
    for s in arange(s_prev, s_curr, 1m):
        l = quintic(s)  # dl/ds = 0, d²l/ds² = 0 at both ends
```

增密后的路径点间距为 1m，总长约 90 个点，可直接用于二次规划或控制。

> 📍 **代码位置**: `planner/motion_plan_path_planning.py:215-363`

---

### 7.2 二次规划 (QP) 精细化避障

#### 7.2.1 为什么需要 QP？

DP 的局限：
1. **离散采样**导致路径不平滑（阶梯状）
2. **车辆形状**被简化为质点，忽略了车身的长宽约束

QP 在 DP 确定的**凸可行域**（convex feasible region）内求解连续优化问题，生成满足车辆形状约束的平滑路径。

#### 7.2.2 优化变量

对 DP 路径上的 $n$ 个点（降采样后），每个点维护 3 个变量：

$$\mathbf{x} = [l_0, l'_0, l''_0, l_1, l'_1, l''_1, \dots, l_{n-1}, l'_{n-1}, l''_{n-1}]^T \in \mathbb{R}^{3n}$$

其中 $l'_i = dl/ds|_i$, $l''_i = d^2l/ds^2|_i$。

#### 7.2.3 等式约束：三阶连续性

相邻两点之间用三次多项式连接，要求 $l, l', l''$ 连续：

由三次多项式的泰勒展开（以 $l'''$ 为自由参数）：

$$l_{i+1} = l_i + l'_i \Delta s + \frac{1}{2}l''_i \Delta s^2 + \frac{1}{6}l'''_i \Delta s^3$$

$$l'_{i+1} = l'_i + l''_i \Delta s + \frac{1}{2}l'''_i \Delta s^2$$

消去 $l'''_i$ 得到 $(2n-2)$ 个线性等式约束，对应矩阵 $A_{eq} \mathbf{x} = 0$：

$$A_{eq}^{(i)} = \begin{bmatrix} 1 & \Delta s & \frac{\Delta s^2}{2} & -1 & 0 & \frac{\Delta s^2}{6} \\ 0 & 1 & \frac{\Delta s}{2} & 0 & -1 & \frac{\Delta s}{2} \end{bmatrix}$$

#### 7.2.4 不等式约束：车辆矩形形状

关键洞察：车辆是矩形，不能只凭质心位置判断碰撞。QP 通过 8 个线性不等式约束保证车辆矩形的四个角都在可行域内：

$$\begin{bmatrix} 1 & d_1 & 0 \\ 1 & d_1 & 0 \\ 1 & -d_2 & 0 \\ 1 & -d_2 & 0 \\ -1 & -d_1 & 0 \\ -1 & -d_1 & 0 \\ -1 & d_2 & 0 \\ -1 & d_2 & 0 \end{bmatrix} \begin{bmatrix} l_i \\ l'_i \\ l''_i \end{bmatrix} \leq \begin{bmatrix} l_{max}[front] - w/2 \\ l_{max}[front] + w/2 \\ \vdots \\ -l_{min}[back] + w/2 \end{bmatrix}$$

其中 $d_1=3m$（车头到质心），$d_2=3m$（车尾到质心），$w=3m$（车宽）。

#### 7.2.5 代价函数矩阵分解

$$J = \underbrace{w_l \|L\|^2}_{\text{靠近参考线}} + \underbrace{w_{dl}\|L'\|^2 + w_{ddl}\|L''\|^2 + w_{dddl}\|L'''\|^2}_{\text{平滑性}} + \underbrace{w_{centre}\|L - L_{centre}\|^2}_{\text{凸空间中心}} + \underbrace{w_{end}\|L_{end}\|^2}_{\text{终端正则化}}$$

**各项权重的物理含义**：

| 代价项 | 权重 | 物理含义 |
|--------|------|----------|
| $l$ | 1000 | 靠近参考线（车道中心） |
| $dl/ds$ | 10000 | 抑制横向"漂移"（不希望路径太斜） |
| $d^2l/ds^2$ | 3000 | 抑制曲率过大（舒适性） |
| $d^3l/ds^3$ | 150 | 抑制曲率突变（更高阶舒适性） |
| centre | 250 | 向凸空间中心靠拢（安全性） |
| end_l, end_dl, end_ddl | 40 | 引导路径终点回到参考线 |

**二次规划化为标准形式**：

$$H = 2 \sum w_i H_i, \quad f = -2 w_{centre} \cdot L_{centre}$$

其中 $H_l = \text{diag}(1,0,0,1,0,0,\dots)$, $H_{dl} = \text{diag}(0,1,0,0,1,0,\dots)$, 依此类推。

$$H_{dddl} = \begin{bmatrix} 0 & 0 & -1 & 0 & 0 & 1 & \cdots \\ \vdots & & & \ddots & & & \end{bmatrix}$$

#### 7.2.6 L_min/L_max 边界计算

上下界根据 DP 路径和障碍物位置确定：

1. **默认边界**：$l_{min} = -6m$, $l_{max} = +6m$
2. **障碍物影响范围**：对每个障碍物，确定其在 S 方向的影响区间 $[s_{obs} - L/2, s_{obs} + L/2]$
3. **方向判断**：比较 DP 路径在障碍物质心处的 $l$ 值与障碍物的 $l$ 值
   - 障碍物在路径**右侧**：$l_{max}[j] = \min(l_{max}[j], l_{obs} - w_{obs}/2)$
   - 障碍物在路径**左侧**：$l_{min}[j] = \max(l_{min}[j], l_{obs} + w_{obs}/2)$
4. **偏移修正**：将车头和车尾的约束索引分别偏移 $+1$，适当扩展边界

#### 7.2.7 QP 的凸性保证

QP 问题为凸的充分条件：Hessian 矩阵 $H \succeq 0$（半正定）。本问题中：

$$H = 2 \left( w_l H_l + w_{dl} H_{dl} + w_{ddl} H_{ddl} + w_{dddl} H_{dddl}^T H_{dddl} + w_{centre} I \right)$$

所有 $H_i$ 均为对角矩阵（半正定），$H_{dddl}^T H_{dddl}$ 半正定（Gram 矩阵性质），且权重均为正。因此 $H \succ 0$（严格正定），QP 有唯一全局最优解。

> 📍 **代码位置**: `planner/motion_plan_path_planning.py:21-162` (QP), `planner/motion_plan_path_planning.py:165-212` (边界计算)

---

### 7.3 SL → XY 坐标逆变换

将 QP 优化后的 $(s, l)$ 序列转为笛卡尔坐标 $(x, y, \theta, \kappa)$：

1. 对每个 $s_i$，在 $s_{map}$ 中线性查找匹配点索引
2. 计算投影点：$\vec{r}_{proj} = \vec{r}_{match} + (s_i - s_{match}) \cdot \vec{t}_{match}$
3. 计算实际点：$\vec{r}_{actual} = \vec{r}_{proj} + l_i \cdot \vec{n}_{proj}$
4. 对离散 XY 点做平滑并计算 $(\theta, \kappa)$

> 📍 **代码位置**: `planner/motion_plan_path_planning.py:541-601`

---

## 8. 速度规划：S-T 图方法

> ⚠️ **状态**：核心框架已搭建，但完整实现未完成。

### 8.1 S-T 图的数学描述

S-T 图是一个二维空间：
- **横轴 (T)**：时间
- **纵轴 (S)**：沿路径的弧长

自车在 S-T 图上表示为一条从 $(0, s_0)$ 出发的单调递增曲线，斜率 = 速度。

障碍物在 S-T 图上表示为占据区域，形状由障碍物沿路径方向的长度和时间区间决定。

### 8.2 障碍物投影到 S-T 图

**匀速预测模型**（假设障碍物保持当前速度）：

$$s_{obs}(t) = s_{obs,0} + v_{obs} \cdot t$$

**自车**：

$$s_{ego}(t) = s_0 + v_{ego} \cdot t$$

### 8.3 非均匀采样设计

近距离需要更高的空间精度（为安全），远距离可用较粗精度（为效率）：

| S 范围 | 采样间隔 | 物理原因 |
|--------|---------|----------|
| 0–20m | 0.2m | 紧急制动区，需要精细控制 |
| 20–40m | 0.4m | 近距跟车区 |
| 40–60m | 0.8m | 中距决策区 |
| 60–80m | 1.5m | 远距预判区 |
| 80–100m | 2.5m | 长距规划区 |

这种设计体现了**精度与效率的权衡**——危险场景需要精细建模，安全场景可以粗化以降低计算量。

### 8.4 动态障碍物相遇分析 (test_code9.py)

**相对速度**：$\Delta v = v_{ego} - v_{obs}$

**相遇开始时间**（自车头与障碍物尾相遇）：

$$t_{meet} = \frac{dis - L_{ego}/2 - L_{obs}/2}{\Delta v}$$

**重叠持续时间**：

$$\Delta t = \frac{L_{ego} + L_{obs}}{\Delta v}$$

**相遇起始 S 位置**：

$$s_{meet} = s_0 + dis + v_{obs} \cdot t_{meet} - L_{obs}/2$$

将动态障碍物转化为在 $[s_{meet}, s_{meet} + L_{obs} \cdot \Delta t \cdot v_{obs}]$ 区间内的虚拟静态障碍物，送入路径规划 DP+QP。

> 📍 **代码位置**: `planner/motion_plan_speed_planning.py` | `test_code9.py:137-169`

---

## 9. 五次多项式：最小 Jerk 最优轨迹

### 9.1 为什么是五次多项式？

**最小 Jerk 最优控制问题**：在给定起点和终点状态的条件下，最小化加加速度（Jerk，即加速度的变化率）的积分：

$$\min_{l(s)} \int_{s_0}^{s_f} \left( \frac{d^3l}{ds^3} \right)^2 ds$$

$$\text{s.t. } l(s_0), l'(s_0), l''(s_0), l(s_f), l'(s_f), l''(s_f) \text{ 给定}$$

该问题的 Euler-Lagrange 方程为 $\frac{d^6 l}{ds^6} = 0$，其通解为**五次多项式**：

$$l(s) = a_0 + a_1 s + a_2 s^2 + a_3 s^3 + a_4 s^4 + a_5 s^5$$

**物理意义**：五次多项式使 jerk 的平方积分最小 → 乘员感受到的"顿挫感"最小 → 最舒适的轨迹。

**导数关系**：

$$l'(s) = a_1 + 2a_2 s + 3a_3 s^2 + 4a_4 s^3 + 5a_5 s^4$$

$$l''(s) = 2a_2 + 6a_3 s + 12a_4 s^2 + 20a_5 s^3$$

### 9.2 系数求解

6 个边界条件 → 6 个未知系数。构建线性系统：

$$A = \begin{bmatrix}
1 & s_0 & s_0^2 & s_0^3 & s_0^4 & s_0^5 \\
0 & 1 & 2s_0 & 3s_0^2 & 4s_0^3 & 5s_0^4 \\
0 & 0 & 2 & 6s_0 & 12s_0^2 & 20s_0^3 \\
1 & s_f & s_f^2 & s_f^3 & s_f^4 & s_f^5 \\
0 & 1 & 2s_f & 3s_f^2 & 4s_f^3 & 5s_f^4 \\
0 & 0 & 2 & 6s_f & 12s_f^2 & 20s_f^3
\end{bmatrix}$$

$$\mathbf{c} = A^{-1} \cdot [l_0, l'_0, l''_0, l_f, l'_f, l''_f]^T$$

**数值注意**：矩阵 A 在 $s_0, s_f$ 相差很大时可能接近奇异（病态）。代码中使用 `np.linalg.inv()` 直接求逆，对于通常的 $s_f - s_0 = 15m$ 的 DP 场景，条件数可接受。

### 9.3 计算结果示例

对于 DP 相邻点的连接（两端 $l'=0, l''=0$），五次多项式自动保证：
- 路径在两端是**二阶连续**的 $(C^2)$
- 路径在中间段是**平滑的**（无尖角）
- jerk 在整段上平方积分最小

> 📍 **代码位置**: `planner/planner_utiles.py:651-683`

---

## 10. 参考线平滑：QP 二次规划方法

### 10.1 问题背景

CARLA 提供的原始 waypoint 序列可能不够平滑，直接用作参考线会导致控制器产生不必要的转向动作。需要在保持几何相似性的前提下进行平滑。

### 10.2 优化建模

优化 $\mathbf{x} = [x_0, y_0, \dots, x_{n-1}, y_{n-1}]^T \in \mathbb{R}^{2n}$（约 60 个点）。

**三项代价**：

**1. 平滑代价** — 最小化二阶差分（曲率相关）：

$$J_{smooth} = \sum_{i=0}^{n-3} \left[ (x_i - 2x_{i+1} + x_{i+2})^2 + (y_i - 2y_{i+1} + y_{i+2})^2 \right] = \mathbf{x}^T A_1^T A_1 \mathbf{x}$$

其中 $A_1$ 的每两行编码一个二阶差分算子。

**2. 紧凑代价** — 最小化相邻点间距（避免点积聚）：

$$J_{length} = \sum_{i=0}^{n-2} \left[ (x_i - x_{i+1})^2 + (y_i - y_{i+1})^2 \right] = \mathbf{x}^T A_2^T A_2 \mathbf{x}$$

**3. 几何相似代价** — 保持与原始点的接近度：

$$J_{ref} = \sum_{i=0}^{n-1} \left[ (x_i - x_i^{ref})^2 + (y_i - y_i^{ref})^2 \right] = \|\mathbf{x} - \mathbf{x}_{ref}\|^2$$

展开得：$J_{ref} = \mathbf{x}^T I \mathbf{x} - 2 \mathbf{x}_{ref}^T \mathbf{x} + \text{const}$

**总 H 和 f**：

$$H = 2\left( w_{smooth} A_1^T A_1 + w_{length} A_2^T A_2 + w_{ref} I \right)$$

$$f = -2 w_{ref} \mathbf{x}_{ref}$$

### 10.3 约束与权重选取

约束：$|x_i - x_i^{ref}| \leq 0.2m$, $|y_i - y_i^{ref}| \leq 0.2m$

权重 $(0.4, 0.3, 0.3)$ 的选择体现了平滑性与保真性的平衡：
- 平滑性权重最高 (0.4)：首要目标是消除 zigzag
- 紧凑性和保真性各 0.3：次要目标
- 三者之和为 1.0，方便理解和调参

> 📍 **代码位置**: `planner/planner_utiles.py:247-347`

---

## 11. 航向角与曲率的数值计算

### 11.1 航向角：中点欧拉法

采用**中点欧拉法**而非简单前向/后向差分，以提高精度：

1. 计算相邻点差分：$\Delta x_i, \Delta y_i$（n-1 个值）
2. 边界延拓：$dx_i = (\Delta x_{i-1} + \Delta x_i) / 2$（首尾分别补 $\Delta x_0$ 和 $\Delta x_{n-2}$）
3. $\theta_i = \arctan2(dy_i, dx_i)$

中点欧拉法的**截断误差为 $O(h^2)$**，而简单前向差分为 $O(h)$。

### 11.2 曲率：sin 近似避免多值性

曲率 $\kappa = d\theta / ds$，其中 $d\theta$ 可能有 $2\pi$ 的多值性问题。代码用 $\sin(d\theta)$ 近似：

$$d\theta_i = \sin\left( \frac{\Delta\theta_{i-1} + \Delta\theta_i}{2} \right)$$

$$ds_i = \sqrt{dx_i^2 + dy_i^2}$$

$$\kappa_i = \frac{d\theta_i}{ds_i}$$

**$\sin(d\theta) \approx d\theta$ 的合理性**：对于平滑道路，相邻点的航向角变化 $|d\theta| \ll 1$ rad，$\sin(d\theta) = d\theta + O(d\theta^3)$，误差三阶小。

> 📍 **代码位置**: `planner/planner_utiles.py:180-214`

---

## 12. 辅助算法

### 12.1 位置预测

为补偿规划延迟（规划器计算需要时间，而车辆在持续运动），在规划起点处使用预测位置而非当前位置。

**笛卡尔预测**（控制器的误差计算中也使用）：

$$x_{pred} = x + V_x t_s \cos\phi - V_y t_s \sin\phi$$
$$y_{pred} = y + V_y t_s \cos\phi + V_x t_s \sin\phi$$
$$\phi_{pred} = \phi + \dot{\phi} \cdot t_s$$

其中 $t_s = 0.1s$（控制器）或 $0.2s$（规划器）。

**理论依据**：假设在短时间 $t_s$ 内，$V_x$, $V_y$, $\dot{\phi}$ 近似不变（零阶保持），则车辆在 Frenet 坐标系下做匀速圆周运动。

> 📍 **代码位置**: `planner/planner_utiles.py:571-594`

### 12.2 障碍物感知：向量点积筛选

传统传感器（如 CARLA 自带的 ObstacleDetector）只能检测直线方向的单个物体。代码实现了基于向量运算的扇区感知：

**核心判断**：障碍物在自车前方 $\iff$ 障碍物方向向量与自车速度向量的点积为正：

$$\vec{v}_1 \cdot \vec{V}_{ego} > 0$$

**横向距离判断**（是否在同一条路上）：

$$d_{lat} = \vec{v}_1 \cdot \begin{bmatrix} -\sin\theta \\ \cos\theta \\ 0 \end{bmatrix}$$

阈值 $-10m < d_{lat} < 12m$，覆盖约 3 条车道宽度。

> 📍 **代码位置**: `test_code9.py:49-90`

### 12.3 YOLOv3 目标检测

> ⚠️ **状态**: 已集成但注释掉（Python 推理太慢）

YOLOv3 的**多尺度检测**特性：
- 13×13 网格：检测大目标（整车）
- 26×26 网格：检测中目标
- 52×52 网格：检测小目标（远处车辆、行人）

**NMS**（IoU 阈值 0.3）用于去除重叠检测框。

> 📍 **代码位置**: `sensors/Sensors_camera_lib.py:147-221`

---

## 13. 算法参数汇总

### 13.1 控制参数

| 参数 | LQR | MPC | 说明 |
|------|-----|-----|------|
| $Q[0,0]$ ($e_d$) | 200 | 250 | 横向位置误差权重 |
| $Q[1,1]$ ($\dot{e}_d$) | 1 | 1 | 横向速度误差权重 |
| $Q[2,2]$ ($e_\phi$) | 50 | 50 | 航向角误差权重 |
| $Q[3,3]$ ($\dot{e}_\phi$) | 1 | 1 | 横摆角速度误差权重 |
| $R$ | 1 | 1 | 控制量代价 |
| $F$ | N/A | $I_4$ | MPC 终端代价 |
| $N$ | N/A | 6 | MPC 预测区间 |
| $P$ | N/A | 2 | MPC 控制区间 |
| $T_s$ | 0.1s | 0.1s | 离散化间隔 |
| Riccati $\varepsilon$ | 0.1 | N/A | 收敛阈值 |
| 最大迭代 | 5000 | N/A | 迭代次数上限 |

### 13.2 PID 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| $K_P$ | 1.15 | 纯比例控制 |
| 积分分离阈值 | 1 km/h | $|e| > 1$ 时清零积分 |
| 误差窗口 | 60 步 | deque(maxlen=60) |
| $dt$ | 0.01s | 采样时间 |

### 13.3 动态规划参数

| 参数 | 值 |
|------|-----|
| row × col | 12 × 6 |
| sample_s | 15m |
| sample_l | 1.5m |
| $w_{collision}$ | $10^{12}$ |
| $w_{dl}$ / $w_{ddl}$ / $w_{dddl}$ | 300 / 1000 / 5000 |
| $w_{ref}$ | 20 |
| $d_{danger}$ / $d_{safe}$ | 4m / 6m |
| 五次曲线上采样点 | 10 |
| 增密分辨率 | 1m |

### 13.4 二次规划参数 (路径)

| 参数 | 值 |
|------|-----|
| $w_l$ | 1000 |
| $w_{dl}$ (一阶) | 10000 |
| $w_{ddl}$ (二阶) | 3000 |
| $w_{dddl}$ (三阶) | 150 |
| $w_{centre}$ | 250 |
| $w_{end}$ | 40 |
| $d_1$ (车头) / $d_2$ (车尾) / $w$ (车宽) | 3m / 3m / 3m |

### 13.5 参考线平滑

| 参数 | 值 |
|------|-----|
| $w_{smooth}$ / $w_{length}$ / $w_{ref}$ | 0.4 / 0.3 / 0.3 |
| $x_{thre}$ / $y_{thre}$ | 0.2m |
| 局部参考线长度 | ~60 点 |

### 13.6 系统运行参数

| 参数 | 值 |
|------|-----|
| CARLA 同步频率 | 20 Hz ($\Delta t = 0.05s$) |
| 规划:控制频率比 | 1:50 ~ 1:100 |
| 规划位置预测 $t_s$ | 0.2s |
| 控制位置预测 $t_s$ | 0.1s |
| 感知范围 | 50m |
| 横向感知宽度 | ~22m (-10 ~ +12m) |

---

## 附录 A: 文件与算法对照表

| 文件 | 包含的算法 |
|------|-----------|
| `controller/Controller.py` | Bicycle Model, LQR, MPC, MPC+前馈, PID, 双线性变换, Riccati迭代, QP求解(cvxopt) |
| `planner/global_path_plan.py` | A\*搜索, 拓扑图构建, Graph构建(NetworkX) |
| `planner/motion_plan_path_planning.py` | DP(动态规划), QP(二次规划), SL↔XY变换, Bellman递推 |
| `planner/motion_plan_speed_planning.py` | S-T图构建, 速度DP(未完成) |
| `planner/planner_utiles.py` | Frenet坐标变换, 参考线QP平滑, 航向角/曲率数值计算, 五次多项式, 位置预测 |
| `sensors/Sensors_camera_lib.py` | YOLOv3多尺度检测, NMS |
| `test_code9.py` | 向量点积感知, 多进程规划-控制, 动态障碍物预测与相遇分析 |

## 附录 B: 关键参考文献

本项目的算法理论主要参考以下经典方法：

1. **EM Planner**: Fan H, et al. "Baidu Apollo EM Motion Planner" — S-L 图 + DP + QP 框架
2. **LQR/MPC**: Rajamani R. "Vehicle Dynamics and Control" (Chapter 4) — 自行车模型推导
3. **Frenet Frame**: Werling M, et al. "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenét Frame"
4. **A\***: Hart P E, Nilsson N J, Raphael B. "A Formal Basis for the Heuristic Determination of Minimum Cost Paths"
5. **QP Path Optimization**: Xu W, et al. "A Real-Time Motion Planner with Trajectory Optimization for Autonomous Vehicles"

---

> **文档版本**: v2 (增强版)
> **生成日期**: 2026-05-31
> **基于代码版本**: commit `de16b0a`
