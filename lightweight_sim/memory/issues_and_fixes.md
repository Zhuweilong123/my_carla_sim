# 遇到的问题及解决办法归档

> 项目: Lightweight Planning & Control Simulator
> 日期: 2026-06-06
> 版本: 含 Phase 1-3 所有问题

---

## 问题 1: Python 3.14 + pygame 安装失败

### 现象

```bash
pip install pygame
# ERROR: Failed to build 'pygame' when getting requirements to build wheel
# FileNotFoundError: [WinError 2] 系统找不到指定的文件 (pacman)
```

### 原因

Python 3.14.5 太新，pygame 官方未发布 cp314 预编译 wheel，pip 回退到源码编译但缺少 MSYS2 环境。

### 解决办法

**使用 `pygame-ce`（Community Edition）替代**：

```bash
pip install pygame-ce
```

- `pygame-ce` 有 cp314 预编译 wheel
- API 完全兼容原版 pygame，`import pygame` 不变

---

## 问题 2: pygame-ce SysFont 在 Python 3.14 崩溃

### 现象

```python
pygame.font.SysFont('consolas', 20)
# TypeError: expected str, bytes or os.PathLike object, not int
#  File "pygame/sysfont.py", line 80, in initsysfonts_win32
```

### 原因

Python 3.14 的 `os.path.splitext()` 不再接受非字符串参数。Windows 注册表字体枚举返回了部分 int 类型值。

### 解决办法

在 `simulator/app.py` 模块加载时 monkey-patch 两个函数：

1. `initsysfonts_win32` → 过滤非 str 字体名
2. `SysFont.__init__` → 捕获 TypeError 回退到 `pygame.font.Font(None, size)`

HUD/Renderer 字体创建添加 try/except 回退。

---

## 问题 3: CARLA API 强依赖

### 现象

原控制器依赖 `carla.Vehicle.get_location()` 等 CARLA API，无法独立运行。

### 解决办法

设计 `VehicleState` dataclass，实现 `get_error_state(ref_path)` 完全替代 CARLA API 调用。控制器新增 `control(x,y,phi,vx,vy,r,ref_path)` 接口。

---

## 问题 4: 按键完全不响应

### 现象（多次迭代）

**第1次**: Windows 上 `pygame.KEYDOWN` 事件不可靠，Q/R 键不响应。

**修复**: 改用 `pygame.key.get_pressed()` 轮询 + 去抖机制。

**第2次**: 修复后仍不灵敏，WASD 也无法操作。

**修复**: 重写主循环，`pygame.event.pump()` 后直接用 `get_pressed()` 处理所有按键。

**第3次（最终根因）**: 按 R/Z 后所有按键失效。

**根因**: `pygame.event.get()` 调用会**吞掉所有类型的事件**（包括 KEYDOWN），但代码只处理了 QUIT 和 ESC。其余 KEYDOWN 事件被丢弃，`get_pressed()` 也读不到（短暂按键已作为事件消费）。

**最终修复**: 改用类型过滤：
```python
pygame.event.get(pygame.QUIT)        # 只取QUIT
pygame.event.get(pygame.MOUSEWHEEL)  # 只取滚轮
keys = pygame.key.get_pressed()      # 所有按键统一从这读
```

---

## 问题 5: 道路绘制不对称

### 现象

车辆看起来不在道路中间，只有一侧实线和中间虚线。

### 原因

所有车道线 offset 都在参考线法向量的**同一侧**。参考线成了道路左边缘而非中心线。

### 解决办法

- 道路以参考线为中心对称绘制：offset 从 `-N*lane_width/2` 到 `+N*lane_width/2`
- 添加灰色路面多边形填充
- `world.get_lane_center()` 返回值对称分布

---

## 问题 6: cvxopt 大文件下载慢

### 现象

`pip install cvxopt` (13.8MB) 和 `pygame-ce` (10.6MB) 耗时超过5分钟。

### 原因

网络带宽有限（约40-50 KB/s）。

### 解决办法

分批安装，使用 `--default-timeout=300`。

---

## 问题 7: Frenet 法向量方向约定

### 现象

原 CARLA 代码多处标注 `*****` 提示法向量方向可能有问题。

### 原因

CARLA 使用 UE4 左手坐标系，法向量 `n=[-sinθ, cosθ]` 含义与标准右手系不同。

### 解决办法

统一使用 `n=[-sinθ, cosθ]`，在代码注释中标注约定。道路居中后参考线=车道分界线，l<0 为右侧车道。

---

## 问题 8: 规划器子进程在 Windows 上崩溃

### 现象

```python
Process(target=_planning_process, args=(conn,))
p.start()
p.is_alive()  # → False (立即死亡, exitcode=1)
```

### 原因

Windows multiprocessing 使用 **spawn** 模式（非 fork），子进程启动全新 Python 解释器。`sys.path` 不包含项目根目录，导致 `from lightweight_sim...` 导入失败。

### 解决办法

**放弃 multiprocessing，改用 threading**：

- 线程共享内存空间和 sys.path → 无导入问题
- cvxopt QP 求解时会释放 GIL → 并行性不受影响
- `queue.Queue` 替代 `Pipe` 进行线程间通信
- 非阻塞：`get_result()` 通过 `get_nowait()` 实现

---

## 问题 9: DP 规划页面卡死

### 现象

切换自动模式后，pygame 画面冻结，控制台持续输出 "Planning timed out"。

### 原因

初版用 `conn.recv()` **阻塞**等待子进程返回结果。QP 求解需要 0.1-0.3s，主循环被堵住。

### 解决办法

1. `poll_result()` 非阻塞检查队列
2. 主循环轮询：有数据→取，无数据→跳过
3. 超时保护：200步(~10s)未返回则放弃

---

## 问题 10: DP 代价函数 TypeError

### 现象

```python
TypeError: only 0-dimensional arrays can be converted to Python scalars
```

### 原因

`evaluate_quintic()` 返回 (10,1) ndarray，`dl.T @ dl` 产生 (1,1) 矩阵，NumPy 新版 `float()` 不接受多维数组。

### 解决办法

所有矩阵乘法结果加 `.item()` 转为标量：
```python
float((dl.T @ dl).item())
```

---

## 问题 11: 避障路径超前（离障碍物太远就开始偏移）

### 现象

避障白色曲线在车前方很远就开始偏移，还没到障碍物就出现了大幅度的绕行。

### 原因

1. DP 列间距 sample_s=15m 太大，第一个避障判断点在 15m 外
2. 障碍物距离变为负数（车已通过）后 DP 仍尝试避障
3. 规划路径从预测点开始，与车辆当前位置脱节

### 解决办法

1. sample_s: 15→8m, col: 6→10, sample_l: 1.5→1.0m（网格更密）
2. 过滤 obs_s < -5m 的障碍物（车后方）
3. 路径首点插入车辆当前位置 (s=0)
4. 无障碍时自动切回全局参考线

---

## 问题 12: 游戏窗口刚打开时按键失效

### 现象

pygame 窗口打开后，按小写 r/z 无反应，切换 Caps Lock 大写也无效。

### 根因

同问题 4 的最终根因：`pygame.event.get()` 吞掉所有 KEYDOWN 事件。

### 解决办法

同问题 4 的最终修复：类型过滤 `get()`，统一用 `get_pressed()` 处理按键。

---

## 问题总结表

| # | 问题 | 严重度 | 阶段 | 状态 |
|---|------|--------|------|------|
| 1 | Python 3.14 + pygame 无预编译wheel | 🔴 阻塞 | Phase 1 | ✅ pygame-ce |
| 2 | pygame-ce SysFont TypeError | 🔴 阻塞 | Phase 1 | ✅ monkey-patch |
| 3 | CARLA API 强依赖 | 🟡 架构 | Phase 2 | ✅ VehicleState |
| 4 | 按键完全不响应（3次迭代） | 🔴 阻塞 | Phase 1-3 | ✅ get_pressed+类型过滤 |
| 5 | 道路绘制不对称 | 🟡 视觉 | Phase 2 | ✅ 居中对称 |
| 6 | cvxopt大文件下载慢 | 🟢 效率 | Phase 1 | ✅ 分批安装 |
| 7 | Frenet法向量约定 | 🟡 架构 | Phase 1 | ✅ 统一约定 |
| 8 | Windows spawn子进程崩溃 | 🔴 阻塞 | Phase 3 | ✅ 改用threading |
| 9 | DP规划阻塞主循环 | 🔴 阻塞 | Phase 3 | ✅ 非阻塞轮询 |
| 10 | DP numpy维度错误 | 🔴 阻塞 | Phase 3 | ✅ .item() |
| 11 | 避障路径超前 | 🟡 功能 | Phase 3 | ✅ 加密网格+过滤 |
| 12 | 窗口启动时按键失效 | 🔴 阻塞 | Phase 3 | ✅ 同问题4 |
