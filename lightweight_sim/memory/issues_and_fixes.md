# 遇到的问题及解决办法归档

> 项目: Lightweight Planning & Control Simulator  
> 日期: 2026-06-06

---

## 问题 1: Python 3.14 + pygame 安装失败

### 现象

```bash
pip install pygame
# ERROR: Failed to build 'pygame' when getting requirements to build wheel
# FileNotFoundError: [WinError 2] 系统找不到指定的文件 (pacman)
```

### 原因分析

1. Python 3.14.5 是 2025 年底发布的最新版本，`pygame` 官方尚未发布 cp314 的预编译 wheel
2. pip 自动回退到源码编译（`pygame-2.6.1.tar.gz`）
3. 编译过程需要 MSYS2 环境中的 `pacman` 安装 SDL2 等依赖，Windows 系统缺少该工具

### 解决办法

**使用 `pygame-ce`（Community Edition）替代原版 `pygame`**：

```bash
pip install pygame-ce
```

- `pygame-ce` 有 cp314 预编译 wheel (`pygame_ce-2.5.7-cp314-cp314-win_amd64.whl`, 10.6MB)
- API 完全兼容原版 pygame，`import pygame` 导入方式不变
- 社区版维护更活跃，Bug 修复更快

### 相关文件

- 安装命令: `pip install numpy cvxopt networkx pygame-ce`

---

## 问题 2: pygame-ce 在 Python 3.14 上 SysFont 崩溃

### 现象

```python
pygame.font.SysFont('consolas', 20)
# TypeError: expected str, bytes or os.PathLike object, not int
#   File "pygame/sysfont.py", line 80, in initsysfonts_win32
#     if splitext(font)[1].lower() not in OpenType_extensions:
```

### 原因分析

1. `initsysfonts_win32()` 遍历 Windows 注册表字体信息时，Python 3.14 的 Win32 API 返回了部分整数类型的值
2. `os.path.splitext()` 函数期望字符串参数，遇到整数后抛出 `TypeError`
3. Python 3.14 加强了对 `os.path` 函数的类型检查（之前版本可能隐式转换）

根本原因: **Python 3.14 的 `os.path.splitext()` 不再接受非字符串参数**

### 解决办法

在 `simulator/app.py` 中，**在任何 pygame 初始化之前** monkey-patch 两个函数：

```python
def _patch_pygame_sysfont():
    """Monkey-patch pygame.sysfont for Python 3.14 compatibility."""
    import pygame.sysfont as _sf

    # Patch 1: 替换 initsysfonts_win32，过滤非字符串字体名
    _orig_initsysfonts_win32 = _sf.initsysfonts_win32
    def _safe_initsysfonts_win32():
        fonts = {}
        try:
            result = _orig_initsysfonts_win32()
            for name, path in result.items():
                if isinstance(name, str) and isinstance(path, str):
                    fonts[name] = path
        except Exception:
            pass
        return fonts
    _sf.initsysfonts_win32 = _safe_initsysfonts_win32

    # Patch 2: SysFont 构造函数捕获 TypeError 回退到默认字体
    _orig_SysFont_init = _sf.SysFont.__init__
    def _safe_SysFont_init(self, name, size, bold=False, italic=False):
        try:
            _orig_SysFont_init(self, name, size, bold, italic)
        except TypeError:
            self.__dict__.clear()
            pygame.font.Font.__init__(self, None, size)
    _sf.SysFont.__init__ = _safe_SysFont_init

_patch_pygame_sysfont()  # 模块导入时立即执行
```

**补充措施**: HUD 和 Renderer 中的字体创建添加 try/except 回退：

```python
try:
    self.font = pygame.font.SysFont("consolas", font_size)
except Exception:
    self.font = pygame.font.Font(None, font_size)  # 使用默认字体
```

### 副作用

- 自定义字体名（如 'consolas'）无法使用，自动回退到 pygame 默认字体
- 视觉效果略有下降（字体不同），但不影响功能

### 相关文件

- `simulator/app.py:15-55` — monkey-patch 代码
- `visualization/hud.py:20-31` — HUD 字体回退
- `visualization/renderer.py:29-32` — 渲染器字体回退

---

## 问题 3: CARLA API 强依赖导致算法无法独立运行

### 现象

原项目中的控制器需要 CARLA 仿真器才能运行：

```python
# 原代码: controller/Controller.py
vehicle_loc = self._vehicle.get_location()  # 依赖 carla.Vehicle
V = self._vehicle.get_velocity()            # 依赖 carla.Vehicle
fi = self._vehicle.get_transform().rotation.yaw * (math.pi / 180)
```

### 解决办法

**设计统一的数据接口 `VehicleState`**，完全替代 CARLA API 调用：

1. 创建 `VehicleState` dataclass 存储车辆状态（x, y, phi, vx, vy, r, steer, accel, timestamp）
2. 在 `EgoVehicle` 中实现 `get_error_state(ref_path)` 方法
3. 该方法内部调用 `algorithms/utils/frenet.py` 的坐标变换函数计算 `e_rr = [ed, ėd, eφ, ėφ]`
4. 控制器新增 `_control_from_state(state)` 接口，从 VehicleState 直接取值

```python
# 新代码: simulator/vehicle.py
def get_error_state(self, ref_path, ts=0.1):
    s = self._state
    # 预测补偿
    pred_x = s.x + s.vx * ts * cos(s.phi) - s.vy * ts * sin(s.phi)
    pred_y = s.y + s.vy * ts * cos(s.phi) + s.vx * ts * sin(s.phi)
    # Frenet投影 → 匹配点 → 计算误差
    match_idx, proj_list = find_match_points([(pred_x, pred_y)], ref_path)
    # ... 计算 ed, ėd, eφ, ėφ
    return np.array([ed, ed_dot, ephi, ephi_dot])
```

### 相关文件

- `simulator/data_types.py` — VehicleState 定义
- `simulator/vehicle.py` — get_error_state() 实现
- `algorithms/utils/frenet.py` — Frenet 坐标变换（从 planner_utiles.py 迁移）

---

## 问题 4: 大文件下载超时

### 现象

`pip install` 时下载 cvxopt (13.8MB) 和 pygame-ce (10.6MB) 耗时超过 5 分钟。

### 原因

网络环境带宽有限（约 40-50 KB/s），且 pip 默认无超时限制。

### 解决办法

1. 使用 `pip install --default-timeout=300` 增加超时时间
2. 分批安装（先安装小包，再安装大包）
3. 验证时每个包单独安装，确认哪个包慢

### 最终安装耗时

| 包 | 大小 | 耗时 |
|----|------|------|
| numpy | 12.5 MB | ~5s (本地缓存命中) |
| cvxopt | 13.8 MB | ~6 min |
| networkx | 2.1 MB | ~55s |
| pygame-ce | 10.6 MB | ~5.5 min |

---

## 问题 5: Frenet 坐标系的法向量方向约定

### 现象

原 CARLA 代码中多处标注 `***************************************` 提示法向量方向可能存在歧义。

### 原因

CARLA 使用 UE4（Unreal Engine 4）的**左手坐标系**：
- X 轴向前
- Y 轴向右
- Z 轴向上

在左手系中，法向量 `n = [-sin(θ), cos(θ)]` 的含义与右手系不同：
- CARLA/UE4: 车辆在参考线**左侧**时，$l < 0$
- 标准数学: 车辆在切线方向**左侧**时，$l > 0$

### 解决办法

1. 在 `algorithms/utils/frenet.py` 中保持一致使用 `n_r = [-sin(θ), cos(θ)]`
2. `simulator/world.py` 中的车道编号：`lane_idx=0` 为最右侧车道（l > 0）
3. 代码注释中标注约定

### 相关文件

- `algorithms/utils/frenet.py:103-105` — 法向量计算
- `simulator/world.py:104` — 车道偏移约定

---

## 问题 6: cvxopt QP 求解器在极端情况下崩溃

### 风险 (未实际触发)

当 `Vx ≈ 0` 时，动力学模型的 A 矩阵某些元素 → ∞ 或出现奇异，导致离散化后的 H 矩阵不正定，cvxopt 无法求解。

### 预置缓解措施

1. **Vx 下限保护**: `Vx = Vx + 0.0001` 防止除零
2. **H 矩阵正定性**: 权重全为正值，保证 H ≻ 0
3. **QP 失败回退**: 规划层捕获异常时可回退到 DP 路径

```python
# controller/Controller.py 中的保护
if Vx < 0:
    Vx = -max(abs(Vx), 0.005)
else:
    Vx = max(Vx, 0.005)
```

---

## 问题 7: 运动学模型在高速/大曲率下精度不足

### 风险 (未实际触发)

运动学自行车模型不考虑轮胎侧偏，在高速（>60km/h）或大曲率（转弯半径 < 50m）场景下，与实际物理的偏差会增大。

### 预置缓解措施

1. `EgoVehicle` 同时实现了 `dynamic_step()` 方法，可随时切换
2. 车辆参数 `VehicleParams` 包含轮胎侧偏刚度（Cf, Cr），可用于动力学模式
3. 调用接口统一为 `ego.step(steer, accel, dt, model="kinematic")`
4. 切换只需将 `model="kinematic"` 改为 `model="dynamic"`

### 相关文件

- `simulator/vehicle.py:58-88` — kinematic_step()
- `simulator/vehicle.py:90-115` — dynamic_step()

---

## 问题总结表

| # | 问题 | 严重度 | 解决状态 |
|---|------|--------|----------|
| 1 | Python 3.14 + pygame 无预编译wheel | 🔴 阻塞 | ✅ 已解决 (换用pygame-ce) |
| 2 | pygame-ce SysFont 在Python 3.14崩溃 | 🔴 阻塞 | ✅ 已解决 (monkey-patch) |
| 3 | CARLA API强依赖 | 🟡 架构 | ✅ 已解决 (VehicleState接口) |
| 4 | pip大文件下载慢 | 🟢 效率 | ✅ 已解决 (分批安装) |
| 5 | Frenet法向量方向约定不一致 | 🟡 架构 | ✅ 已解决 (统一约定+注释) |
| 6 | cvxopt QP在极端情况崩溃 | 🟡 鲁棒性 | ⚠️ 已预防 (未触发) |
| 7 | 运动学模型高速/大曲率精度 | 🟡 精度 | ⚠️ 已预防 (保留动力学接口) |
