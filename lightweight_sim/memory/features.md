# 项目功能点归档

> 项目: Lightweight Planning & Control Simulator (轻量级规划控制仿真器)
> 日期: 2026-06-06
> 版本: Phase 3 — 规划控制联调

---

## 一、项目概述

脱离 CARLA 独立运行的轻量级自动驾驶仿真器。将原 CARLA 项目中的规划控制算法解耦为纯 Python 实现，配合 pygame 2D 俯视图进行可视化和交互。

---

## 二、仿真器核心功能

### 2.1 车辆模型

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 运动学自行车模型 | 4状态(x,y,phi,v)，控制量(steer,accel)，20Hz更新 | `simulator/vehicle.py:kinematic_step()` |
| 动力学自行车模型 | 4状态误差动力学(ed,ėd,eφ,ėφ)，可对接LQR/MPC | `simulator/vehicle.py:dynamic_step()` |
| 误差状态计算 | 完全替代CARLA API，通过Frenet投影计算e_rr | `simulator/vehicle.py:get_error_state()` |
| 车辆参数配置 | Tesla Model 3参数(a,b,m,Cf,Cr,Iz) | `simulator/data_types.py:VehicleParams` |
| SAT碰撞检测 | 分离轴定理实现矩形精确碰撞 | `simulator/vehicle.py:check_collision_with_obstacle()` |

### 2.2 道路模型

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 直线道路 | 参数化生成(length, heading) | `simulator/world.py:_generate_straight()` |
| 圆弧道路 | 参数化生成(radius, angle, center) | `simulator/world.py:_generate_arc()` |
| Waypoint道路 | 自定义路点列表 | `simulator/world.py:_generate_segment()` |
| 多段拼接 | 多段RoadSegment组合成完整道路 | `simulator/world.py:_generate_road()` |
| QP参考线平滑 | 三项代价函数(平滑+紧凑+相似)二次规划 | `algorithms/utils/reference_line.py` |
| 弧长映射 | s_map自动计算，支持车辆原点定位 | `simulator/world.py:_build_s_map()` |
| **道路居中绘制** | 参考线=道路中心线，灰色路面填充，车道线对称 | `visualization/renderer.py:draw_road()` |

### 2.3 障碍物系统

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 静态障碍物 | 位置固定的矩形障碍物 | `simulator/data_types.py:Obstacle` |
| 动态障碍物 | 带速度+运动方向的障碍物 | `simulator/data_types.py:Obstacle.step()` |
| 场景配置加载 | 从场景配置字典批量创建障碍物 | `simulator/obstacle.py:add_from_config()` |
| AABB快速剔除 | 轴对齐包围盒预筛选 | `simulator/obstacle.py:check_collision()` |
| SAT精确检测 | 分离轴定理矩形碰撞 | `simulator/obstacle.py:_sat_collision()` |

### 2.4 仿真引擎

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 固定步长 | dt=0.05s (20Hz) 物理更新 | `simulator/engine.py` |
| 碰撞检测 | 每步自动检测自车与所有障碍物 | `simulator/engine.py:step()` |
| 终点判断 | 距离目标点<2m自动触发到达 | `simulator/engine.py` |
| 时间管理 | sim_time + step_count 追踪 | `simulator/engine.py` |

### 2.5 可视化 (`visualization/`)

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 俯视图渲染 | 灰色路面 + 白色车道线 + 绿色参考线 + 彩色轨迹 | `visualization/renderer.py` |
| 车辆绘制 | 旋转矩形 + 方向箭头 | `visualization/renderer.py:draw_vehicle()` |
| 障碍物绘制 | 蓝色(静态)/红色(动态)矩形 + 速度方向线 | `visualization/renderer.py:draw_obstacles()` |
| 相机系统 | 平滑跟随 + 鼠标滚轮缩放(2x-20x) | `visualization/renderer.py:Camera` |
| 网格背景 | 10m间距参考网格 | `visualization/renderer.py:draw_grid()` |
| **道路居中** | 参考线=中心线，路面多边形填充 | `visualization/renderer.py:draw_road()` |
| **DP轨迹渲染** | 黄色点序列=DP粗路径 | `visualization/renderer.py:draw_path()` |
| **QP轨迹渲染** | 白色实线=QP精细避障路径 | `visualization/renderer.py:draw_path()` |
| HUD面板 | FPS/速度/位置/误差/控制量/碰撞状态 | `visualization/hud.py` |
| **模式指示器** | 屏幕顶部大字显示MANUAL/AUTO(LQR/MPC) | `simulator/app.py:_render()` |
| **跟踪误差显示** | HUD 实时 ed (m) / ephi (deg) | `visualization/hud.py` |
| 误差曲线 | 右下角实时e_d(白)/e_φ(黄)历史曲线 | `visualization/hud.py:_draw_error_graph()` |
| **控制器debug标记** | 预测点(白)/投影点(深红)小圆点 | `simulator/app.py:_render()` |
| 颜色常量 | 统一管理所有可视化颜色 | `visualization/colors.py` |

### 2.6 交互控制

| 功能 | 按键 | 说明 |
|------|------|------|
| 手动/自动切换 | **Q** | 去抖防重复触发 |
| 油门 | **W / ↑** | 手动模式 |
| 倒车/减速 | **S / ↓** | 手动模式 |
| 左转 | **A / ←** | 手动模式 |
| 右转 | **D / →** | 手动模式 |
| 刹车 | **Space** | 手动模式 |
| 重置 | **R** | 保留当前控制器类型 |
| 暂停 | **P** | 暂停时仍渲染一帧 |
| 切换LQR/MPC | **M** | 运行时实时切换控制器 |
| 缩放 | **+/- / 鼠标滚轮** | 2x-20x |
| 退出 | **ESC** | |
| **全部按键** | **get_pressed()** | 统一轮询, 不依赖KEYDOWN事件 |

---

## 三、控制算法 (`algorithms/controller/`)

### 3.1 LQR 横向控制

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 自行车模型A/B矩阵 | 4状态误差动力学，双线性变换离散化 | `lat_lqr.py:_cal_A_B()` |
| Riccati方程迭代 | 离散DARE求解，收敛阈值ε=0.1，最大5000次 | `lat_lqr.py:_solve_LQR()` |
| 前馈控制 | 道路曲率补偿，消除弯道稳态误差 | `lat_lqr.py:_cal_feedforward()` |
| 误差状态计算 | Frenet投影+位置预测(0.1s延迟补偿) | `lat_lqr.py:_cal_error_state()` |
| **解耦接口** | `control(x,y,phi,vx,vy,r,ref_path)` — 纯数值输入 | `lat_lqr.py:control()` |
| Q权重 | diag(200, 1, 50, 1)，R=1 | `lat_lqr.py:__init__()` |

### 3.2 MPC 横向控制

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 预测模型 | N=6预测区间, P=2控制区间 | `lat_mpc.py:__init__()` |
| M/C/Cc矩阵构造 | 预测状态序列的紧凑表达 | `lat_mpc.py:_solve_QP()` |
| QP约束 | $-1 \leq \delta \leq 1$ 显式约束 | `lat_mpc.py:_solve_QP()` |
| cvxopt求解 | 内点法，约束Gx≤h | `lat_mpc.py:_solve_QP()` |
| QP失败回退 | 捕获异常返回零转角 | `lat_mpc.py:_solve_QP()` |
| Q权重 | diag(250, 1, 50, 1)，F=I₄终端代价 | `lat_mpc.py:_default_Q()` |

### 3.3 PID 纵向控制

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 纯比例控制 | K_P=1.15, K_I=0, K_D=0 | `lon_pid.py:__init__()` |
| 积分分离 | \|error\| > 1km/h 清零积分 | `lon_pid.py:control()` |
| 滑动窗口 | deque(maxlen=60) 存储误差历史 | `lon_pid.py:control()` |
| 目标速度接口 | `set_target(speed_kmh)` | `lon_pid.py:set_target()` |

### 3.4 横纵向联合 (`combined.py`)

| 功能 | 描述 |
|------|------|
| **门面模式** | `VehicleController` 统一 `step()` 接口 |
| 输入 | (x,y,phi,vx,vy,r) → 输出 (steer,throttle,brake) |
| **M键切换** | 运行时 `_switch_controller()` LQR↔MPC |
| 参考线更新 | `update_ref_path()` 接收规划器输出 |

---

## 四、路径规划算法 (`algorithms/planner/`)

### 4.1 DP 动态规划

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| S-L图采样 | 12×10网格, s间隔8m, l间隔1.0m | `dp_path_plan.py:DP_algorithm()` |
| 五次多项式连接 | 相邻列节点以最小Jerk轨迹连接 | `dp_path_plan.py` |
| 碰撞代价 | $≤4$m: $10^{12}$, $4\sim6$m: $5000/d^2$ | `dp_path_plan.py:_calc_collision()` |
| 平滑代价 | [w_dl=300, w_ddl=1000, w_dddl=5000] | `dp_path_plan.py:cal_start_cost()` |
| 参考线代价 | w_ref=20，偏向l=0 | `dp_path_plan.py` |
| Bellman递推 | `cost[i,j]=min_k(cost[k,j-1]+neighbor_cost)` | `dp_path_plan.py:DP_algorithm()` |
| 路径增密 | 1m分辨率五次多项式插值 | `dp_path_plan.py:enrich_DP_s_l()` |
| 左行惩罚 | 左侧车道+10000代价(右行规则) | `dp_path_plan.py:DP_algorithm()` |

### 4.2 QP 二次规划

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| 等式约束 | 相邻点三阶连续性(l,l',l'') | `qp_path_plan.py:Quadratic_planning()` |
| 车辆形状约束 | 车头3m+车尾3m 矩形8不等式 | `qp_path_plan.py` |
| 凸空间中心代价 | w_centre=250，趋向可行域中心 | `qp_path_plan.py` |
| 终点正则化 | l→0, dl→0, ddl→0 | `qp_path_plan.py` |
| l_min/l_max边界 | 根据DP路径+障碍物位置自动计算 | `qp_path_plan.py:cal_lmin_lmax()` |
| SL→XY逆变换 | Frenet投影→笛卡尔坐标+平滑 | `qp_path_plan.py:frenet_2_x_y_theta_kappa()` |

### 4.3 MotionPlanner 调度器

| 功能 | 描述 | 实现位置 |
|------|------|----------|
| **后台线程** | threading.Thread，避免Windows spawn问题 | `motion_planner.py:MotionPlanner` |
| 请求/响应队列 | `queue.Queue` 非阻塞通信 | `motion_planner.py` |
| 规划管道 | match→sample→QP smooth→DP→QP→XY | `motion_planner.py:_planning_thread()` |
| **障碍物过滤** | obs_s < -5m（车辆后方）自动剔除 | `motion_planner.py` |
| **空结果处理** | 无障碍时自动切回参考线 | `motion_planner.py` |
| 非阻塞轮询 | `poll_result()` + `get_result()` | `motion_planner.py` |
| 自动重启 | 线程崩溃自动重建 | `motion_planner.py:plan()` |

---

## 五、算法工具库 (`algorithms/utils/`)

| 功能 | 描述 | 来源 |
|------|------|------|
| 航向角/曲率计算 | 中点欧拉法O(h²)，sin近似避免多值 | `geometry.py` |
| 五次多项式 | 6边界条件→6系数矩阵求逆 + evaluate | `quintic.py` |
| 参考线QP平滑 | 三项代价cvxopt求解，±0.2m约束 | `reference_line.py` |
| Frenet坐标变换 | 匹配点/投影点/s/l/导数全套 | `frenet.py` |

---

## 六、预设场景

| 场景 | 道路 | 障碍物 | 终点 | 用途 |
|------|------|--------|------|------|
| `_default_config()` | 200m直道, 2车道 | 无 | (190, 0) | LQR巡航 |
| `straight_with_obstacle()` | 1000m直道, 2车道 | 1个静态车@60m | (190, 0) | **DP+QP避障** |
| `curve_scenario()` | 50m直+90°圆弧+100m直 | 无 | 无 | LQR过弯 |

---

## 七、仿真主循环架构

```
┌─ pygame.event.pump()         ← 刷新事件队列
├─ pygame.event.get(QUIT)      ← 只取退出(不消费KEYDOWN)
├─ pygame.event.get(MOUSEWHEEL)← 只取滚轮
├─ keys = get_pressed()        ← 一次性读取所有按键
│   ├─ ESC → 退出
│   ├─ Q/P/R/M → 去抖切换
│   └─ WASD → 连续控制
├─ 自动模式: LQR/MPC + PID 控制
├─ 规划(每50步): 线程异步 DP+QP
│   ├─ send request → 后台线程
│   └─ poll_result → 非阻塞接收
├─ 物理步进: dt=0.05s
├─ 渲染: 道路+车辆+障碍物+轨迹+HUD
└─ clock.tick(60) → 60fps
```
