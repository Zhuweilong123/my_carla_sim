"""可视化颜色常量"""

# 道路
ROAD_SURFACE = (80, 80, 80)         # 深灰路面
LANE_LINE = (200, 200, 200)         # 白色车道线
LANE_DASH = (180, 180, 180)         # 虚线车道线
ROAD_EDGE = (255, 255, 255)         # 道路边界

# 自车
EGO_COLOR = (255, 136, 0)           # 橙色
EGO_DIRECTION = (255, 200, 0)       # 方向指示

# 障碍物
STATIC_OBS = (0, 100, 255)          # 蓝色-静态
DYNAMIC_OBS = (255, 50, 50)         # 红色-动态
OBS_BORDER = (200, 200, 200)

# 路径/轨迹
REF_PATH_RAW = (0, 200, 0)          # 绿色-原始参考线
REF_PATH_SMOOTH = (0, 255, 100)     # 亮绿-平滑参考线
DP_PATH = (255, 255, 0)             # 黄色-DP路径
QP_PATH = (255, 255, 255)           # 白色-QP路径
PLANNED_TRAJ = (255, 100, 255)      # 紫色-最终规划轨迹

# Debug
MATCH_POINT = (0, 255, 255)         # 青色-匹配点
PROJ_POINT = (100, 0, 0)            # 深红-投影点
PREDICT_POINT = (255, 255, 255)     # 白色-预测点

# HUD
HUD_BG = (0, 0, 0, 40)             # 半透明黑背景
HUD_TEXT = (255, 255, 255)          # 白色文字
HUD_WARNING = (255, 100, 100)       # 警告红

# 背景
BACKGROUND = (30, 30, 30)           # 深色背景
GRID = (50, 50, 50)                 # 网格线
