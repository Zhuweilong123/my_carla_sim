#   -*- coding: utf-8 -*-
# @Author  : Weilong Zhu
# @Time    : 2022-06-27,14:25
# @File    : motion_plan_speed_planning.py


"""
本文件解决的是运动规划下的速度规划问题，解决动态障碍物问题
在已知笛卡尔坐标系下的路径之后，进行速度规划的四个步骤：
1.直接以笛卡尔坐标的path为坐标轴，建立frenet坐标系
2.动态障碍物投影到frenet坐标系上，生成ST图
3.速度决策，动态规划
4.速度规划，二次规划
"""


def construct_ST_graph_and_planning(path_s: list, ego_pos: int, ego_speed: int, obs_pos: list, obs_speed: list, time_interval=8):
    """
    我们每次规划的s的长度为100米，获取当前的速度，假设当前车辆通过这段距离是匀速运行的，所以可以预估时间区间T
    T = 100/cur_speed，市区的速度最高也就是50km/h, 13.888m/s, 因此如果道路通行不受限（无障碍物，无限速）保持最高速度匀速大概7.2s通过，
    我们取8s的时间间隔
    先假设车辆在往相同的方向行驶，只有超车和减速跟随两种决策，在实际中如其实只有往往是选择超车，因为如果前方车辆速度大于自车时，我们不认为前车是障碍物
    前车速度小于或者等于自车时，我们才会选择换道超车，先做出换道超车的效果

    实现步骤
    1.计算障碍物未来的每个时间所在的位置
    2.计算自车未来每个时间所在的位置
    3.找到这些未知的相交区域
    4.采样并且计算每个点的cost
    5.根据每个点的cost动态规划
    6.回溯找到最优的决策
    :param path_s: 路径规划所得到的轨迹s
    :param ego_pos: 自车的位置
    :param ego_speed: 自车的速度
    :param obs_pos:  动态障碍物的当前位置
    :param obs_speed:  动态障碍物的当前速度
    :param time_interval:  S-T图的时间区间
    :return:  返回每个时间的期望位置
    """
    # 计算自车在8秒时间区间的预测位置,考虑规划起点位置
    ego_predict_pos_list = []
    for t in range(time_interval+1):
        ego_predict_pos_list.append(ego_pos + t*ego_speed)

    # 计算障碍物在8秒时间区间的预测位置，考虑初始起点位置
    obs_predict_pos_list = []  # 列表存储每个障碍物的预测位置
    # 列表的每个位置对应的是一个动态障碍物的预测列表[[障碍物1的预测],[障碍物2的预测]]
    for obs_n in range(len(obs_pos)):
        cur_obs = []
        for t in range(time_interval + 1):
            cur_obs.append(obs_pos + t * obs_speed)
        obs_predict_pos_list.append(cur_obs)
    # 由于选择的时间间隔是1s，因此









