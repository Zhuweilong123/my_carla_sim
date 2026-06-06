"""Lightweight Planning & Control Simulator — 入口"""

import sys
import os

# 确保项目根目录在 path 中
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lightweight_sim.simulator.app import SimulatorApp


def main():
    """启动仿真器

    可用场景:
      SimulatorApp._default_config()          — 200m直道, 无障碍 (LQR巡航)
      SimulatorApp.straight_with_obstacle()   — 直道+静态障碍 (需Phase3路径规划避障)
      SimulatorApp.curve_scenario()           — 90°弯道 (LQR过弯)
    """
    # 三车道双障碍物连续避障
    # 可选: _default_config() / straight_with_obstacle() / curve_scenario()
    app = SimulatorApp(SimulatorApp.three_lane_double_obstacle())
    app.run()


if __name__ == "__main__":
    main()
