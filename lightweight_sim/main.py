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
    # 直道+障碍物场景 (Phase 3: DP+QP 自动避障)
    app = SimulatorApp(SimulatorApp.straight_with_obstacle())
    app.run()


if __name__ == "__main__":
    main()
