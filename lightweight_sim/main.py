"""Lightweight Planning & Control Simulator — 入口"""

import sys
import os

# 确保项目根目录在 path 中
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lightweight_sim.simulator.app import SimulatorApp


def main():
    """启动仿真器"""
    # 可选场景:
    #   SimulatorApp()                                    默认直道
    #   SimulatorApp(SimulatorApp.straight_with_obstacle())  直道+障碍
    #   SimulatorApp(SimulatorApp.curve_scenario())           弯道

    app = SimulatorApp(SimulatorApp.straight_with_obstacle())
    app.run()


if __name__ == "__main__":
    main()
