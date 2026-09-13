"""Command-line entry point for the lightweight vehicle simulator."""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lightweight_sim.simulator.app import SimulatorApp


SCENARIOS = {
    "default": SimulatorApp._default_config,
    "obstacle": SimulatorApp.straight_with_obstacle,
    "three_lane": SimulatorApp.three_lane_double_obstacle,
    "curve": SimulatorApp.curve_scenario,
}


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run the lightweight vehicle simulator")
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS),
        default="obstacle",
        help="scenario to run",
    )
    args = parser.parse_args(argv)

    config = SCENARIOS[args.scenario]()
    SimulatorApp(config).run()


if __name__ == "__main__":
    main()
