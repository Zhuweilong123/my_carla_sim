"""Public renderer module with a stable visualization import boundary."""

import importlib
import sys

_simulator = importlib.import_module("lightweight_sim.engine.simulator")
sys.modules.setdefault("lightweight_sim.simulator", _simulator)
for _name in ("data_types", "obstacle", "world", "vehicle", "engine", "scenarios"):
    _module = importlib.import_module(f"lightweight_sim.engine.simulator.{_name}")
    sys.modules.setdefault(f"lightweight_sim.simulator.{_name}", _module)

from ._renderer_impl import *  # noqa: F401,F403,E402
