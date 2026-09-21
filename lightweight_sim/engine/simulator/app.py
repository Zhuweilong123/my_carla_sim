"""Public simulator application entry point."""

import importlib
import sys

_visualization = importlib.import_module("lightweight_sim.visualization")
sys.modules.setdefault("lightweight_sim.engine.visualization", _visualization)
for _name in ("colors", "hud", "renderer", "ros_gui"):
    _module = importlib.import_module(f"lightweight_sim.visualization.{_name}")
    sys.modules.setdefault(f"lightweight_sim.engine.visualization.{_name}", _module)

from ._app_impl import *  # noqa: F401,F403,E402

from ..runtime_config import DEFAULT_RUNTIME_CONFIG
from . import _app_impl as _app_impl_module
from .reverse_engine import SimulationEngine as _RuntimeSimulationEngine

# Keep the application implementation unchanged while routing both front ends
# through the reverse-capable engine adapter.
_app_impl_module.SimulationEngine = _RuntimeSimulationEngine


_LegacySimulatorApp = SimulatorApp


class SimulatorApp(_LegacySimulatorApp):
    """Standalone adapter using the shared runtime timing defaults.

    The implementation remains in ``_app_impl``; this public adapter applies
    the same physics/control/planning timing used by the ROS 2 nodes.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._apply_runtime_defaults()

    def _apply_runtime_defaults(self):
        self.engine.physics_dt = self.config.physics_dt
        self.controller.lat.ts = self.engine.physics_dt
        self.controller.lon.dt = self.engine.physics_dt
        self._plan_interval = DEFAULT_RUNTIME_CONFIG.plan_interval_steps(
            self.engine.physics_dt
        )

    def _reset(self):
        super()._reset()
        self._apply_runtime_defaults()
