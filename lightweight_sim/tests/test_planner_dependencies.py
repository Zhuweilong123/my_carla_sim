"""Keep solver dependencies confined to the selected planning algorithm."""
import subprocess
import sys


def test_geometry_and_baseline_load_without_scipy():
    script = '''
import importlib.abc
import sys

class BlockScipy(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "scipy" or fullname.startswith("scipy."):
            raise ModuleNotFoundError("SciPy deliberately unavailable")

sys.meta_path.insert(0, BlockScipy())
from lightweight_sim.engine.reference_line import ReferenceLineCore
from lightweight_sim.engine.simulator.world import World
from lightweight_sim.engine.algorithms.planner.st_speed import SpeedPlan
from lightweight_sim.engine.algorithms.controller.combined import VehicleController
from lightweight_sim.engine.reference_line import create_local_planner
options = dict(global_frenet_path=[(0, 0, 0, 0), (100, 0, 0, 0)],
               lane_width=3.5, num_lanes=2, reference_lane_index=0, target_lane=0)
baseline = create_local_planner("baseline", **options)
assert baseline._plan((0, 0), (0, 0), [])
assert "scipy" not in sys.modules
try:
    create_local_planner("dp_qp", **options)
except RuntimeError as exc:
    assert "sudo apt install python3-scipy" in str(exc)
else:
    raise AssertionError("DP+QP must explicitly require the solver dependency")
'''
    result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
