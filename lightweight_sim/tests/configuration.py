"""Read the same parameter layers used by the standard ROS launch."""
from pathlib import Path
import math

import yaml


CONFIG_DIR = Path(__file__).parents[1] / "config"
CONFIG_FILES = (
    "system.yaml", "vehicle.yaml", "algorithms.yaml", "baseline.yaml",
    "compatibility.yaml", "default.yaml",
)


def load_configuration(config_dir=CONFIG_DIR):
    merged = {}
    for name in CONFIG_FILES:
        document = yaml.safe_load((Path(config_dir) / name).read_text(encoding="utf-8"))
        for selector, section in document.items():
            assert selector.startswith('/**/'), f'Namespace-dependent selector: {selector}'
            node = selector.removeprefix('/**/')
            target = merged.setdefault(node, {})
            parameters = section["ros__parameters"]
            assert not target.keys() & parameters.keys(), f"Duplicate parameters for {node} in {name}"
            target.update(parameters)
    return merged


def speed_tracking_limits(budget, target_speed_kmh):
    """Keep speed tuning from automatically relaxing absolute quality caps."""
    speed = float(target_speed_kmh)
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('tracking target speed must be positive and finite')
    settling = float(budget['settling_time_s'])
    if not math.isfinite(settling) or settling < 0:
        raise ValueError('tracking settling time must be non-negative and finite')
    limits = {}
    for name in ('rms', 'p95', 'peak'):
        fraction = float(budget[name + '_target_fraction'])
        cap = float(budget[name + '_max_kmh'])
        if not (math.isfinite(fraction) and 0 < fraction <= 1
                and math.isfinite(cap) and cap > 0):
            raise ValueError(f'invalid {name} speed tracking budget')
        limits[name] = min(speed * fraction, cap)
    if not limits['rms'] <= limits['p95'] <= limits['peak']:
        raise ValueError('tracking budgets must satisfy rms <= p95 <= peak')
    return limits
