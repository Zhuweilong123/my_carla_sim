import re
from pathlib import Path

import pytest

from lightweight_sim.engine.runtime_config import DEFAULT_RUNTIME_CONFIG


def test_shared_runtime_defaults_match_fixed_step_planning():
    runtime = DEFAULT_RUNTIME_CONFIG

    assert runtime.physics_dt == pytest.approx(0.05)
    assert runtime.dynamic_max_substep_s == pytest.approx(0.0025)
    assert runtime.local_transition_distance_m == pytest.approx(12.0)
    assert runtime.control_period == pytest.approx(runtime.physics_dt)
    assert runtime.plan_interval_steps() == 1


def test_shared_runtime_speed_defaults_match_ros_configuration():
    runtime = DEFAULT_RUNTIME_CONFIG
    config_path = Path(__file__).parents[1] / "config" / "default.yaml"
    yaml_text = config_path.read_text(encoding="utf-8")

    shared_parameters = {
        "default_speed_limit_kmh": runtime.default_speed_limit_kmh,
        "straight_speed_limit_kmh": runtime.straight_speed_limit_kmh,
        "curve_speed_limit_kmh": runtime.curve_speed_limit_kmh,
        "intersection_speed_limit_kmh": runtime.intersection_speed_limit_kmh,
        "lane_change_speed_limit_kmh": runtime.lane_change_speed_limit_kmh,
        "parking_speed_limit_kmh": runtime.parking_speed_limit_kmh,
        "target_speed_ratio": runtime.target_speed_ratio,
        "speed_profile_lookahead_m": runtime.speed_profile_lookahead_m,
        "max_lateral_accel_mps2": runtime.max_lateral_accel_mps2,
        "planned_path_timeout_s": runtime.safe_stop_plan_timeout_s,
        "safety_stop_timeout_s": runtime.controller_safety_heartbeat_timeout_s,
    }
    for name, expected in shared_parameters.items():
        matches = re.findall(
            rf"(?m)^\s+{re.escape(name)}:\s*([-+]?\d+(?:\.\d+)?)\s*$",
            yaml_text,
        )
        assert matches, f"{name} is missing from config/default.yaml"
        assert float(matches[0]) == pytest.approx(expected), name
