import pytest

from lightweight_sim.engine.runtime_config import DEFAULT_RUNTIME_CONFIG


def test_shared_runtime_defaults_match_fixed_step_planning():
    runtime = DEFAULT_RUNTIME_CONFIG

    assert runtime.physics_dt == pytest.approx(0.05)
    assert runtime.control_period == pytest.approx(runtime.physics_dt)
    assert runtime.plan_interval_steps() == 10
