import pytest

from lightweight_sim.tests.configuration import speed_tracking_limits


@pytest.mark.parametrize('speed,expected', [
    (20.0, dict(rms=.3, p95=.5, peak=1.0)),
    (60.0, dict(rms=.6, p95=1.0, peak=2.0)),
    (120.0, dict(rms=.6, p95=1.0, peak=2.0)),
])
def test_speed_tuning_preserves_absolute_quality_caps(acceptance_configuration, speed, expected):
    budget = acceptance_configuration['speed_tracking']
    assert speed_tracking_limits(budget, speed) == pytest.approx(expected)


@pytest.mark.parametrize('key,value', [
    ('settling_time_s', -1.0),
    ('settling_time_s', float('nan')),
    ('rms_target_fraction', 0.0),
    ('p95_target_fraction', 1.1),
    ('peak_max_kmh', float('inf')),
    ('peak_max_kmh', .1),
])
def test_invalid_tracking_budget_is_rejected(acceptance_configuration, key, value):
    budget = dict(acceptance_configuration['speed_tracking'])
    budget[key] = value
    with pytest.raises(ValueError):
        speed_tracking_limits(budget, 60.0)
