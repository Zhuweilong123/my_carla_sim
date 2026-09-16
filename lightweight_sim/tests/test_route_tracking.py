import math
import pytest

from lightweight_sim.engine.algorithms.controller.combined import VehicleController
from lightweight_sim.engine.algorithms.utils.route import RouteTracker
from lightweight_sim.engine.ros_nodes.route_session import encode_sequence, decode_sequence

PARAMS = (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0)


def eight(count):
    return [(78*math.cos(t), 42*math.sin(2*t), 0.0, 0.0)
            for t in [2*math.pi*i/count for i in range(count+1)]]


@pytest.mark.parametrize("count", [120, 480])
def test_crossings_and_seam_keep_metric_progress_for_two_laps(count):
    tracker = RouteTracker(eight(count), start_s=0.0)
    previous = 0.0
    for i in range(1, 2401):
        t = 4*math.pi*i/2400
        x, y = 78*math.cos(t), 42*math.sin(2*t)
        heading = math.atan2(84*math.cos(2*t), -78*math.sin(t))
        p = tracker.update(x, y, heading, speed=15, dt=0.05)
        assert -0.1 < p.s-previous < 2.0
        assert p.distance < 0.12
        previous = p.s
    assert tracker.s == pytest.approx(2*tracker.geometry.length, abs=0.2)


def test_repeated_path_preserves_progress_and_explicit_reset_clears_it():
    controller = VehicleController(PARAMS)
    path = [(float(i), 0.0, 0.0, 0.0) for i in range(100)]
    controller.update_ref_path(path)
    controller.step(50, 0, 0, 10, 0, 0)
    progress = controller.lat.route_s
    controller.update_ref_path(list(path), reset=False)
    assert controller.lat.route_s == progress
    controller.update_ref_path(path[40:], reset=False)
    controller.step(51, 0, 0, 10, 0, 0)
    assert controller.lat.x_pro == pytest.approx(51.5)
    controller.update_ref_path(path, reset=True)
    assert controller.lat.route_s == 0
    assert controller.lon._previous_error is None


def test_run_and_plan_version_do_not_alias():
    assert decode_sequence(encode_sequence(1234, 99)) == (1234, 99)
    assert encode_sequence(1235) > encode_sequence(1234, 9999)
    with pytest.raises(ValueError):
        encode_sequence(1234, 1 << 20)
