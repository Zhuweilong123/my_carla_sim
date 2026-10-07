from lightweight_sim.engine.algorithms.controller.lon_pid import LongitudinalPIDController
import pytest


def test_speed_controller_limits_acceleration_slew():
    controller = LongitudinalPIDController(max_jerk=4.0)
    controller.set_target(50.0)

    first_accel = controller.control(0.0)
    second_accel = controller.control(0.0)

    assert first_accel == 0.2
    assert second_accel == 0.4


def test_speed_controller_rejects_dynamic_lateral_coupling():
    without_compensation = LongitudinalPIDController(coupling_gain=0.0)
    with_compensation = LongitudinalPIDController(coupling_gain=1.0)

    without = without_compensation.control(50.0 / 3.6, coupling_accel=0.8)
    compensated = with_compensation.control(50.0 / 3.6, coupling_accel=0.8)

    assert compensated < without


def test_speed_tracking_resumes_from_actual_protective_braking():
    controller = LongitudinalPIDController(max_jerk=6.0, dt=0.05)
    controller.set_target(36.0)
    controller.reset()
    # The supervisor applied -6 m/s² while PID memory was reset to zero.
    resumed = controller.control(10.0, reference_accel=-5.7, actual_accel=-6.0)
    assert resumed == pytest.approx(-5.7)
    assert abs(resumed - (-6.0)) <= controller.max_jerk * controller.dt + 1e-9
    # A subsequent protective reset must also replace candidate history.
    controller.reset()
    overridden = controller.control(10.0, reference_accel=-2.0, actual_accel=1.0)
    assert overridden == pytest.approx(0.7)
    with pytest.raises(ValueError, match='measured acceleration'):
        controller.control(10.0, actual_accel=float('nan'))
