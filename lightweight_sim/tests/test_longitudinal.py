from lightweight_sim.engine.algorithms.controller.lon_pid import LongitudinalPIDController


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
