import numpy as np

from lightweight_sim.engine.algorithms.controller.lat_lqr import LateralLQRController


VEHICLE_PARAMS = (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0)


def test_lqr_solves_discrete_riccati_equation():
    controller = LateralLQRController(VEHICLE_PARAMS, R=2.0, ts=0.05)
    gain = controller.update_lqr_gain(30.0 / 3.6)

    A, B, P, Q, R = controller.A, controller.B, controller.P, controller.Q, controller.R
    riccati_rhs = (
        A.T @ P @ A
        - A.T @ P @ B @ np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        + Q
    )

    assert gain.shape == (1, 4)
    assert P.shape == (4, 4)
    assert np.all(np.isfinite(gain))
    assert np.allclose(P, P.T, atol=1e-9)
    assert np.allclose(P, riccati_rhs, atol=1e-7)
    assert controller.riccati_converged
    assert controller.riccati_iterations < 500


def test_lqr_weights_are_used_when_computing_gain():
    low_lateral_weight = LateralLQRController(
        VEHICLE_PARAMS, Q=np.diag([10.0, 1.0, 50.0, 1.0])
    )
    high_lateral_weight = LateralLQRController(
        VEHICLE_PARAMS, Q=np.diag([400.0, 1.0, 50.0, 1.0])
    )

    low_gain = low_lateral_weight.update_lqr_gain(30.0 / 3.6)
    high_gain = high_lateral_weight.update_lqr_gain(30.0 / 3.6)

    assert not np.allclose(low_gain, high_gain)
