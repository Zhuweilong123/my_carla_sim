import inspect

import pytest

from lightweight_sim.engine.algorithms.controller import (
    LateralController,
    LateralLQRController,
    LateralMPCController,
    LongitudinalController,
    LongitudinalPIDController,
)
from lightweight_sim.engine.algorithms.controller.combined import VehicleController
from lightweight_sim.engine.simulator.steering import SteeringParams

PARAMS = (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0)
PATH = [(float(i), 0.0, 0.0, 0.0) for i in range(100)]


def test_algorithm_contracts_and_sibling_relationship():
    assert inspect.isabstract(LateralController)
    assert inspect.isabstract(LongitudinalController)
    assert issubclass(LateralLQRController, LateralController)
    assert issubclass(LateralMPCController, LateralController)
    assert not issubclass(LateralMPCController, LateralLQRController)
    assert issubclass(LongitudinalPIDController, LongitudinalController)


@pytest.mark.parametrize('dynamic', [False, True])
def test_lqr_and_mpc_preserve_tracking_lifecycle(dynamic):
    steering = SteeringParams(mode='dynamic') if dynamic else SteeringParams()
    lqr = VehicleController(PARAMS, steering_params=steering)
    mpc = VehicleController(PARAMS, controller_type='MPC_controller', steering_params=steering)
    for controller in (lqr, mpc):
        controller.update_ref_path(PATH)
    for i in range(8):
        args = (10 + i * 0.5, 0.2, 0.01, 10, 0.0, 0.0)
        for controller in (lqr, mpc):
            steer, throttle, brake = controller.step(*args, actual_steer=0.02)
            assert abs(steer) <= controller.lat.max_steer
            assert 0 <= throttle <= 1 and 0 <= brake <= 1
        assert mpc.lat.solver_status == 'optimal'
        assert mpc.lat.route_s == pytest.approx(lqr.lat.route_s)
    for controller in (lqr, mpc):
        progress = controller.lat.route_s
        controller.update_ref_path(list(PATH), reset=False)
        assert controller.lat.route_s == progress
        controller.update_ref_path(PATH[5:], reset=False)
        controller.step(15, 0.2, 0.01, 10, 0, 0, actual_steer=0.02)
        assert controller.lat.x_pro == pytest.approx(15)
        controller.update_ref_path(PATH, reset=True)
        assert controller.lat.route_s == 0
        assert all(value == 0 for value in controller.lat.command_history)
        assert controller.lon._previous_error is None


class ConstantLateral(LateralController):
    def set_path(self, path, *, preserve=False):
        self.path = path
        self.preserved = preserve

    def reset_tracking(self):
        self.reset_actuator_history()
        self.was_reset = True

    def control(self, x, y, phi, vx, vy, r, ref_path):
        self.path = ref_path
        return 0.1


class ConstantLongitudinal(LongitudinalController):
    def control(self, current_speed_ms, coupling_accel=0.0):
        self.coupling = coupling_accel
        return 1.5

    def reset(self):
        self.was_reset = True


def test_vehicle_controller_accepts_independent_algorithms():
    lat, lon = ConstantLateral(), ConstantLongitudinal()
    controller = VehicleController(lateral_controller=lat, longitudinal_controller=lon)
    assert controller.lat is lat
    assert controller.lon is lon
    assert controller.step(0, 0, 0, 10, 0, 0) == (0, 0, 0)
    controller.update_ref_path(PATH)
    assert lat.was_reset and lon.was_reset
    controller.set_target_speed(30)
    assert lon.target_speed == 30
    assert controller.step(0, 0, 0, 10, 2, 0.1) == pytest.approx((0.1, 0.5, 0))
    assert lon.coupling == pytest.approx(0.2)
    controller.update_ref_path(PATH[5:], reset=False)
    assert lat.preserved
