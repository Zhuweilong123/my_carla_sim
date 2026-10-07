"""Check physical settings across independent model/controller boundaries."""
import math
from dataclasses import replace

import numpy as np
import pytest

from lightweight_sim.engine.algorithms.controller.combined import VehicleController
from lightweight_sim.engine.algorithms.controller.lat_lqr import LateralLQRController
from lightweight_sim.engine.algorithms.controller.lon_pid import LongitudinalPIDController
from lightweight_sim.engine.simulator.data_types import ControlCommand, VehicleParams
from lightweight_sim.engine.simulator.engine import SimulationEngine
from lightweight_sim.engine.simulator.scenarios import make_scenario
from lightweight_sim.engine.simulator.steering import steering_profile


@pytest.mark.parametrize('dt', [.025, .1])
def test_engine_default_step_and_reset_use_configured_period(dt):
    config = make_scenario('default')
    config.physics_dt = dt
    config.steering = replace(steering_profile('assumed'), delay_s=dt*2)
    engine = SimulationEngine(config)
    state = engine.step(ControlCommand(steer=.1))
    assert state.timestamp == pytest.approx(dt)
    assert len(engine.steering.history_snapshot(dt)) == 2
    engine.reset()
    assert engine.step().timestamp == pytest.approx(dt)


@pytest.mark.parametrize('dt', [0., -.1, float('nan'), float('inf')])
def test_engine_rejects_invalid_period_before_running(dt):
    config = make_scenario('default')
    config.physics_dt = dt
    with pytest.raises(ValueError, match='physics_dt'):
        SimulationEngine(config)


def test_custom_vehicle_limits_period_and_units_reach_both_controllers():
    params = VehicleParams(a=1.2, b=2., width=2.3, body_overhang=1.2,
                           max_steer=.3, max_accel=2., max_decel=4.)
    config = make_scenario('default')
    config.vehicle_params, config.physics_dt = params, .1
    engine = SimulationEngine(config)
    controller = VehicleController(vehicle_params=params, dt=.1, target_speed_kmh=36.)
    assert engine.ego.length == pytest.approx(4.4)
    assert engine.ego.width == params.width
    assert controller.lat._model().a == params.a
    assert controller.lat._model().b == params.b
    assert controller.lat.max_steer == params.max_steer
    assert controller.lat.ts == controller.lon.dt == engine.physics_dt
    controller.update_ref_path([(0., 0., 0., 0.), (1000., 0., 0., 0.)])
    _, throttle, brake = controller.step(20., 0., 0., 0., 0., 0.)
    assert throttle == 1. and brake == 0.
    controller.set_target_speed(0.)
    controller.lon.reset()
    _, throttle, brake = controller.step(20., 0., 0., 10., 0., 0.)
    assert throttle == 0. and brake == 1.


def test_controller_rejects_conflicting_physical_parameters_and_unknown_algorithm():
    params = VehicleParams()
    with pytest.raises(ValueError, match='disagrees'):
        VehicleController(vehicle_para=replace(params, a=1.3).lateral_tuple, vehicle_params=params)
    with pytest.raises(ValueError, match='unknown lateral controller'):
        VehicleController(controller_type='MCP_controller')


@pytest.mark.parametrize('value', [0., -.1, float('nan'), float('inf')])
def test_controllers_reject_invalid_period(value):
    with pytest.raises(ValueError):
        VehicleController(dt=value)
    with pytest.raises(ValueError):
        LongitudinalPIDController(dt=value)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -1.])
def test_lqr_and_pid_reject_invalid_costs_and_gains(value):
    with pytest.raises(ValueError):
        LateralLQRController(VehicleParams().lateral_tuple, R=value)
    with pytest.raises(ValueError):
        LateralLQRController(VehicleParams().lateral_tuple, Q=np.diag([value, 1., 1., 1.]))
    with pytest.raises(ValueError):
        LongitudinalPIDController(max_jerk=value)
    with pytest.raises(ValueError):
        LongitudinalPIDController(K_P=value)


def test_pid_jerk_limit_uses_configured_period():
    controller = LongitudinalPIDController(dt=.1, max_jerk=2.)
    controller.set_target(36.)
    assert controller.control(0.) == pytest.approx(.2)
    assert controller.control(0.) == pytest.approx(.4)
