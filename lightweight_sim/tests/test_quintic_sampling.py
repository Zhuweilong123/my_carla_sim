import math

import numpy as np
import pytest

from lightweight_sim.engine.algorithms.planner.motion_planner import MotionPlanner
from lightweight_sim.engine.algorithms.utils.quintic import (
    evaluate_quintic, quintic_transition, sample_quintic_path,
)
from lightweight_sim.engine.reference_line import RouteAwareMotionPlanner


def test_normalized_quintic_boundary_conditions_and_monotonicity():
    u = np.linspace(0, 1, 101)
    values = quintic_transition(u)
    assert values[0] == 0 and values[-1] == 1
    assert np.all(np.diff(values) >= 0)
    assert quintic_transition([-1, 2]) == pytest.approx([0, 1])
    l, dl, ddl, _ = evaluate_quintic([0, 0, 0, 10, -15, 6], np.array([0., 1.]))
    assert l == pytest.approx([0, 1])
    assert dl == pytest.approx([0, 0])
    assert ddl == pytest.approx([0, 0])
    assert quintic_transition(0.25) == pytest.approx(0.103515625)


def test_sparse_reference_is_densified_along_quintic_and_keeps_endpoint():
    reference = [(0., 0., 0., 0.), (7., 0., 0., 0.), (25., 0., 0., 0.)]
    path = sample_quintic_path(reference, 0., 3.5, 12.3, 0.5)
    x, y = np.asarray(path)[:, :2].T
    assert len(path) > len(reference)
    assert np.all(np.diff(x) > 0)
    assert max(np.diff(x)) <= 0.5+1e-9
    assert any(abs(value-7.) < 1e-9 for value in x)
    endpoint = next(point for point in path if abs(point[0]-12.3) < 1e-9)
    assert endpoint[1] == pytest.approx(3.5)
    assert y == pytest.approx(3.5*quintic_transition(x/12.3))
    assert path[-1][:2] == pytest.approx((25, 3.5))
    assert max(y) <= 3.5+1e-12
    assert all(math.isfinite(value) for point in path for value in point)
    assert any(abs(point[2]) > 0.01 for point in path)
    assert any(abs(point[3]) > 0.001 for point in path)


def test_short_horizon_does_not_force_transition_to_finish_early():
    path = sample_quintic_path([(0., 0., 0., 0.), (4., 0., 0., 0.)], 0, 3.5, 12)
    assert path[-1][1] == pytest.approx(3.5*quintic_transition(1/3))
    assert path[-1][1] < 3.5


def test_interpolated_corridor_limits_each_inserted_sample():
    path = sample_quintic_path([(0., 0., 0., 0.), (20., 0., 0., 0.)],
                              0, 3.5, 12, lateral_bounds=[(-1., 2.), (-1., 0.5)])
    assert all(-1 <= y <= 2-1.5*x/20+1e-9 for x, y, _, _ in path)
    assert path[-1][1] == pytest.approx(0.5)


def test_heading_interpolation_crosses_pi_without_flipping_normal():
    reference = [(0., 0., math.radians(179), 0.),
                 (-10., 0., math.radians(-179), 0.)]
    path = sample_quintic_path(reference, 1, 1, 12)
    assert all(y < -0.99 for _, y, _, _ in path)


def test_duplicates_empty_and_degenerate_reference_are_supported():
    assert sample_quintic_path([], 0, 1, 12) == []
    path = sample_quintic_path([(0., 0., 0., 0.), (0., 0., 0., 0.), (5., 0., 0., 0.)], 0, 1, 12)
    assert all(math.dist(a[:2], b[:2]) > 0 for a, b in zip(path[:-1], path[1:]))
    single = sample_quintic_path([(0., 0., 0., 0.)], 0, 1, 12)
    assert single == [(0., 0., 0., 0.)]


@pytest.mark.parametrize('resolution', [0., -1., float('nan'), float('inf')])
def test_invalid_sampling_resolution_is_rejected(resolution):
    with pytest.raises(ValueError):
        MotionPlanner([(0., 0., 0., 0.), (20., 0., 0., 0.)], sampling_resolution_m=resolution)
    with pytest.raises(ValueError):
        sample_quintic_path([(0., 0., 0., 0.), (20., 0., 0., 0.)], 0, 1, 12, resolution)


@pytest.mark.parametrize('routed', [False, True])
def test_planners_anchor_sparse_transition_at_projection(routed):
    reference = [(0., 0., 0., 0.), (100., 0., 0., 0.)]
    if routed:
        planner = RouteAwareMotionPlanner(reference, lane_width=3.5, num_lanes=3,
                                          reference_lane_index=1, target_lane=1)
    else:
        planner = MotionPlanner(reference, lane_width=3.5, num_lanes=3)
    obstacles = [] if routed else [(80., 0.3, 4.5, 2., 0., 0.)]
    path = planner._plan((50., 0.3), (50., 0.3), obstacles)
    assert path[0][:2] == pytest.approx((50., 0.3))
    assert path[-1][1] == pytest.approx(0. if routed else 3.5)
    assert len(path) >= 101
    assert any(abs(point[2]) > 0.001 for point in path)


def test_inserted_samples_participate_in_collision_checks():
    reference = [(0., 0., 0., 0.), (30., 0., 0., 0.)]
    planner = RouteAwareMotionPlanner(reference, lane_width=3.5, num_lanes=1,
                                      reference_lane_index=0, target_lane=0)
    obstacle = [(15., 0., 0.5, 0.5, 0., 0.)]
    assert planner._trajectory_is_safe(reference, obstacle)
    dense = planner._build_candidate(reference, [0, 1], 0, 0)
    assert not planner._trajectory_is_safe(dense, obstacle)
