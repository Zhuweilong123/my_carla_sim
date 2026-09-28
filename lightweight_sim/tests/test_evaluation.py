import csv
import gzip
import math

import pytest

from lightweight_sim.engine.analysis.evaluation import run_evaluation, stats, resample
from lightweight_sim.engine.analysis.tracking import TrackingMonitor, FigureEightOracle
from lightweight_sim.engine.algorithms.utils.route import RouteGeometry
from lightweight_sim.engine.simulator.data_types import VehicleState, PathPoint
from lightweight_sim.visualization.ros_gui import RosGuiView
from lightweight_sim.visualization._ros_gui_impl import GuiSnapshot, road_strip_polygons


def test_gui_and_monitor_measure_actual_state_with_same_units():
    path = [(0., 0., 0., 0.), (100., 0., 0., 0.)]
    state = VehicleState(x=20, y=0.5, phi=math.radians(5), vx=12, timestamp=2)
    measured = TrackingMonitor(path).update(state)
    assert measured["ed_m"] == pytest.approx(0.5)
    assert measured["ephi_deg"] == pytest.approx(5)
    snapshot = GuiSnapshot(state=state, reference_line_path=[PathPoint(*p) for p in path])
    ed, ephi = RosGuiView._tracking_error(snapshot)
    assert (ed, math.degrees(ephi)) == pytest.approx((0.5, 5))
    snapshot.tracking_metrics = measured
    assert RosGuiView._tracking_error(snapshot) == pytest.approx((0.5, math.radians(5)))


def test_independent_oracle_detects_wrong_crossing_branch():
    oracle = FigureEightOracle()
    for i in range(61):
        t = math.pi/2*i/60
        phase, heading, region = oracle.update(78*math.cos(t), 42*math.sin(2*t))
    assert phase == pytest.approx(math.pi/2, abs=1e-5)
    assert region == "crossing_1"
    assert not oracle.wrong_branch(heading, heading, region)
    wrong_branch_heading = math.atan2(-84, 78)
    assert oracle.wrong_branch(wrong_branch_heading, heading, region)


def test_density_experiment_keeps_vertices_and_length():
    path = [(0., 0., 0., 0.), (4., 0., 0., 0.), (4., 4., 1.57, 0.)]
    dense = resample(path, 0.5)
    assert all(p in dense for p in path)
    assert RouteGeometry(dense).length == pytest.approx(RouteGeometry(path).length)


def test_figure_eight_road_uses_local_non_self_intersecting_strips():
    path = []
    for index in range(241):
        t = 2.0 * math.pi * index / 240.0
        x = 78.0 * math.cos(t)
        y = 42.0 * math.sin(2.0 * t)
        theta = math.atan2(84.0 * math.cos(2.0 * t), -78.0 * math.sin(t))
        path.append(PathPoint(x=x, y=y, theta=theta, kappa=0.0))

    strips = list(road_strip_polygons(path, lane_width=3.5, num_lanes=3))

    assert len(strips) == 240
    assert all(len(strip) == 4 for strip in strips)
    assert all(
        math.isfinite(value)
        for strip in strips
        for point in strip
        for value in point
    )


def test_peak_statistics_include_time_and_empty_window_is_explicit():
    rows = [dict(error=-3, time_s=1, region="crossing_1"), dict(error=1, time_s=2, region="other")]
    result = stats(rows, "error")
    assert result["rms"] == pytest.approx(math.sqrt(5))
    assert result["peak_time_s"] == 1
    assert result["peak_region"] == "crossing_1"
    assert stats([], "error") is None


def test_evaluation_reproducible_and_archive_cannot_be_overwritten(tmp_path):
    settings = dict(laps=0, duration=1.0, seed=7, noise_m=0.02, delay_steps=1)
    a = run_evaluation(tmp_path, "a", **settings)
    b = run_evaluation(tmp_path, "b", **settings)
    assert a["all"]["lateral_error_m"] == b["all"]["lateral_error_m"]
    assert a["startup"]["samples"] == 20
    assert a["steady"]["lateral_error_m"] is None
    assert a["reference"]["sha256"] == b["reference"]["sha256"]
    with gzip.open(tmp_path/"figure_eight_a.csv.gz", "rt") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 20
    assert "controller_ed_pred_m" in rows[0]
    with pytest.raises(FileExistsError):
        run_evaluation(tmp_path, "a", **settings)
