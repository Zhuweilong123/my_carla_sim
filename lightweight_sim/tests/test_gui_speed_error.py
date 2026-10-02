"""Speed errors use executed references and simulation-time history."""

import pytest

from lightweight_sim.engine.simulator.data_types import VehicleState
from lightweight_sim.visualization.ros_gui import GuiSnapshot, RosGuiView


@pytest.fixture
def view(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    gui = RosGuiView(width=800, height=600, render_fps=1000)
    yield gui
    gui.close()


def test_speed_error_freshness_and_control_source(view):
    snapshot = GuiSnapshot(state=VehicleState(timestamp=1))
    snapshot.tracking_metrics = {"timestamp": 1, "speed_error_kmh": -2.4}
    assert view._speed_error(snapshot) == -2.4
    snapshot.state.timestamp = 1.3
    assert view._speed_error(snapshot) is None
    snapshot.state.timestamp = 1
    snapshot.control_source = "MANUAL"
    assert view._speed_error(snapshot) is None
    snapshot.control_source = "AUTO"
    snapshot.status.scenario = "reverse_parking"
    assert view._speed_error(snapshot) is None


def test_history_shared_time_axis_pause_and_reset(view):
    snapshot = GuiSnapshot(state=VehicleState(timestamp=1))
    snapshot.tracking_metrics = {"timestamp": 1, "speed_error_kmh": 3.6,
                                 "ed_m": 0.0, "reference_heading_rad": 0.0}
    view.render(snapshot)
    view.render(snapshot)
    assert view.hud.history_times == [1]
    assert view.hud.speed_history == [3.6]
    snapshot.status.paused = True
    snapshot.state.timestamp = 2
    view.render(snapshot)
    assert view.hud.history_times == [1]
    snapshot.status.paused = False
    snapshot.state.timestamp = 22
    view.render(snapshot)
    assert view.hud.history_times == [22]
    assert view.hud.speed_history == [None]
    snapshot.state.timestamp = 0
    view.render(snapshot)
    assert view.hud.history_times == [0]
    assert len(view.hud.ed_history) == len(view.hud.ephi_history) == len(view.hud.speed_history)
    view.hud.clear_history()
    assert not view.hud.history_times and not view.hud.speed_history


def test_three_error_rows_have_units_and_missing_speed_label(view, monkeypatch):
    labels = []
    monkeypatch.setattr(view.hud, "_text", lambda text, *args: labels.append(text))
    view.hud._draw_error_graph(560, 330, 220, 240, .1, .02, -1.2)
    assert any(label.startswith("LATERAL") and "m" in label for label in labels)
    assert any(label.startswith("HEADING") for label in labels)
    assert "SPEED  ev -1.20 km/h" in labels
    labels.clear()
    view.hud._draw_error_graph(560, 330, 220, 240, .1, .02)
    assert "SPEED  ev -- km/h" in labels
