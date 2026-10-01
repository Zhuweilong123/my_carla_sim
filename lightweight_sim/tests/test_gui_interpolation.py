"""GUI poses are smooth in wall time without changing simulation samples."""

import math

import pygame
import pytest

from lightweight_sim.engine.simulator.data_types import VehicleState
from lightweight_sim.visualization.state_interpolation import VehicleStateInterpolator
from lightweight_sim.visualization.ros_gui import GuiSnapshot, RosGuiView


@pytest.mark.parametrize("fps", [30, 60, 120])
def test_linear_motion_at_each_render_rate_and_no_extrapolation(fps):
    poses = VehicleStateInterpolator()
    start = VehicleState(timestamp=1, x=0, y=2)
    end = VehicleState(timestamp=1.05, x=0.5, y=3)
    poses.push(start, received_at=10)
    poses.push(end, received_at=10.05)
    for frame in range(round(fps * 0.05) + 1):
        elapsed = frame / fps
        shown = poses.sample(now=10.05 + elapsed)
        alpha = min(1.0, elapsed / 0.05)
        assert shown.x == pytest.approx(0.5 * alpha)
        assert shown.y == pytest.approx(2 + alpha)
    assert poses.sample(now=20).x == end.x
    assert (start.x, start.y, end.x, end.y) == (0, 2, 0.5, 3)


def test_interpolation_uses_wall_time_when_simulation_runs_slowly():
    poses = VehicleStateInterpolator()
    poses.push(VehicleState(timestamp=1, x=0), received_at=10)
    poses.push(VehicleState(timestamp=1.05, x=1), received_at=10.10)
    assert poses.sample(now=10.15).x == pytest.approx(0.5)
    assert poses.sample(now=10.20).x == pytest.approx(1)


def test_heading_crosses_pi_by_the_shortest_rotation():
    poses = VehicleStateInterpolator()
    poses.push(VehicleState(timestamp=1, phi=math.radians(179)), received_at=10)
    poses.push(VehicleState(timestamp=1.05, phi=math.radians(-179)), received_at=10.05)
    midpoint = poses.sample(now=10.075)
    assert abs(midpoint.phi) == pytest.approx(math.pi)


def test_duplicate_samples_do_not_restart_playback_and_pause_snaps():
    poses = VehicleStateInterpolator()
    poses.push(VehicleState(timestamp=1, x=0), received_at=10)
    end = VehicleState(timestamp=1.05, x=1)
    poses.push(end, received_at=10.05)
    poses.push(end, received_at=10.07)
    assert poses.sample(now=10.075).x == pytest.approx(0.5)
    assert poses.sample(now=10.075, paused=True).x == 1
    assert poses.sample(now=10.08).x == 1


@pytest.mark.parametrize("stamp,arrival", [(0, 10.1), (1.05, 10.1), (1.10, 11)])
def test_time_reset_teleport_and_long_transport_gap_snap(stamp, arrival):
    poses = VehicleStateInterpolator()
    poses.push(VehicleState(timestamp=1, x=0), received_at=10)
    poses.push(VehicleState(timestamp=1.05, x=1), received_at=10.05)
    poses.push(VehicleState(timestamp=stamp, x=100), received_at=arrival)
    assert poses.sample(now=arrival).x == 100
    poses.reset()
    assert poses.sample(now=arrival) is None


def test_camera_and_vehicle_share_display_pose_but_hud_uses_actual_state(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    view = RosGuiView(width=800, height=600, render_fps=1000)
    try:
        actual = VehicleState(timestamp=1.05, x=10, y=20)
        display = VehicleState(timestamp=1.025, x=9, y=19)
        snapshot = GuiSnapshot(state=actual)
        monkeypatch.setattr(view._display_states, "sample", lambda **kw: display)
        drawn = []
        telemetry = []
        monkeypatch.setattr(view.renderer, "draw_vehicle", lambda state: drawn.append(state))
        monkeypatch.setattr(view.hud, "render", lambda **kw: telemetry.append(kw["state"]))
        view.render(snapshot)
        assert (view.camera.cx, view.camera.cy) == (display.x, display.y)
        assert drawn == [display]
        assert telemetry == [actual]
        assert snapshot.state is actual
        view.reset_display_state()
        assert view._camera_scenario is None
    finally:
        view.close()


def test_camera_follow_smoothing_depends_on_elapsed_time_not_frame_count(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    view = RosGuiView(width=800, height=600, render_fps=1000)
    try:
        snapshot = GuiSnapshot(state=VehicleState(x=10))
        view._camera_scenario = snapshot.status.scenario
        now = [10.1]
        monkeypatch.setattr("lightweight_sim.visualization._ros_gui_impl.time.monotonic", lambda: now[0])
        view._last_render_time = 10.0
        view.render(snapshot)
        one_frame = view.camera.cx
        view.camera.cx = 0
        view._last_render_time = 10.0
        now[0] = 10.05
        view.render(snapshot)
        now[0] = 10.1
        view.render(snapshot)
        assert view.camera.cx == pytest.approx(one_frame)
    finally:
        view.close()


@pytest.mark.parametrize("callback_cost,expected_calls", [(0.0, 16), (0.003, 2)])
def test_ros_callback_draining_respects_count_and_time_budget(monkeypatch, callback_cost, expected_calls):
    pytest.importorskip("rclpy")
    pytest.importorskip("lightweight_sim_msgs.msg")
    from lightweight_sim.engine.ros_nodes.gui_node import GuiNode
    from lightweight_sim.engine.ros_nodes import _gui_node_impl

    now = [0.0]
    calls = []
    node = object()

    def spin_once(target, timeout_sec):
        assert target is node
        assert timeout_sec == 0
        calls.append(target)
        now[0] += callback_cost

    monkeypatch.setattr(_gui_node_impl.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(_gui_node_impl.rclpy, "spin_once", spin_once)
    GuiNode._process_ros_events(node)
    assert len(calls) == expected_calls
