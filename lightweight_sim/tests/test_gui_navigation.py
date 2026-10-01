"""Exercise public GUI navigation through the pygame event queue."""

import pygame
import pytest

from lightweight_sim.engine.simulator.data_types import VehicleState
from lightweight_sim.visualization.ros_gui import GuiSnapshot, RosGuiView


@pytest.fixture
def view(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setattr(pygame.key, "get_mods", lambda: 0)
    gui = RosGuiView(width=800, height=600, render_fps=1000)
    pygame.event.clear()
    yield gui
    gui.close()


@pytest.mark.parametrize("flipped", [False, True])
def test_precise_touchpad_scroll_pans_both_axes_and_stays_put(view, flipped):
    snapshot = GuiSnapshot(state=VehicleState(x=10, y=20))
    view.render(snapshot)
    pygame.event.clear()
    old_scale = view.camera.scale
    before = view.camera.world_to_screen(10, 20)
    pygame.event.post(pygame.event.Event(
        pygame.MOUSEWHEEL, x=0, y=0, precise_x=0.75, precise_y=-0.375,
        flipped=flipped,
    ))
    view.poll_actions()
    after = view.camera.world_to_screen(10, 20)
    sign = -1 if flipped else 1
    assert after == (before[0] + sign * 24, before[1] - sign * 12)
    assert view.camera.scale == old_scale
    assert view.camera.follow_enabled is False
    center = (view.camera.cx, view.camera.cy)
    snapshot.state.x += 15
    snapshot.state.y -= 5
    view.render(snapshot)
    assert (view.camera.cx, view.camera.cy) == center


def test_ctrl_scroll_and_keyboard_zoom_do_not_pan(view, monkeypatch):
    monkeypatch.setattr(pygame.key, "get_mods", lambda: pygame.KMOD_CTRL)
    old_scale = view.camera.scale
    pygame.event.post(pygame.event.Event(
        pygame.MOUSEWHEEL, x=0, y=1, precise_x=0.0, precise_y=0.5,
    ))
    view.poll_actions()
    assert view.camera.scale == pytest.approx(old_scale * 1.05)
    assert (view.camera.cx, view.camera.cy) == (0, 0)
    assert view.camera.follow_enabled is True
    pygame.event.post(pygame.event.Event(pygame.MOUSEWHEEL, x=1, y=0))
    view.poll_actions()
    assert view.camera.scale == pytest.approx(old_scale * 1.05)
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_MINUS))
    view.poll_actions()
    assert view.camera.scale == pytest.approx(old_scale * 1.05 * 0.9)


def test_middle_drag_ends_on_release_or_focus_loss_and_preserves_clicks(view):
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=2, pos=(400, 300)))
    pygame.event.post(pygame.event.Event(
        pygame.MOUSEMOTION, rel=(60, -30), pos=(460, 270), buttons=(0, 1, 0),
    ))
    view.poll_actions()
    assert (view.camera.cx, view.camera.cy) == pytest.approx((-10, -5))
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, button=2, pos=(460, 270)))
    pygame.event.post(pygame.event.Event(
        pygame.MOUSEMOTION, rel=(60, 30), pos=(520, 300), buttons=(0, 0, 0),
    ))
    view.poll_actions()
    assert (view.camera.cx, view.camera.cy) == pytest.approx((-10, -5))
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=2, pos=(400, 300)))
    pygame.event.post(pygame.event.Event(pygame.WINDOWFOCUSLOST))
    pygame.event.post(pygame.event.Event(
        pygame.MOUSEMOTION, rel=(60, 30), pos=(460, 330), buttons=(0, 0, 0),
    ))
    view.poll_actions()
    assert (view.camera.cx, view.camera.cy) == pytest.approx((-10, -5))
    view.render(GuiSnapshot())
    button = view._mode_button_rects["TOGGLE_PAUSE"]
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=button.center))
    actions = view.poll_actions()
    assert sum(action.kind == "toggle_pause" for action in actions) == 1


def test_f_rejoins_vehicle_follow_and_new_scene_recenters(view):
    snapshot = GuiSnapshot(state=VehicleState(x=10, y=20))
    view.render(snapshot)
    view.camera.pan(60, 30)
    snapshot.state.x += 30
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_f))
    view.poll_actions()
    view.render(snapshot)
    assert view.camera.follow_enabled is True
    assert (view.camera.cx, view.camera.cy) == (snapshot.state.x, snapshot.state.y)
    snapshot.state.x += 10
    view.render(snapshot)
    assert view.camera.cx > 40
    view.camera.pan(60, 30)
    snapshot.status.scenario = "new_scene"
    view.render(snapshot)
    assert view.camera.follow_enabled is True
    assert (view.camera.cx, view.camera.cy) == (snapshot.state.x, snapshot.state.y)
