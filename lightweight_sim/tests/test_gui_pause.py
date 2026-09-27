"""GUI pause controls share the same action for keyboard and mouse input."""

import pygame

from lightweight_sim.visualization.ros_gui import GuiSnapshot, RosGuiView


def test_e_key_and_toolbar_toggle_pause_once_per_press(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    view = RosGuiView(width=800, height=600, render_fps=1000)
    try:
        snapshot = GuiSnapshot()
        view.render(snapshot)
        button = view._mode_button_rects["TOGGLE_PAUSE"]

        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_e))
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_e))
        actions = view.poll_actions()
        assert sum(action.kind == "toggle_pause" for action in actions) == 1
        assert all(
            not (action.kind == "set_mode" and action.value == "EMERGENCY_STOP")
            for action in actions
        )

        pygame.event.post(pygame.event.Event(pygame.KEYUP, key=pygame.K_e))
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_e))
        actions = view.poll_actions()
        assert sum(action.kind == "toggle_pause" for action in actions) == 1

        snapshot.status.paused = True
        view.render(snapshot)
        pygame.event.post(pygame.event.Event(
            pygame.MOUSEBUTTONDOWN, button=1, pos=button.center,
        ))
        actions = view.poll_actions()
        assert sum(action.kind == "toggle_pause" for action in actions) == 1
    finally:
        view.close()
