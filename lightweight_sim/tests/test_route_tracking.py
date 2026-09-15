import math

from lightweight_sim.engine.algorithms.controller.lat_lqr import LateralLQRController


VEHICLE_PARAMS = (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0)


def test_projection_does_not_jump_to_later_crossing_branch():
    # The first horizontal branch and a later diagonal branch share (0, 0).
    # Progress is already on the horizontal branch, so the later branch must
    # not become eligible just because it is geometrically close.
    ref_path = [
        (-4.0, 0.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0, 0.0),
        (4.0, 4.0, math.pi / 2.0, 0.0),
        (0.0, 0.0, -3.0 * math.pi / 4.0, 0.0),
        (-4.0, -4.0, -3.0 * math.pi / 4.0, 0.0),
    ]
    controller = LateralLQRController(VEHICLE_PARAMS)
    controller.route_progress = 1.0
    controller.min_index = 1
    controller.last_ref_heading = 0.0
    controller.search_back_segments = 1
    controller.search_forward_segments = 2

    projection, closed, _ = controller._project_reference(0.0, 0.0, 0.0, ref_path)

    assert not closed
    assert projection is not None
    assert projection[2] < 4.0
    assert projection[5] == 0.0


def test_reset_tracking_clears_route_identity():
    controller = LateralLQRController(VEHICLE_PARAMS)
    controller.route_progress = 12.5
    controller.min_index = 12
    controller.last_ref_heading = 1.2

    controller.reset_tracking()

    assert controller.route_progress == 0.0
    assert controller.min_index == 0
    assert controller.last_ref_heading is None
