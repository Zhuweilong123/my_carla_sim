import math

from parking_module.core.types import BoxObstacle, ParkingSlot, Pose2D
from parking_module.planning import HybridAStarPlanner


def test_hybrid_astar_plans_from_arbitrary_pose_with_obstacles():
    slot = ParkingSlot(46.0, 7.5, -math.pi / 2.0)
    obstacles = (
        BoxObstacle(43.75, 7.5, 6.0, 0.25, -math.pi / 2.0),
        BoxObstacle(48.25, 7.5, 6.0, 0.25, -math.pi / 2.0),
        BoxObstacle(46.0, 10.5, 4.5, 0.25, 0.0),
    )
    trajectory = HybridAStarPlanner().plan(
        Pose2D(30.0, 0.0, math.pi / 2.0), slot, obstacles
    )

    assert trajectory.planner_name == "hybrid_astar_reverse_parking"
    assert trajectory.points[0].pose == Pose2D(30.0, 0.0, math.pi / 2.0)
    assert trajectory.points[-1].pose == slot.goal
    assert any(point.gear < 0 for point in trajectory.points)
