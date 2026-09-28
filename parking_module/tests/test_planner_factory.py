from parking_module.planning import (
    HybridAStarPlanner,
    ReverseParkingPlanner,
    create_planner,
)


def test_factory_keeps_baseline_and_hybrid_planners_selectable():
    assert isinstance(create_planner("baseline"), ReverseParkingPlanner)
    assert isinstance(create_planner("dubins"), ReverseParkingPlanner)
    assert isinstance(create_planner("hybrid-a-star"), HybridAStarPlanner)
