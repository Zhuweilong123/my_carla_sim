# Routing module

The routing package is independent from the obstacle-aware local planner.
`lightweight_sim.engine.routing` contains the map model, JSON loader, lane-edge
validation, A* search, and reference-path construction.  The ROS adapter is
`routing_node`.

## Map model

Maps contain `nodes` and directed lane `edges`.  Each edge has a centerline and
an explicit `successors` list.  A lane-change or junction connector is just
another edge, so the A* implementation does not need special cases for the
topology.

The repository includes `config/maps/` with the synthetic `demo_grid`,
`straight_obstacle`, `three_lane_double_obs`, and `curve_90deg` maps.  The
node loads every JSON map in that directory by default and selects one by
`RouteRequest.map_id`.

## ROS interface

The node exposes:

- service `routing/compute_route` (`lightweight_sim_msgs/srv/ComputeRoute`);
- topic `routing/request` (`lightweight_sim_msgs/msg/RouteRequest`);
- topic `routing/route` (`lightweight_sim_msgs/msg/RoutePlan`).

The simulator publishes a latched `routing/request` for scenarios that declare
`routing_map_id`, a destination, and start/goal lane indices.  This makes a
scenario reset or switch create a new routing request with its `run_id`; local
planning can consume the corresponding `RoutePlan` without reconstructing the
mission from ad-hoc context fields.

`RoutePlan` contains the ordered lane edges, maneuver labels, target lane,
total length, and a continuous `(x, y, theta, kappa)` reference path.  Dynamic
obstacles are deliberately not part of Routing; they remain inputs to the
behavior/local planner.

The node is started by the standard launch file.  Its map directory or a
single map can be overridden by `map_dir` or `map_file`:

```bash
ros2 run lightweight_sim routing_node \
  --ros-args -p map_dir:=/path/to/maps

ros2 run lightweight_sim routing_node \
  --ros-args -p map_file:=/path/to/map.json
```

The existing `Path` message remains the local controller trajectory format.
Routing metadata uses the new `RouteRequest`, `RouteSegment`, and `RoutePlan`
messages so road and lane identity are not lost.
