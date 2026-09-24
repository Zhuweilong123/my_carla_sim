# Reference-line module

`reference_line_node` converts a Routing topology result into the continuous
lane reference consumed by the local planner. It is deliberately separate from
both A* search and obstacle avoidance.

## Data flow

```text
routing/route (RoutePlan)
        |
        v
reference_line_node
        |
        v
routing/reference_line (ReferenceLine)
        |
        v
planner_node -> planned_path -> controller_node
```

For every successful `RoutePlan`, the node reloads the ordered edge geometry
from the selected map, validates the edge sequence, removes repeated joins,
resamples at one metre by default, and applies a bounded local smoothing pass.
The smoothing pass keeps both endpoints fixed, caps displacement with
`max_lateral_deviation_m` (0.15 m by default), and clamps every intermediate
sample to the map's left/right lane boundary with `boundary_margin_m` (0.10 m
by default).

Maps may provide per-edge `left_boundary` and `right_boundary` polylines. If
they are omitted, the loader derives them from the edge centerline and the
top-level `lane_width` (3.5 m by default). `num_lanes` is also map metadata.
The resulting `ReferenceLine` publishes the selected lane's sampled boundaries
plus the outer boundaries of all parallel lanes on the routed road segment.
The latter are the `drivable_*_boundary` fields used by `planner_node` to
reject and clamp local trajectories outside the map corridor.

`ReferenceLine` retains `request_id`, map ID, route ID, source segments,
reference-lane index, target lane, spacing, and the generated path geometry.
Consumers must pair it with the current `sim/context.run_id` through
`request_id`.

## Planner behavior

`planner_node` uses `routing/reference_line` when it matches the active run.
Its local candidates are expressed relative to the routed lane, so an obstacle
can cause a temporary lane change while the target lane remains the preferred
post-obstacle destination. Candidates and the resulting transition are bounded
by the ReferenceLine drivable corridor, with `routing_corridor_margin_m`
(1.1 m by default) reserved from each outer edge. If no matching Routing
reference is available, it keeps the existing simulator reference path as a
fallback.

## Parameters

```bash
ros2 run lightweight_sim reference_line_node \
  --ros-args \
  -p map_dir:=/path/to/maps \
  -p sample_spacing_m:=0.5 \
  -p max_lateral_deviation_m:=0.10 \
  -p boundary_margin_m:=0.15
```

The standard launch file starts the node by default. Set
`reference_line_enabled:=false` to disable it, or set Planner parameter
`use_routing_reference:=false` to keep local planning on the simulator
reference path while continuing to publish reference lines for inspection.
