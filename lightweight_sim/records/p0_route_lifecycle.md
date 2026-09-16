# P0: ROS route lifecycle and scenario consistency

The simulator publishes a latched `sim/context` (`std_msgs/String`, JSON schema
1) with run_id, scenario/route_id, target speed, vehicle model, lane dimensions
and physics timestep. A fresh run_id is issued on reset and scene selection.
Reference Path.sequence uses `(run_id << 20)`; local plans add a monotonically
increasing version in the low 20 bits. All nodes must be rebuilt together.

Consumers pair context and reference regardless of delivery order, reject old
run/version results, and invalidate pending plans on reset. Controller target
speed and planner lane parameters follow the simulator context. GUI reads the
same values. Unchanged reference paths preserve controller state; changed local
paths transfer the physical reference anchor. An expired avoidance plan brakes
until a fresh plan arrives. Obstacle-free planning explicitly selects the full
global route, including its closed-loop seam.

The controller uses metric unwrapped route_s with a physically bounded search
window. route_progress remains a segment-unit compatibility diagnostic. Normal
updates are local; initialization/path replacement can acquire a new anchor.
Application update_ref_path calls retain explicit-reset default semantics.

Validation commands (WSL, from the package directory):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q
source /opt/ros/lyrical/setup.bash
source ../install/setup.bash
ROS_DOMAIN_ID=87 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tests/test_ros_lifecycle.py -q -s
```

The ROS acceptance test creates the real three nodes in one executor using DDS,
explicitly advances fixed-step physics, and checks ten complete figure-eight
laps, route continuity, no collision/offroad, reset, rejection of a prior-run
plan, and scene switching with synchronized parameters. This tests transport
and callbacks at accelerated simulation speed, not wall-clock scheduling load.

Verified 2026-09-16 with use_sim_time enabled on planner and controller:
7070 steps, 353.50 simulated seconds, 4936.850 m controller progress, 10 laps,
no collision or offroad. The DDS test also passed reset and scene switching.
The non-ROS suite passed 18 tests; the ROS acceptance adds one test.
