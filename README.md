# Vehicle Motion

[English](README.md) | [简体中文](README.zh-CN.md)

Vehicle Motion is a lightweight 2D vehicle simulation and ROS 2 planning/control workspace. It is intended for repeatable development of vehicle dynamics, lane-level routing, local obstacle avoidance, path tracking, and parking interfaces. The simulator is independent of CARLA; `carla_legacy/` contains archived CARLA code and is not part of the current ROS 2 runtime.

## Current capabilities

- Fixed-step simulation with configurable physics period, vehicle dimensions, speed limits, and actuator parameters; supports kinematic and simplified dynamic bicycle models.
- ROS 2 nodes for simulation, A* lane-topology routing, route-derived reference-line generation, local planning, cruise control, mode arbitration, and safe stopping when required routing/planning data is missing or stale.
- A 2D GUI client that subscribes to the ROS graph (it does not run a second simulator), visualizes the road, routing/reference paths, local planned path, vehicle state, and control status, and supports runtime scene/mode switching.
- An independent `parking_module` ROS 2 package with a reverse-parking baseline and controller adapter.
- Regression tests with split GitHub Actions: ROS-independent Python tests run on every push/PR; ROS build, integration tests and launch smoke run for relevant changes or manual dispatch.

The default simulation and control periods are 0.05 s (20 Hz). These settings and most vehicle, planner, routing, controller, GUI, and safety parameters are in `lightweight_sim/config/default.yaml`; shared timing defaults are defined in `lightweight_sim/engine/runtime_config.py`. Architecture and algorithm notes: [design overview](lightweight_sim/design/DESIGN.md), [algorithms](lightweight_sim/design/ALGORITHMS.md), [ROS 2](lightweight_sim/design/ROS2.md), [Routing](lightweight_sim/design/ROUTING.md), and [reference line](lightweight_sim/design/REFERENCE_LINE.md).

## Build and launch (ROS 2 / WSL2)

The maintained workflow uses ROS 2 Lyrical in WSL2. In a new terminal:

```bash
source /opt/ros/lyrical/setup.bash
cd /mnt/d/AI_tools/vehicle_motion
python3 -m pip install -r lightweight_sim/requirements.txt
colcon build
source install/setup.bash
ros2 launch lightweight_sim lightweight_sim.launch.py gui:=true
```

On a Windows-mounted `/mnt/d` workspace, use plain `colcon build` rather than `--symlink-install`. For a headless run, use `gui:=false`. Choose the initial scene and steering model with launch arguments:

```bash
ros2 launch lightweight_sim lightweight_sim.launch.py \
  scenario:=figure_eight gui:=true steering_profile:=ideal
```

`steering_profile:=assumed` enables the simulated delayed/rate-limited steering actuator. Its parameters are engineering assumptions, not calibration data for a real vehicle.

## Built-in scenes

The GUI scene shortcuts are `1`–`7`:

| Key | Scene | Description |
| --- | --- | --- |
| 1 | `default` | Two-lane straight-road cruise. |
| 2 | `obstacle` | Straight road with a static obstacle. |
| 3 | `three_lane` | Three-lane road with sequential obstacles. |
| 4 | `curve` | Road with a 90-degree curve. |
| 5 | `figure_eight` | Closed three-lane figure-eight route. |
| 6 | `reverse_parking` | Perpendicular reverse-parking scene; use the parking module for autonomous parking. |
| 7 | `demo_grid` | Synthetic two-way city grid with two lanes per direction and multiple intersections. |

Routing is part of the standard simulator launch. The bundled JSON maps are in `lightweight_sim/config/maps/`; the routing node loads the map set by default and each routed scene requests its matching map/lanes. `map_dir` and `map_file` can be set on `routing_node` to use custom maps. See [Routing](lightweight_sim/design/ROUTING.md) and [Reference Line](lightweight_sim/design/REFERENCE_LINE.md) for map format, A* behavior, and reference-line validation/smoothing.

## GUI controls

- Click `CRUISE`, `PARKING`, or `E-STOP` to select the task; the GUI control-source button or `Q` switches `AUTO`/`MANUAL`.
- `C`, `K`, and `E` select cruise, parking, and emergency stop. `1`–`7` switch scenes.
- In manual mode, `W/S` or the up/down arrows drive, `A/D` or the left/right arrows steer, and `SPACE` brakes.
- `P` pauses/resumes, `R` resets, the mouse wheel or `+`/`-` zooms, and `ESC` closes the GUI.

GUI rendering requires a Linux display (for WSL2, WSLg). Headless simulation, ROS topics, services, and tests do not require the GUI.

## Unified cruise and parking interface

To start the GUI with the independent parking controller and shared control-mode interface:

```bash
ros2 launch parking_module unified_vehicle.launch.py gui:=true
```

The parking package can also be launched directly in parking mode:

```bash
ros2 launch parking_module unified_vehicle.launch.py \
  scenario:=reverse_parking mode:=PARKING gui:=false
```

The cruise and parking controllers publish candidate commands; `controller_manager` selects the active source/mode and is the final command arbiter. The parking planner/controller is a baseline for the reverse-parking maneuver, not a production-grade planner or vehicle safety system. More details are in [`parking_module/README.md`](parking_module/README.md).

## ROS interfaces and diagnostics

Useful topics include:

| Topic | Purpose |
| --- | --- |
| `/vehicle/state`, `/obstacles` | Current ego state and detected/simulated obstacles. |
| `/routing/request`, `/routing/route` | Mission request and A* topology route. |
| `/routing/reference_line` | Smoothed, map-validated routed-lane reference. |
| `/planned_path` | Current local planner trajectory. |
| `/control_command` | Final arbitrated command to the simulator. |
| `/sim/status`, `/tracking/metrics` | Lifecycle/safety outcome and tracking diagnostics. |

Useful checks and controls:

```bash
ros2 topic list
ros2 topic echo /vehicle/state
ros2 topic echo /routing/route
ros2 topic echo /routing/reference_line
ros2 topic echo /planned_path
ros2 topic echo /sim/status --once
ros2 service call /sim/reset std_srvs/srv/Empty '{}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/step std_srvs/srv/Trigger '{}'
```

`/control_command` is the final actuator-facing command. Cruise, manual, and parking candidates use separate topics and are arbitrated by `controller_manager`. The `safe_stop_node` requests braking if the active run lacks a matching routing/reference/local plan or the planning data becomes stale. See [ROS 2 architecture](lightweight_sim/design/ROS2.md) for node graph, QoS, services, and launch options.

## Tests and CI

Build and run the simulator test suite from the repository root with the ROS environment sourced:

```bash
source /opt/ros/lyrical/setup.bash
cd /mnt/d/AI_tools/vehicle_motion
colcon build --packages-up-to lightweight_sim
source install/setup.bash
python3 -m pytest
```

The ROS launch smoke test can be run with:

```bash
LIGHTWEIGHT_SIM_INSTALL="$PWD/install" \
  bash lightweight_sim/scripts/ros2_smoke_test.sh
```

Run the independent parking-module tests with `python3 -m pytest parking_module/tests`. GitHub Actions runs the Python-only tests on every push and pull request without installing ROS. A separate ROS 2 integration workflow builds the ROS packages and runs ROS-dependent tests plus the launch smoke test when ROS interfaces, launch/configuration, or related packages change; it can also be started manually. Experimental process records under `lightweight_sim/records/` are kept locally and ignored by Git.

## Repository layout

```text
lightweight_sim/       Simulator, ROS 2 nodes, planners/controllers, maps, GUI and tests
lightweight_sim_msgs/  ROS 2 message and service definitions
parking_module/        Independent reverse-parking planner/controller ROS 2 package
carla_legacy/          Archived CARLA implementation (not required by lightweight_sim)
```
