# Standalone parking module

`parking_module` is an independent planning and control package for the
reverse-parking experiment. Its core code imports only Python's standard
library and its own data contracts; it does not import `lightweight_sim`.
ROS 2 and `lightweight_sim_msgs` are used only by the optional
`parking_controller_node` adapter.

The first implementation is a deterministic reverse-parking baseline:

- a straight approach segment;
- a collision-checked Hermite reverse maneuver;
- a gear-aware pure-pursuit controller with safe stop on gear changes.

This provides a stable module boundary for later replacing the planner with
Hybrid A* or adding a model-predictive controller without changing simulator
code.

## Build

From the repository root in WSL2:

```bash
source /opt/ros/lyrical/setup.bash
colcon build
source install/setup.bash
```

Use plain `colcon build` on `/mnt/d` rather than `--symlink-install`; the
Windows-mounted filesystem can make colcon's symlink cleanup fail.

## Run with the simulator

For the unified GUI workflow, start all controllers and the single command
arbiter with one launch command:

```bash
ros2 launch parking_module unified_vehicle.launch.py gui:=true
```

The GUI provides `CRUISE`, `PARKING`, `E-STOP` and `AUTO`/`MANUAL` controls.
`PARKING` selects the `reverse_parking` scene and routes only the parking
candidate to the simulator; `CRUISE` routes the built-in cruise controller.
Press `Q` to toggle the control source. In manual mode, `W/S` drive,
`A/D` steer, and `SPACE` brakes. The final actuator topic is written only by
`controller_manager`.

To start directly in parking mode:

```bash
ros2 launch parking_module unified_vehicle.launch.py \
  scenario:=reverse_parking mode:=PARKING gui:=false
```

The standalone workflow below is still supported for testing the adapter in
isolation.

Start the simulator with its built-in controller disabled:

```bash
ros2 launch lightweight_sim lightweight_sim.launch.py \
  scenario:=reverse_parking controller_enabled:=false gui:=true
```

In a second terminal, after sourcing the same workspaces, start the standalone
controller:

```bash
ros2 run parking_module parking_controller_node
```

The node consumes `vehicle/state`, `obstacles`, and the latched `sim/context`,
then publishes `control_command`. It activates only when the context declares
`maneuver: reverse_parking`.

## Run the independent tests

```bash
python -m pytest parking_module/tests
```
