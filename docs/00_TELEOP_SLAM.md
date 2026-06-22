# 00 - Teleoperated SLAM Mapping

Drive the car manually while SLAM Toolbox builds a map of the environment.
Supports simulation (`mvsim`) and the physical car (`real`), with keyboard or gamepad control.
Keyboard teleoperation is only available in simulation (`mode:=mvsim`).

---

## Quick start

```bash
# Physical car + gamepad (default)
ros2 launch stack_launcher 00_teleop_slam.launch.py

# Physical car + gamepad (explicit)
ros2 launch stack_launcher 00_teleop_slam.launch.py mode:=real teleop:=controller

# Simulation + gamepad
ros2 launch stack_launcher 00_teleop_slam.launch.py mode:=mvsim

# Simulation + keyboard
ros2 launch stack_launcher 00_teleop_slam.launch.py mode:=mvsim teleop:=keyboard
```

---

## Arguments

### Core

| Argument | Default | Choices | Description |
|---|---|---|---|
| `mode` | `real` | `mvsim`, `real` | Run in simulation or on the physical car |
| `teleop` | `controller` | `keyboard`, `controller` | Input device (`keyboard` is `mode:=mvsim` only) |

### Simulation (`mode:=mvsim`)

| Argument | Default | Description |
|---|---|---|
| `world_file` | `f1tenth_catalunya.world.xml` | MVSim world file to load |
| `vehicle_config_file` | `mvsim_vehicle.yaml` | Vehicle model parameters |
| `world_config_file` | `mvsim_worlds.yaml` | World-specific parameters |
| `init_pose` | *(empty)* | Initial pose override: `'x y yaw_deg'` |

Available worlds in `rbsf_mvsim/maps/`:

- `f1tenth_catalunya.world.xml`
- `f1tenth_saopaulo.world.xml`
- `f1tenth_yasmarina.world.xml`
- `hallway_simulation_sci.world.xml`
- `simple_oval.world.xml`

### Keyboard (`mode:=mvsim teleop:=keyboard`)

| Argument | Default | Description |
|---|---|---|
| `max_speed` | `1.0` | Maximum speed in m/s for keyboard teleop |

### SLAM

| Argument | Default | Description |
|---|---|---|
| `slam_params_file` | `auto` | Path to slam_toolbox params YAML. `auto` selects `mapper_params_real.yaml` or `mapper_params_sim.yaml` by mode |

### Map saving

| Argument | Default | Description |
|---|---|---|
| `map_output_dir` | `/ros2_ws/maps` | Directory where map files are written |
| `save_button` | `0` | Gamepad button index that triggers a save (`teleop:=controller`) |

---

## Keyboard controls

When `mode:=mvsim teleop:=keyboard`, an xterm window opens automatically. **Click on it to give it focus**, then use:

| Key | Action |
|---|---|
| `W` / `Up` | Forward |
| `S` / `Down` | Backward |
| `A` / `Left` | Steer left |
| `D` / `Right` | Steer right |
| `q` | Quit keyboard node |

---

## Saving a map

### With gamepad (`teleop:=controller`)

Press the button at index `save_button` (default: **A / button 0**).
The map is saved to `map_output_dir` with a timestamp: `map_YYYYMMDD_HHMMSS/`.

### With keyboard (`teleop:=keyboard`)

Call the SLAM Toolbox service directly from another terminal:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/ros2_ws/maps/my_map'}"
```

---

## Nodes launched

| Node | Package | Condition |
|---|---|---|
| `mvsim` | `mvsim` | `mode:=mvsim` |
| `bringup` (VESC, LiDAR, camera) | `rbsf_bringup` | `mode:=real` |
| `ackermann_mux` | `ackermann_mux` | always |
| `async_slam_toolbox_node` | `slam_toolbox` | always |
| `map_saver_node` | `rbsf_slam_toolbox` | always |
| `ackermann_to_twist` | `stack_launcher` | `mode:=mvsim` |
| `joy` + `joy_teleop` | `joy`, `joy_teleop` | `teleop:=controller` |
| `keyboard_teleop` | `stack_launcher` | `mode:=mvsim` AND `teleop:=keyboard` |
