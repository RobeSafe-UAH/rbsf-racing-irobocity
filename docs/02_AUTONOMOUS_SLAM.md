# 02 - Autonomous SLAM Mapping

Drive the car autonomously along a wall using a PI controller while SLAM Toolbox
builds a map of the environment in the background.
Supports simulation (`mvsim`), the physical car (`real`), and distributed mode (`distributed`)
where hardware runs on a separate embedded device.

---

## Quick start

```bash
# Physical car (default)
ros2 launch stack_launcher 02_autonomous_slam.launch.py

# Simulation
ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=mvsim

# Simulation with custom params file
ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=mvsim \
  params_file:=/ros2_ws/src/controller/rbsf_wall_following/config/wall_following_sim.yaml

# Distributed (compute node only - hardware device runs 00_distributed_bringup)
ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=distributed
```

---

## Arguments

### Core

| Argument | Default | Choices | Description |
|---|---|---|---|
| `mode` | `real` | `mvsim`, `real`, `distributed` | Run in simulation, on the physical car, or as compute node only |
| `startup_delay` | `2.5` | - | Seconds to wait before starting the wall-follower node |

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

### Controller

| Argument | Default | Description |
|---|---|---|
| `params_file` | `auto` | Path to a custom params YAML. `auto` selects `wall_following_sim.yaml` or `wall_following_real.yaml` by mode |

Controller gains and target distance are set in the params YAML - they are not individual launch arguments.
See `rbsf_wall_following/config/` for parameter names (`desired_distance`, `speed`, `distance_kp`, `distance_ki`, etc.).

### SLAM

| Argument | Default | Description |
|---|---|---|
| `slam_params_file` | `auto` | Path to slam_toolbox params YAML. `auto` selects `mapper_params_real.yaml` or `mapper_params_sim.yaml` by mode |

### Map saving

| Argument | Default | Description |
|---|---|---|
| `map_output_dir` | `/ros2_ws/maps` | Directory where map files are written |
| `save_button` | `0` | Gamepad button index that triggers a save |

---

## How the PI controller works

```
error     =  desired_distance - measured_distance
steering  =  distance_kp * error + distance_ki * integral(error) dt
```

The wall distance is measured from LiDAR returns. A separate orientation PI loop
(`orientation_kp`, `orientation_ki`) corrects the heading angle relative to the wall.

### Tuning tips

| Symptom | Likely cause | Try |
|---|---|---|
| Slow oscillations | `distance_kp` too low | Increase `distance_kp` |
| Fast oscillations | `distance_kp` too high | Reduce `distance_kp` |
| Steady offset from wall | No integral term | Add a small `distance_ki` |
| Windup / drift | `distance_ki` too large | Reduce `distance_ki` or lower `distance_integral_limit` |

---

## Saving a map

### In simulation (`mode:=mvsim`)

An xterm window opens automatically. **Click on it to give it focus**, then press `m`
(keystrokes are hidden) to save the map. Alternatively, press button `save_button`
(default: **A / button 0**) on a connected gamepad.

### On real car or distributed (`mode:=real` / `mode:=distributed`)

Press button `save_button` (default: **A / button 0**) on the connected gamepad.

The map is saved to `map_output_dir` with a timestamp: `map_YYYYMMDD_HHMMSS/`.

---

## Config files

Controller parameters live in `rbsf_wall_following/config/`:

| File | Used when |
|---|---|
| `wall_following_sim.yaml` | `mode:=mvsim` |
| `wall_following_real.yaml` | `mode:=real` |

SLAM Toolbox parameters live in `rbsf_slam_toolbox/config/`:

| File | Used when |
|---|---|
| `mapper_params_sim.yaml` | `mode:=mvsim` |
| `mapper_params_real.yaml` | `mode:=real` |

---

## Nodes launched

| Node | Package | Condition |
|---|---|---|
| `mvsim` | `mvsim` | `mode:=mvsim` |
| `bringup` (VESC, LiDAR, camera) | `rbsf_bringup` | `mode:=real` |
| `wall_following` (sub-launcher) | `rbsf_wall_following` | always |
| `async_slam_toolbox_node` | `slam_toolbox` | always |
| `map_saver_node` | `rbsf_slam_toolbox` | always |
| `keyboard_map_trigger` | `stack_launcher` | `mode:=mvsim` |
| `ackermann_to_twist` | `stack_launcher` | `mode:=mvsim` |

> In `distributed` mode, hardware bringup is skipped - the embedded device runs
> `00_distributed_bringup.launch.py` instead.
