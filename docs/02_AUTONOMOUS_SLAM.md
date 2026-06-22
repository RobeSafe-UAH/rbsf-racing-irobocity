# 02 — Autonomous SLAM Mapping

Drive the car autonomously along a wall using a PI controller while SLAM Toolbox
builds a map of the environment in the background.
Supports simulation (`mvsim`) and the physical car (`real`).

---

## Quick start

```bash
# Simulation
ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=mvsim

# Physical car
ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=real
```

---

## Arguments

### Core

| Argument | Default | Choices | Description |
|---|---|---|---|
| `mode` | `real` | `mvsim`, `real`, `distributed` | Select the stack environment |
| `startup_delay` | `2.5` | Any non-negative number | Delay before starting the controller |

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

### Controller configuration

The controller is implemented in
`rbsf_wall_following/rbsf_wall_following/wall_follower_node.py`. Its topics,
references and PI gains are defined directly in the source code; it does not
load a parameter file or expose tuning arguments through the launcher.
The `/scan` subscription is remapped to `/laser1` only in MVSim mode.

### Map saving

| Argument | Default | Description |
|---|---|---|
| `map_output_dir` | `/ros2_ws/maps` | Directory where map files are written |
| `save_button` | `0` | Gamepad button index that triggers a save |

---

## How the PI controller works

```
distance_error = measured_distance - reference_distance
steering = distance_Kp·distance_error + distance_Ki·∫distance_error dt
         + orientation_Kp·orientation_error + orientation_Ki·∫orientation_error dt
```

The left-wall geometry is estimated from two LiDAR rays at 90 and 60 degrees.

### Tuning tips

| Symptom | Likely cause | Try |
|---|---|---|
| Slow oscillations | `Kp` too low | Increase `Kp` |
| Fast oscillations | `Kp` too high | Reduce `Kp` |
| Steady offset from wall | No integral term | Add a small `Ki` |
| Windup / drift | `Ki` too large | Reduce `Ki` |

---

## Saving a map

An xterm window opens automatically. **Click on it to give it focus**, then press `m`
(keystrokes are hidden) to save the map.  Alternatively, press button `save_button`
(default: **A / button 0**) on a connected gamepad.

The map is saved to `map_output_dir` with a timestamp: `map_YYYYMMDD_HHMMSS/`.

---

## Config files

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
| `wall_follower` | `rbsf_wall_following` | always |
| `async_slam_toolbox_node` | `slam_toolbox` | always |
| `map_saver_node` | `rbsf_slam_toolbox` | always |
| `keyboard_map_trigger` | `stack_launcher` | always |
| `ackermann_to_twist` | `stack_launcher` | `mode:=mvsim` |
