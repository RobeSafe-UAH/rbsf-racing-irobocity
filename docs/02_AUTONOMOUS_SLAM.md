# 02 — Autonomous SLAM Mapping

Drive the car autonomously along a wall using a PID controller while SLAM Toolbox
builds a map of the environment in the background.
Supports simulation (`mvsim`) and the physical car (`real`).

---

## Quick start

```bash
# Simulation — right wall (default)
ros2 launch stack_launcher 02_autonomous_slam.launch.py

# Simulation — left wall
ros2 launch stack_launcher 02_autonomous_slam.launch.py side:=left

# Physical car — right wall
ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=real

# Physical car — left wall, slower
ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=real side:=left speed:=0.4
```

---

## Arguments

### Core

| Argument | Default | Choices | Description |
|---|---|---|---|
| `mode` | `mvsim` | `mvsim`, `real` | Run in simulation or on the physical car |
| `side` | `right` | `left`, `right` | Wall side to follow |

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

### PID controller

These override the values in `pid_params_sim.yaml` / `pid_params_real.yaml`.
Pass `auto` (the default) to use the value from the YAML file unchanged.

| Argument | Sim default | Real default | Description |
|---|---|---|---|
| `desired_distance` | `0.5` | `0.7` | Target wall distance in metres |
| `speed` | `3.0` | `0.6` | Forward speed in m/s |
| `kp` | `0.9` | `1.0` | Proportional gain |
| `ki` | `0.02` | `0.0` | Integral gain |
| `kd` | `0.08` | `0.08` | Derivative gain |
| `params_file` | `auto` | `auto` | Full path to a custom params YAML |

### Map saving

| Argument | Default | Description |
|---|---|---|
| `map_output_dir` | `/ros2_ws/maps` | Directory where map files are written |
| `save_button` | `0` | Gamepad button index that triggers a save |

---

## How the PID controller works

```
error  =  desired_distance − measured_distance
steering  =  Kp·error + Ki·∫error dt + Kd·(Δerror/Δt)
```

The wall distance is measured by averaging LiDAR returns in a narrow angular sector
centred at ±90° (right/left).  The steering sign is flipped automatically when
`side:=left` so the car turns toward the wall when too far, away when too close.

### Tuning tips

| Symptom | Likely cause | Try |
|---|---|---|
| Slow oscillations | `Kp` too low | Increase `Kp` |
| Fast oscillations | `Kp` too high | Reduce `Kp`, increase `Kd` |
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

Default PID parameters live in `rbsf_wall_follower/config/`:

| File | Used when |
|---|---|
| `pid_params_sim.yaml` | `mode:=mvsim` |
| `pid_params_real.yaml` | `mode:=real` |

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
| `pid_controller` | `rbsf_wall_follower` | always |
| `async_slam_toolbox_node` | `slam_toolbox` | always |
| `map_saver_node` | `rbsf_slam_toolbox` | always |
| `keyboard_map_trigger` | `stack_launcher` | always |
| `ackermann_to_twist` | `stack_launcher` | `mode:=mvsim` |
