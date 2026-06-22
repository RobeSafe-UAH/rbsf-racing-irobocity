# 01 - Autonomous Wall Following

Drive the car autonomously along a wall using a simple PI controller that reads
distance from the LiDAR and outputs Ackermann steering commands.
No localization or mapping - pure reactive control.
Supports simulation (`mvsim`), the physical car (`real`), and distributed mode (`distributed`)
where hardware runs on a separate embedded device.

---

## Quick start

```bash
# Physical car (default)
ros2 launch stack_launcher 01_wall_following.launch.py

# Simulation
ros2 launch stack_launcher 01_wall_following.launch.py mode:=mvsim

# Simulation with custom params file
ros2 launch stack_launcher 01_wall_following.launch.py mode:=mvsim \
  params_file:=/ros2_ws/src/controller/rbsf_wall_following/config/wall_following_sim.yaml

# Distributed (compute node only - hardware device runs 00_distributed_bringup)
ros2 launch stack_launcher 01_wall_following.launch.py mode:=distributed
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
See the **Config files** section below for the parameter names.

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

## Config files

Controller parameters live in `rbsf_wall_following/config/`:

| File | Used when |
|---|---|
| `wall_following_sim.yaml` | `mode:=mvsim` |
| `wall_following_real.yaml` | `mode:=real` |

Key parameters in these files:

| Parameter | Description |
|---|---|
| `desired_distance` | Target distance from the wall in metres |
| `speed` | Forward speed in m/s |
| `distance_kp` | Distance-error proportional gain |
| `distance_ki` | Distance-error integral gain |
| `distance_integral_limit` | Anti-windup clamp for the distance integrator |
| `orientation_kp` | Heading-error proportional gain |
| `orientation_ki` | Heading-error integral gain |
| `orientation_integral_limit` | Anti-windup clamp for the orientation integrator |
| `scan_topic` | LiDAR topic to subscribe to |
| `drive_topic` | Ackermann command topic to publish |

Edit these files to persist tuned gains, or point to a custom YAML with `params_file:=<path>`.

---

## Nodes launched

| Node | Package | Condition |
|---|---|---|
| `mvsim` | `mvsim` | `mode:=mvsim` |
| `bringup` (VESC, LiDAR, camera) | `rbsf_bringup` | `mode:=real` |
| `wall_following` (sub-launcher) | `rbsf_wall_following` | always |
| `ackermann_to_twist` | `stack_launcher` | `mode:=mvsim` |

> In `distributed` mode, hardware bringup is skipped - the embedded device runs
> `00_distributed_bringup.launch.py` instead.
