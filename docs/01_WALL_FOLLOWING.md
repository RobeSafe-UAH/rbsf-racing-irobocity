# 01 — Autonomous Wall Following

Drive the car autonomously along a wall using a simple PI controller that reads
distance from the LiDAR and outputs Ackermann steering commands.
No localization or mapping — pure reactive control.
Supports simulation (`mvsim`) and the physical car (`real`).

---

## Quick start

```bash
# Simulation
ros2 launch stack_launcher 01_wall_following.launch.py mode:=mvsim

# Physical car
ros2 launch stack_launcher 01_wall_following.launch.py mode:=real
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

The topics, references and PI gains are configured directly in Exercise 0 of
`rbsf_wall_following/rbsf_wall_following/wall_follower_node.py`. The
wall-following controller does not load a parameter file or expose tuning
arguments through the launcher.

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

## Nodes launched

| Node | Package | Condition |
|---|---|---|
| `mvsim` | `mvsim` | `mode:=mvsim` |
| `bringup` (VESC, LiDAR, camera) | `rbsf_bringup` | `mode:=real` |
| `wall_follower` | `rbsf_wall_following` | always |
| `ackermann_to_twist` | `stack_launcher` | `mode:=mvsim` |
