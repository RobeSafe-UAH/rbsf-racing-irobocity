# 03 - Centerline Tracking

Drive the car autonomously along a pre-computed centerline using the
`rbsf_centerline_tracking` controller.
Supports simulation (`mvsim`) and the physical car (`real`).

---

## Quick start

```bash
# Physical car (default)
ros2 launch stack_launcher 03_centerline_tracking.launch.py

# Physical car with a centerline CSV
ros2 launch stack_launcher 03_centerline_tracking.launch.py \
  centerline_csv:=/ros2_ws/maps/my_track/centerline.csv

# Simulation
ros2 launch stack_launcher 03_centerline_tracking.launch.py mode:=mvsim

# Simulation with a specific world and centerline
ros2 launch stack_launcher 03_centerline_tracking.launch.py \
  mode:=mvsim \
  world_file:=f1tenth_saopaulo.world.xml \
  centerline_csv:=/ros2_ws/maps/saopaulo/centerline.csv
```

---

## Arguments

### Core

| Argument | Default | Choices | Description |
|---|---|---|---|
| `mode` | `real` | `mvsim`, `real` | Run in simulation or on the physical car |

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

### Centerline

| Argument | Default | Description |
|---|---|---|
| `centerline_csv` | *(empty)* | Path to a CSV file to publish as `nav_msgs/Path`. If empty, no centerline is published |
| `centerline_topic` | `/centerline` | Topic on which the centerline path is published |

---

## Nodes launched

| Node | Package | Condition |
|---|---|---|
| `mvsim` | `mvsim` | `mode:=mvsim` |
| `bringup` (VESC, LiDAR, camera) | `rbsf_bringup` | `mode:=real` |
| `centerline_tracking` (sub-launcher) | `rbsf_centerline_tracking` | always |
| `centerline_publisher` | `rbsf_slam_toolbox` | `centerline_csv` is set |
| `ackermann_to_twist` | `stack_launcher` | `mode:=mvsim` |
