# rbsf-racing-irobocity

<img src="docs/media/robesafe.png" width="350"/> &nbsp; <img src="docs/media/uah_logo.png" width="350"/>

ROS 2 autonomous racing stack developed at RobeSafe (UAH) for the iRobocity summer course.
It runs on an F1Tenth-class car (or in MVSim simulation) and provides teleoperated SLAM mapping,
reactive wall-following, and centerline-tracking, all built on a unified launch system that
supports real, simulation, and distributed deployment modes.

---

## Requirements

| Dependency | Version |
|---|---|
| Ubuntu | 22.04 |
| Docker | Compatible with NVIDIA Container Toolkit |
| NVIDIA GPU | Driver compatible with CUDA 12.9 |
| Git | Any recent version |
| `xhost` | Pre-installed on Ubuntu (for GUI forwarding) |

---

## Installation

### 1. Clone the repository

```bash
git clone <repo-url>
cd rbsf-racing-irobocity
```

### 2. Initialize submodules

```bash
git submodule update --init --recursive
```

> `f1tenth_system` is itself a collection of submodules (`ackermann_mux`, `vesc`, `teleop_tools`).
> A plain `git submodule update --init` leaves those children empty and colcon won't find the packages.

### 3. Build the Docker image

```bash
make build
```

This builds two layered images: `curso_uah_irobocity:base` (ROS 2 Humble + CUDA + RealSense drivers)
and `curso_uah_irobocity:racing` (workspace + Python environment on top).
Your host UID/GID are baked in to avoid volume permission issues.

### 4. Run the container

```bash
make run
```

The repo root is mounted at `/ros2_ws` inside the container.
`entrypoint.sh` runs automatically and:

1. Sources `/opt/ros/humble/setup.bash`
2. Runs `colcon build` to compile the workspace
3. Sources `install/local_setup.bash`
4. Checks for `.venv` and runs `uv sync` if needed

### 5. Open additional terminals

```bash
make attach
```

---

## Usage

### Quick-start table

| Goal | Command |
|---|---|
| Teleoperated SLAM (real car, gamepad) | `ros2 launch stack_launcher 00_teleop_slam.launch.py` |
| Teleoperated SLAM (simulation, keyboard) | `ros2 launch stack_launcher 00_teleop_slam.launch.py mode:=mvsim teleop:=keyboard` |
| Wall following (real car) | `ros2 launch stack_launcher 01_wall_following.launch.py` |
| Wall following (simulation) | `ros2 launch stack_launcher 01_wall_following.launch.py mode:=mvsim` |
| Autonomous SLAM (real car) | `ros2 launch stack_launcher 02_autonomous_slam.launch.py` |
| Autonomous SLAM (simulation) | `ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=mvsim` |
| Centerline tracking (real car) | `ros2 launch stack_launcher 03_centerline_tracking.launch.py` |
| Centerline tracking (simulation) | `ros2 launch stack_launcher 03_centerline_tracking.launch.py mode:=mvsim` |
| Hardware bringup only (distributed, embedded device) | `ros2 launch stack_launcher 00_distributed_bringup.launch.py` |

See `docs/` for the detailed argument reference for each launch file.

---

## Deployment modes

Every stack launcher (except `00_distributed_bringup`) accepts a `mode` argument:

| Mode | Description |
|---|---|
| `real` | Full stack on a single machine with physical hardware attached |
| `mvsim` | MVSim simulation - no hardware required |
| `distributed` | Compute node only; hardware runs on a separate embedded device via `00_distributed_bringup.launch.py` |

Both devices in a distributed setup must share the same `ROS_DOMAIN_ID`
(set in `Makefile`, default **200** - change to your position number).

---

## ROS 2 packages

### `base_system/f1tenth_system`

F1Tenth platform drivers: VESC motor controller, LiDAR, USB camera, and joystick.
Provides the low-level sensor and actuator interface used by all stack launchers in `real` mode.

### `base_system/rbsf_bringup`

Top-level hardware bringup launch file (`bringup.launch.py`) that starts the F1Tenth drivers
with project-specific configuration.

### `base_system/rbsf_mvsim`

MVSim simulation environment. Provides world files (`.world.xml`) for F1Tenth tracks and a
launch file (`launch_world.launch.py`) that mirrors the hardware interface.

Available worlds in `rbsf_mvsim/maps/`:

- `f1tenth_catalunya.world.xml`
- `f1tenth_saopaulo.world.xml`
- `f1tenth_yasmarina.world.xml`
- `hallway_simulation_sci.world.xml`
- `simple_oval.world.xml`

### `controller/rbsf_wall_following`

Reactive wall-following controller.

| I/O | Topic | Type |
|---|---|---|
| Input | `/scan` (real) / `/laser1` (sim) | `sensor_msgs/LaserScan` |
| Output | `/drive` | `ackermann_msgs/AckermannDriveStamped` |

A PI controller computes steering from two error terms: distance to the wall
(`distance_kp`, `distance_ki`) and heading angle relative to it (`orientation_kp`, `orientation_ki`).
Parameters are tuned in `config/wall_following_real.yaml` and `config/wall_following_sim.yaml`.

### `controller/rbsf_centerline_tracking`

Path-tracking controller that follows a centerline published as `nav_msgs/Path`.

| I/O | Topic | Type |
|---|---|---|
| Input path | `/centerline` | `nav_msgs/Path` |
| Input pose | `/odom` | `nav_msgs/Odometry` |
| Output | `/drive` | `ackermann_msgs/AckermannDriveStamped` |

Parameters live in `config/centerline_tracking_real.yaml` and `config/centerline_tracking_sim.yaml`.

### `localization/rbsf_slam_toolbox`

Wraps SLAM Toolbox with project-specific configuration and adds two utility nodes:

- **`map_saver`** - saves the active SLAM map to a timestamped directory on gamepad button press
  (or keyboard `m` key in simulation).
- **`centerline_publisher`** - reads a centerline CSV and publishes it as `nav_msgs/Path`
  on a configurable topic (used by `03_centerline_tracking`).

Maps are written to `/ros2_ws/maps/map_YYYYMMDD_HHMMSS/`.

### `stack_launcher`

Top-level launch files that wire all packages together:

| Launch file | Description | Full doc |
|---|---|---|
| `00_distributed_bringup.launch.py` | Hardware-only bringup for the embedded device in distributed mode | `docs/00_DISTRIBUTED_BRINGUP.md` |
| `00_teleop_slam.launch.py` | Teleoperated SLAM mapping (keyboard or gamepad) | `docs/00_TELEOP_SLAM.md` |
| `01_wall_following.launch.py` | Autonomous wall following | `docs/01_WALL_FOLLOWING.md` |
| `02_autonomous_slam.launch.py` | Autonomous wall following + simultaneous SLAM mapping | `docs/02_AUTONOMOUS_SLAM.md` |
| `03_centerline_tracking.launch.py` | Centerline-tracking autonomous driving | `docs/03_CENTERLINE_TRACKING.md` |

---

## Repository layout

```
.
+-- deploy/
|   +-- Dockerfile.base       # CUDA + ROS 2 Humble base image
|   +-- Dockerfile.racing     # Workspace + Python env on top of base
+-- docs/
|   +-- INSTALL.md
|   +-- 00_DISTRIBUTED_BRINGUP.md
|   +-- 00_TELEOP_SLAM.md
|   +-- 01_WALL_FOLLOWING.md
|   +-- 02_AUTONOMOUS_SLAM.md
|   +-- 03_CENTERLINE_TRACKING.md
+-- maps/                     # Saved SLAM maps (map_YYYYMMDD_HHMMSS/)
+-- src/
|   +-- base_system/
|   |   +-- f1tenth_system/   # F1Tenth platform drivers (submodule)
|   |   +-- rbsf_bringup/     # Hardware bringup launch
|   |   +-- rbsf_mvsim/       # MVSim simulation environment
|   +-- controller/
|   |   +-- rbsf_wall_following/     # Reactive PI wall-following controller
|   |   +-- rbsf_centerline_tracking/ # Path-tracking controller
|   +-- localization/
|   |   +-- rbsf_slam_toolbox/       # SLAM Toolbox wrapper + map saver + centerline publisher
|   +-- stack_launcher/       # Top-level launch files
+-- entrypoint.sh             # Container startup script
+-- Makefile                  # build / run / attach / clear targets
+-- pyproject.toml            # Python dependencies (managed by uv)
```

---

## Authors

- Santiago Montiel-Marín
- Rodrigo Gutiérrez Moreno
- Miguel Antunes-García
- Fabio Sánchez-García
