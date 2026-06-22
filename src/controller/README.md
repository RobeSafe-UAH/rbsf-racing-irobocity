# Racing Controllers

ROS 2 controllers for the iRobocity racing course. This folder contains two
behaviours: reactive wall following from LiDAR data and centerline tracking
from an ordered path and vehicle odometry.

---

## Requirements

| Dependency | Purpose |
|---|---|
| ROS 2 Humble | Nodes, topics and launch system |
| `sensor_msgs` | LiDAR input messages |
| `nav_msgs` | Path and odometry messages |
| `ackermann_msgs` | Vehicle control commands |
| MVSim | Simulation mode |

The complete workspace and its Docker environment are documented in
[`docs/INSTALL.md`](../../docs/INSTALL.md).

---

## Build

From the workspace root, inside the course container:

```bash
colcon build --packages-select \
  rbsf_wall_following rbsf_centerline_tracking stack_launcher
source install/setup.bash
```

---

## ROS 2 packages

### `rbsf_wall_following`

Reactive controller that follows the left wall without localization or a map.
It reads two LiDAR rays, estimates the local wall geometry and publishes an
Ackermann steering command at constant speed.

#### Interfaces

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/scan` | `sensor_msgs/LaserScan` | Physical-car LiDAR scan |
| Input | `/laser1` | `sensor_msgs/LaserScan` | MVSim LiDAR scan |
| Output | `/drive` | `ackermann_msgs/AckermannDriveStamped` | Speed and steering command |

Set `scan_topic` directly in `wall_follower_node.py` for the environment being
used.

#### Controller pipeline

1. Compute the sampling period from consecutive LiDAR timestamps.
2. Read the lateral ray at 90 degrees and the forward-left ray at 60 degrees.
3. Convert both polar measurements into Cartesian points.
4. Estimate wall distance and wall orientation from those points.
5. Compute distance and orientation errors.
6. Apply one PI contribution to each error, including integral anti-windup.
7. Add both contributions and publish the Ackermann command.

The controller is:

```text
distance_error = measured_distance - reference_distance

distance_steering = distance_kp * distance_error
                  + distance_ki * integral(distance_error dt)

orientation_steering = orientation_kp * orientation_error
                     + orientation_ki * integral(orientation_error dt)

steering = distance_steering + orientation_steering
```

#### Exercises — `wall_follower_node.py`

Complete the exercises in order in
`rbsf_wall_following/rbsf_wall_following/wall_follower_node.py`. Exercise 0
initializes the node, followed by five controller exercises. Exercises 0–4 end
with a temporary `return`; remove it after completing that exercise to continue
to the next one.

0. **Controller initialization** — fill in the topics, vehicle references and
   controller gains directly in `__init__`.
1. **Sampling period** — obtain `dt` from the current and previous
   LiDAR timestamps, using `scan_time` for the first message. The conversion
   of the ROS timestamp to seconds is provided.
2. **Lateral ray and distance error** — convert the lateral angle to a scan
   index, obtain its Cartesian point and compute the distance error.
3. **Forward-left ray and orientation error** — obtain the second Cartesian
   point, estimate the wall orientation and compute its error.
4. **PI steering** — integrate both errors with `dt`, retain the supplied
   anti-windup clamps and combine the proportional and integral terms.
5. **Ackermann command** — fill the speed and steering fields and publish the
   command on `/drive`.

#### Run

```bash
# MVSim
ros2 launch stack_launcher 01_wall_following.launch.py mode:=mvsim

# Physical car
ros2 launch stack_launcher 01_wall_following.launch.py mode:=real
```

---

### `rbsf_centerline_tracking`

Path-tracking controller that selects a lookahead target from an ordered
centerline and controls the vehicle orientation with a PI controller.

#### Interfaces

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/centerline` | `nav_msgs/Path` | Ordered closed-loop centerline |
| Input | `/odom` | `nav_msgs/Odometry` | Physical-car pose |
| Input | `/base_pose_ground_truth` | `nav_msgs/Odometry` | MVSim global pose |
| Output | `/drive` | `ackermann_msgs/AckermannDriveStamped` | Speed and steering command |

The centerline can be loaded from a CSV file by
`rbsf_slam_toolbox/centerline_publisher_node`.

Topics, speed, lookahead distance and PI gains are configured directly in
`centerline_tracker_node.py`. The node subscribes to both `/odom` and
`/base_pose_ground_truth`, so it does not need separate real and simulation
parameter files.

#### Exercises — `centerline_tracker_node.py`

Complete the exercises in order. Exercises 1 and 2 end with a temporary
`return`; remove it after completing that exercise to continue to the next
one. Timestamp handling, vehicle pose, quaternion conversion, closest-point
search and integral anti-windup are already provided.

1. **Lookahead target** — walk forward from the closest centerline point and
   select the first cyclic path point at the configured lookahead distance.
2. **Orientation error** — compute the desired angle, subtract the current
   vehicle angle and then normalize the result to `[-pi, pi]`.
3. **PI control and command** — integrate the angular error, compute steering
   and publish the constant-speed Ackermann command.

#### Run

```bash
ros2 launch stack_launcher 03_centerline_tracking.launch.py \
  mode:=mvsim centerline_csv:=/path/to/centerline_output.csv
```
