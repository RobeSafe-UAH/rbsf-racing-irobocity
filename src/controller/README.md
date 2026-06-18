# Controller Packages

This folder contains autonomous controllers used for teaching and experiments.
Each behaviour lives in its own ROS 2 package so the inputs, outputs and tuning
parameters are easy to inspect independently.

## `rbsf_wall_following`

Reactive controller that follows a wall using a `sensor_msgs/LaserScan`.

- Input: laser scan (`/scan` on the real car, `/laser1` in MVSim).
- Output: `ackermann_msgs/AckermannDriveStamped` on `/drive`.
- Executable: `wall_follower`.
- Launch: `wall_following.launch.py`.
- Configs:
  - `wall_following_real.yaml`
  - `wall_following_sim.yaml`

Run it standalone:

```bash
ros2 launch rbsf_wall_following wall_following.launch.py mode:=mvsim
```

## `rbsf_centerline_tracking`

Path-tracking controller that follows a centerline published as
`nav_msgs/Path`.

- Input path: `/centerline`.
- Input pose: odometry on `/odom`.
- Output: `ackermann_msgs/AckermannDriveStamped` on `/drive`.
- Executable: `centerline_tracker`.
- Launch: `centerline_tracking.launch.py`.
- Configs:
  - `centerline_tracking_real.yaml`
  - `centerline_tracking_sim.yaml`

Run it standalone:

```bash
ros2 launch rbsf_centerline_tracking centerline_tracking.launch.py mode:=mvsim
```

The centerline path can be published by
`rbsf_slam_toolbox/centerline_publisher_node` from a CSV file.
