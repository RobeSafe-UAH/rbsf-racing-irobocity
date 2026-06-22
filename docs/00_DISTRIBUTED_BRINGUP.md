# 00 - Distributed Bringup

Hardware-only bringup for distributed deployments.

Run this file **on the embedded device** (Jetson, RPi, etc.) that has the physical hardware
attached. It brings up VESC, LiDAR, camera, and joystick drivers.
The compute node (laptop or workstation) runs a separate stack launcher with `mode:=distributed`:

```
Embedded device  ->  00_distributed_bringup.launch.py
Compute node     ->  01_wall_following.launch.py mode:=distributed
                     02_autonomous_slam.launch.py mode:=distributed
```

Both devices must share the same `ROS_DOMAIN_ID` so DDS discovery can find each other
over the network.

---

## Quick start

```bash
# On the embedded device - gamepad (default)
ros2 launch stack_launcher 00_distributed_bringup.launch.py

# On the embedded device - keyboard
ros2 launch stack_launcher 00_distributed_bringup.launch.py teleop_device:=keyboard
```

Then, on the compute node:

```bash
ros2 launch stack_launcher 01_wall_following.launch.py mode:=distributed
```

---

## Arguments

| Argument | Default | Choices | Description |
|---|---|---|---|
| `teleop_device` | `controller` | `controller`, `keyboard` | Teleop input device |
| `joy_config` | `f1tenth_stack/config/joy_teleop.yaml` | - | Path to `joy_teleop` YAML config |
| `speed_max_erpm` | `23250.0` | - | VESC speed upper limit in ERPM |
| `speed_min_erpm` | `-23250.0` | - | VESC speed lower limit in ERPM |

---

## Nodes launched

| Node | Package | Condition |
|---|---|---|
| `bringup` (VESC, LiDAR, camera, joystick) | `rbsf_bringup` | always |
