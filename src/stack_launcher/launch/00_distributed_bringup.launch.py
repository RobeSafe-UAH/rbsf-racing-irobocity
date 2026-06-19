"""
00_distributed_bringup — Hardware bringup for distributed mode.

Run this file ONLY on the embedded device (Jetson, RPi, etc.) that has the
physical hardware attached.  It brings up VESC, LiDAR, camera and joystick
drivers.  The compute node runs a separate stack launcher with mode:=distributed
(e.g. 01_wall_following.launch.py mode:=distributed).

Both devices must share the same ROS_DOMAIN_ID so their DDS discovery finds
each other over the network.

Quick start (on the embedded device):
    ros2 launch stack_launcher 00_distributed_bringup.launch.py
    ros2 launch stack_launcher 00_distributed_bringup.launch.py teleop_device:=keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rbsf_bringup_share = FindPackageShare("rbsf_bringup").find("rbsf_bringup")
    f1tenth_config = os.path.join(
        get_package_share_directory("f1tenth_stack"), "config"
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rbsf_bringup_share, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "teleop_device":  LaunchConfiguration("teleop_device"),
            "joy_config":     LaunchConfiguration("joy_config"),
            "speed_max_erpm": LaunchConfiguration("speed_max_erpm"),
            "speed_min_erpm": LaunchConfiguration("speed_min_erpm"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "teleop_device",
            default_value="controller",
            choices=["controller", "keyboard"],
            description="Teleop input device: 'controller' (gamepad) or 'keyboard'",
        ),
        DeclareLaunchArgument(
            "joy_config",
            default_value=os.path.join(f1tenth_config, "joy_teleop.yaml"),
            description="Path to joy_teleop YAML config",
        ),
        DeclareLaunchArgument(
            "speed_max_erpm",
            default_value="23250.0",
            description="VESC speed upper limit in ERPM",
        ),
        DeclareLaunchArgument(
            "speed_min_erpm",
            default_value="-23250.0",
            description="VESC speed lower limit in ERPM",
        ),
        bringup,
    ])
