from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(["/ros2_ws/src/base_system/rbsf_bringup/rviz/base_cfg.rviz"]),
            description="Path to the RViz2 configuration file",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])