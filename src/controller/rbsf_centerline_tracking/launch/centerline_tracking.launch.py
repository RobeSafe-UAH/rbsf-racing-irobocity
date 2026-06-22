from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rbsf_centerline_tracking",
                executable="centerline_tracker",
                name="centerline_tracker",
                output="screen",
            ),
        ]
    )
