from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    startup_delay = float(LaunchConfiguration("startup_delay").perform(context))

    return [
        TimerAction(
            period=startup_delay,
            actions=[
                Node(
                    package="rbsf_wall_following",
                    executable="wall_follower",
                    name="wall_follower",
                    output="screen",
                )
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "startup_delay",
                default_value="2.5",
                description="Seconds to wait before starting the wall_follower node.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
