from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    startup_delay = float(LaunchConfiguration("startup_delay").perform(context))

    remappings = [("/scan", "/laser1")] if mode == "mvsim" else []

    return [
        TimerAction(
            period=startup_delay,
            actions=[
                Node(
                    package="rbsf_wall_following",
                    executable="wall_follower",
                    name="wall_follower",
                    output="screen",
                    remappings=remappings,
                )
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                description="Controller mode: 'real' or 'mvsim'",
                choices=["real", "mvsim"],
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="2.5",
                description="Seconds to wait before starting the wall_follower node.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
