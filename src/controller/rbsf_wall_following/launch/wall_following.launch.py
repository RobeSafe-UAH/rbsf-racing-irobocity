import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    params_file_arg = LaunchConfiguration("params_file").perform(context)
    startup_delay = float(LaunchConfiguration("startup_delay").perform(context))

    rbsf_wall_following_share = FindPackageShare("rbsf_wall_following").find(
        "rbsf_wall_following"
    )
    params_files = {
        "real": os.path.join(
            rbsf_wall_following_share, "config", "wall_following_real.yaml"
        ),
        "mvsim": os.path.join(
            rbsf_wall_following_share, "config", "wall_following_sim.yaml"
        ),
    }

    if mode not in params_files:
        raise RuntimeError(
            f"Unknown mode '{mode}'. Valid options: {list(params_files)}"
        )

    params_file = (
        params_files[mode] if params_file_arg == "auto" else params_file_arg
    )

    return [
        TimerAction(
            period=startup_delay,
            actions=[
                Node(
                    package="rbsf_wall_following",
                    executable="wall_follower",
                    name="wall_follower",
                    output="screen",
                    parameters=[params_file],
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
                description="Controller config mode: 'real' or 'mvsim'",
                choices=["real", "mvsim"],
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="auto",
                description="Wall-following controller parameters YAML.",
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="5.0",
                description="Seconds to wait before starting the wall_follower node.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
