import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    params_file_arg = LaunchConfiguration("params_file").perform(context)

    rbsf_wall_follower_share = FindPackageShare("rbsf_wall_follower").find(
        "rbsf_wall_follower"
    )
    params_files = {
        "real": os.path.join(
            rbsf_wall_follower_share, "config", "pid_params_real.yaml"
        ),
        "mvsim": os.path.join(
            rbsf_wall_follower_share, "config", "pid_params_sim.yaml"
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
        Node(
            package="rbsf_wall_follower",
            executable="wall_follower",
            name="pid_controller",
            output="screen",
            parameters=[params_file],
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
                description="PID controller parameters YAML.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
