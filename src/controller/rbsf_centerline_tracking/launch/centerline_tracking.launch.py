from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    remappings = (
        [("/odom", "/base_pose_ground_truth")]
        if mode == "mvsim"
        else []
    )

    return [
        Node(
            package="rbsf_centerline_tracking",
            executable="centerline_tracker",
            name="centerline_tracker",
            output="screen",
            remappings=remappings,
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
            OpaqueFunction(function=launch_setup),
        ]
    )
