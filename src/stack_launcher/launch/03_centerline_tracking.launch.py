import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)

    rbsf_bringup_share = FindPackageShare("rbsf_bringup").find("rbsf_bringup")
    rbsf_mvsim_share = FindPackageShare("rbsf_mvsim").find("rbsf_mvsim")
    rbsf_centerline_tracking_share = FindPackageShare("rbsf_centerline_tracking").find(
        "rbsf_centerline_tracking"
    )

    bringup_launch_files = {
        "real": os.path.join(rbsf_bringup_share, "launch", "bringup.launch.py"),
        "mvsim": os.path.join(rbsf_mvsim_share, "launch", "launch_world.launch.py"),
    }

    if mode not in bringup_launch_files:
        raise RuntimeError(
            f"Unknown mode '{mode}'. Valid options: {list(bringup_launch_files)}"
        )

    bringup_launch_arguments = {}
    if mode == "mvsim":
        bringup_launch_arguments = {
            "world_file": LaunchConfiguration("world_file"),
            "vehicle_config_file": LaunchConfiguration("vehicle_config_file"),
            "world_config_file": LaunchConfiguration("world_config_file"),
            "init_pose": LaunchConfiguration("init_pose"),
        }

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_files[mode]),
        launch_arguments=bringup_launch_arguments.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rbsf_centerline_tracking_share,
                "launch",
                "centerline_tracking.launch.py",
            )
        ),
        launch_arguments={"mode": mode}.items(),
    )

    nodes = [bringup, controller]

    if mode == "mvsim":
        nodes.append(
            Node(
                package="stack_launcher",
                executable="ackermann_to_twist",
                name="ackermann_to_twist",
                parameters=[{"input_topic": "/drive"}],
            )
        )

    return nodes


def generate_launch_description():
    centerline_publisher = Node(
        package="rbsf_slam_toolbox",
        executable="centerline_publisher",
        name="centerline_publisher",
        output="screen",
        parameters=[
            {
                "csv_path": LaunchConfiguration("centerline_csv"),
                "topic": LaunchConfiguration("centerline_topic"),
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("centerline_csv"), "' != ''"])
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                description="Bringup mode: 'real' or 'mvsim'",
                choices=["real", "mvsim"],
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("rbsf_mvsim"),
                    "maps",
                    "club_circuit.world.xml",
                ]),
                description="MVSim world file used when mode:=mvsim",
            ),
            DeclareLaunchArgument(
                "vehicle_config_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("rbsf_mvsim"),
                    "config",
                    "mvsim_vehicle.yaml",
                ]),
                description="MVSim vehicle config used when mode:=mvsim",
            ),
            DeclareLaunchArgument(
                "world_config_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("rbsf_mvsim"),
                    "config",
                    "mvsim_worlds.yaml",
                ]),
                description="MVSim world config used when mode:=mvsim",
            ),
            DeclareLaunchArgument(
                "init_pose",
                default_value="",
                description="MVSim initial pose override used when mode:=mvsim",
            ),
            DeclareLaunchArgument(
                "centerline_csv",
                default_value="",
                description="Optional CSV file used to publish nav_msgs/Path.",
            ),
            DeclareLaunchArgument(
                "centerline_topic",
                default_value="/centerline",
                description="Topic where the centerline path is published.",
            ),
            OpaqueFunction(function=launch_setup),
            centerline_publisher,
        ]
    )
