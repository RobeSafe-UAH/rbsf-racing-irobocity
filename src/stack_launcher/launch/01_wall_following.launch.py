"""
01_wall_following — Autonomous wall-following with a PID controller.

The car follows the wall on the selected side at a configurable distance.
Works in simulation (mvsim) and on the physical car (real).

Quick start:
    ros2 launch stack_launcher 01_wall_following.launch.py                   # sim, right wall
    ros2 launch stack_launcher 01_wall_following.launch.py mode:=real        # physical car
    ros2 launch stack_launcher 01_wall_following.launch.py side:=left        # left wall
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    side = LaunchConfiguration("side").perform(context)
    desired_distance = LaunchConfiguration("desired_distance").perform(context)
    speed = LaunchConfiguration("speed").perform(context)
    kp = LaunchConfiguration("kp").perform(context)
    ki = LaunchConfiguration("ki").perform(context)
    kd = LaunchConfiguration("kd").perform(context)

    rbsf_bringup_share = FindPackageShare("rbsf_bringup").find("rbsf_bringup")
    rbsf_mvsim_share = FindPackageShare("rbsf_mvsim").find("rbsf_mvsim")
    rbsf_wall_follower_share = FindPackageShare("rbsf_wall_follower").find("rbsf_wall_follower")

    # ------------------------------------------------------------------ #
    # World bringup                                                        #
    # ------------------------------------------------------------------ #
    world_launch = {
        "real":  os.path.join(rbsf_bringup_share, "launch", "bringup.launch.py"),
        "mvsim": os.path.join(rbsf_mvsim_share,   "launch", "launch_world.launch.py"),
    }
    if mode not in world_launch:
        raise RuntimeError(f"Unknown mode '{mode}'. Choose from: {list(world_launch)}")

    world_args = {}
    if mode == "mvsim":
        world_args = {
            "world_file":          LaunchConfiguration("world_file"),
            "vehicle_config_file": LaunchConfiguration("vehicle_config_file"),
            "world_config_file":   LaunchConfiguration("world_config_file"),
            "init_pose":           LaunchConfiguration("init_pose"),
        }

    world_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch[mode]),
        launch_arguments=world_args.items(),
    )

    # ------------------------------------------------------------------ #
    # PID wall-follower controller                                         #
    # ------------------------------------------------------------------ #
    default_params = {
        "real":  os.path.join(rbsf_wall_follower_share, "config", "pid_params_real.yaml"),
        "mvsim": os.path.join(rbsf_wall_follower_share, "config", "pid_params_sim.yaml"),
    }
    params_file = LaunchConfiguration("params_file").perform(context)
    resolved_params = default_params[mode] if params_file == "auto" else params_file

    # side is always set (has real choices, no "auto").
    # Numeric gains use "auto" as a sentinel meaning "use the YAML value".
    overrides = {
        "side":         side,
        "use_sim_time": "true" if mode == "mvsim" else "false",
    }
    for key, val in [
        ("desired_distance", desired_distance),
        ("speed", speed),
        ("kp", kp),
        ("ki", ki),
        ("kd", kd),
    ]:
        if val != "auto":
            overrides[key] = val

    controller = Node(
        package="rbsf_wall_follower",
        executable="wall_follower",
        name="pid_controller",
        output="screen",
        parameters=[resolved_params, overrides],
    )

    nodes = [world_bringup, controller]

    # ------------------------------------------------------------------ #
    # mvsim only: bridge AckermannDriveStamped → Twist for the simulator  #
    # ------------------------------------------------------------------ #
    if mode == "mvsim":
        nodes.append(Node(
            package="stack_launcher",
            executable="ackermann_to_twist",
            name="ackermann_to_twist",
            parameters=[{"input_topic": "/drive"}],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="mvsim",
            choices=["real", "mvsim"],
            description="Bringup mode: 'real' (physical car) or 'mvsim' (simulator)",
        ),

        # --- mvsim arguments ------------------------------------------- #
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rbsf_mvsim"), "maps", "f1tenth_catalunya.world.xml",
            ]),
            description="MVSim world file  [mode:=mvsim]",
        ),
        DeclareLaunchArgument(
            "vehicle_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rbsf_mvsim"), "config", "mvsim_vehicle.yaml",
            ]),
            description="MVSim vehicle config  [mode:=mvsim]",
        ),
        DeclareLaunchArgument(
            "world_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rbsf_mvsim"), "config", "mvsim_worlds.yaml",
            ]),
            description="MVSim world parameters  [mode:=mvsim]",
        ),
        DeclareLaunchArgument(
            "init_pose",
            default_value="",
            description="Initial pose override: 'x y yaw_deg'  [mode:=mvsim]",
        ),

        # --- controller arguments -------------------------------------- #
        DeclareLaunchArgument(
            "side",
            default_value="right",
            choices=["left", "right"],
            description="Wall side to follow",
        ),
        DeclareLaunchArgument(
            "desired_distance",
            default_value="auto",
            description="Target distance from the wall in metres (default: from params YAML)",
        ),
        DeclareLaunchArgument(
            "speed",
            default_value="auto",
            description="Forward speed in m/s (default: from params YAML)",
        ),
        DeclareLaunchArgument(
            "kp",
            default_value="auto",
            description="PID proportional gain (default: from params YAML)",
        ),
        DeclareLaunchArgument(
            "ki",
            default_value="auto",
            description="PID integral gain (default: from params YAML)",
        ),
        DeclareLaunchArgument(
            "kd",
            default_value="auto",
            description="PID derivative gain (default: from params YAML)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value="auto",
            description="Override PID params YAML (default: auto-selects by mode)",
        ),

        OpaqueFunction(function=launch_setup),
    ])
