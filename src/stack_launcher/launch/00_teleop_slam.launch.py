"""
00_teleop_slam — Teleoperated mapping session.

Drive the car while SLAM Toolbox builds a map.  Works in simulation (mvsim)
and on the physical car (real).  Keyboard mode is only available in mvsim.

Quick start:
    ros2 launch stack_launcher 00_teleop_slam.launch.py mode:=mvsim                    # sim + keyboard
    ros2 launch stack_launcher 00_teleop_slam.launch.py mode:=mvsim teleop:=controller # sim + gamepad
    ros2 launch stack_launcher 00_teleop_slam.launch.py                                # real car (controller)

Map save:
    Keyboard   — press 'm' in the xterm window.
    Controller — press save_button on the gamepad (default: button 0).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    teleop = LaunchConfiguration("teleop").perform(context)
    max_speed = float(LaunchConfiguration("max_speed").perform(context))

    if mode == "real" and teleop == "keyboard":
        raise RuntimeError(
            "Keyboard teleoperation is not supported on the real car. "
            "Use teleop:=controller."
        )

    rbsf_bringup_share = FindPackageShare("rbsf_bringup").find("rbsf_bringup")
    rbsf_mvsim_share = FindPackageShare("rbsf_mvsim").find("rbsf_mvsim")
    rbsf_slam_toolbox_share = FindPackageShare("rbsf_slam_toolbox").find("rbsf_slam_toolbox")

    world_launch_files = {
        "real":  os.path.join(rbsf_bringup_share, "launch", "bringup.launch.py"),
        "mvsim": os.path.join(rbsf_mvsim_share,   "launch", "launch_world.launch.py"),
    }

    if mode not in world_launch_files:
        raise RuntimeError(f"Unknown mode '{mode}'. Choose from: {list(world_launch_files)}")

    if mode == "real":
        world_args = {"teleop_device": "controller"}
    else:
        world_args = {
            "world_file":          LaunchConfiguration("world_file"),
            "vehicle_config_file": LaunchConfiguration("vehicle_config_file"),
            "world_config_file":   LaunchConfiguration("world_config_file"),
            "init_pose":           LaunchConfiguration("init_pose"),
            "teleop_device":       teleop,
        }
        if teleop == "controller":
            world_args["joy_config"] = os.path.join(
                rbsf_mvsim_share, "config", "joy_teleop_slam.yaml"
            )

    world_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_files[mode]),
        launch_arguments=world_args.items(),
    )

    # SLAM Toolbox — auto-select params by mode or use override.
    default_slam_params = {
        "real":  os.path.join(rbsf_slam_toolbox_share, "config", "mapper_params_real.yaml"),
        "mvsim": os.path.join(rbsf_slam_toolbox_share, "config", "mapper_params_sim.yaml"),
    }
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    resolved_slam_params = default_slam_params[mode] if slam_params_file == "auto" else slam_params_file

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rbsf_slam_toolbox_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={
            "slam_params_file": resolved_slam_params,
            "use_sim_time":     "true" if mode == "mvsim" else "false",
        }.items(),
    )

    map_saver = Node(
        package="rbsf_slam_toolbox",
        executable="map_saver",
        name="map_saver_node",
        parameters=[{
            "map_output_dir": LaunchConfiguration("map_output_dir"),
            "save_button":    LaunchConfiguration("save_button"),
        }],
    )

    nodes = [world_bringup, slam, map_saver]

    if mode == "mvsim" and teleop == "keyboard":
        nodes.append(Node(
            package="stack_launcher",
            executable="ackermann_to_twist",
            name="ackermann_to_twist",
            parameters=[{"input_topic": "/ackermann_drive"}],
        ))
        nodes.append(Node(
            package="stack_launcher",
            executable="keyboard_teleop",
            name="keyboard_teleop",
            prefix="xterm -e",
            parameters=[{
                "output_topic":   "teleop",
                "forward_speed":  max_speed,
                "backward_speed": max_speed,
            }],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="real",
            choices=["real", "mvsim"],
            description="Bringup mode: 'real' (physical car) or 'mvsim' (simulator)",
        ),
        DeclareLaunchArgument(
            "teleop",
            default_value="keyboard",
            choices=["controller", "keyboard"],
            description="Input device: 'controller' (gamepad) or 'keyboard' (WASD / arrow keys)  [mode:=mvsim only]",
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

        # --- keyboard arguments ---------------------------------------- #
        DeclareLaunchArgument(
            "max_speed",
            default_value="1.0",
            description="Maximum speed in m/s for keyboard teleop  [mode:=mvsim teleop:=keyboard]",
        ),

        # --- SLAM ------------------------------------------------------- #
        DeclareLaunchArgument(
            "slam_params_file",
            default_value="auto",
            description=(
                "Path to slam_toolbox params YAML. "
                "Default 'auto' selects mapper_params_real.yaml or mapper_params_sim.yaml by mode."
            ),
        ),

        # --- map saver ------------------------------------------------- #
        DeclareLaunchArgument(
            "map_output_dir",
            default_value="/ros2_ws/maps",
            description="Directory where map files are written on save",
        ),
        DeclareLaunchArgument(
            "save_button",
            default_value="0",
            description="Gamepad button index that triggers a map save  [teleop:=controller]",
        ),

        OpaqueFunction(function=launch_setup),
    ])
