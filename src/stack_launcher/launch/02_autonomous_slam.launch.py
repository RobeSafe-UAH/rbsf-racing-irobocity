"""
02_autonomous_slam — Autonomous wall-following with SLAM mapping.

Same as 01_wall_following but adds slam_toolbox online-async mapping in the
background.  Works in simulation (mvsim), on the physical car (real), and in
distributed mode where hardware runs on a separate embedded device.

Quick start:
    ros2 launch stack_launcher 02_autonomous_slam.launch.py                        # real car
    ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=mvsim            # simulation
    ros2 launch stack_launcher 02_autonomous_slam.launch.py mode:=distributed      # compute node only

Map save:
    mvsim            — press 'm' in the xterm window, or save_button on the gamepad (default: button 0).
    real/distributed — press save_button on the gamepad (default: button 0).
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Maps the top-level mode to the config mode expected by sub-launchers.
# Sub-launchers (wall_following, slam_toolbox, etc.) only understand "real" and
# "mvsim"; "distributed" uses the same hardware config as "real".
_CONTROLLER_MODE = {
    "real":        "real",
    "mvsim":       "mvsim",
    "distributed": "real",
}


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)

    # Routes to packages.
    rbsf_bringup_share = FindPackageShare("rbsf_bringup").find("rbsf_bringup")
    rbsf_mvsim_share = FindPackageShare("rbsf_mvsim").find("rbsf_mvsim")
    rbsf_wall_following_share = FindPackageShare("rbsf_wall_following").find(
        "rbsf_wall_following"
    )
    rbsf_slam_toolbox_share = FindPackageShare("rbsf_slam_toolbox").find(
        "rbsf_slam_toolbox"
    )

    # Load bringup or simulation world.
    world_launch_files = {
        "real": os.path.join(
            rbsf_bringup_share, "launch", "bringup.launch.py"
        ),
        "mvsim": os.path.join(
            rbsf_mvsim_share, "launch", "launch_world.launch.py"
        ),
    }

    if mode not in _CONTROLLER_MODE:
        raise RuntimeError(
            f"Unknown mode '{mode}'. Valid options: {list(_CONTROLLER_MODE)}"
        )

    nodes = []

    # In distributed mode the embedded device runs 00_distributed_bringup;
    # skip hardware/simulation bringup on this compute node.
    if mode != "distributed":
        world_launch_arguments = {}
        if mode == "mvsim":
            world_launch_arguments = {
                "world_file": LaunchConfiguration("world_file"),
                "vehicle_config_file": LaunchConfiguration("vehicle_config_file"),
                "world_config_file": LaunchConfiguration("world_config_file"),
                "init_pose": LaunchConfiguration("init_pose"),
            }

        world_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch_files[mode]),
            launch_arguments=world_launch_arguments.items(),
        )
        nodes.append(world_bringup)

    # Load wall-following controller launch file.
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rbsf_wall_following_share,
                "launch",
                "wall_following.launch.py",
            )
        ),
        launch_arguments={
            "mode":          _CONTROLLER_MODE[mode],
            "startup_delay": LaunchConfiguration("startup_delay"),
        }.items(),
    )
    nodes.append(controller)

    # Resolve slam params file: auto-select by mode or use the override.
    default_slam_params = {
        "real":  os.path.join(rbsf_slam_toolbox_share, "config", "mapper_params_real.yaml"),
        "mvsim": os.path.join(rbsf_slam_toolbox_share, "config", "mapper_params_sim.yaml"),
    }
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    resolved_slam_params = (
        default_slam_params[_CONTROLLER_MODE[mode]]
        if slam_params_file == "auto"
        else slam_params_file
    )

    use_sim_time = "true" if mode == "mvsim" else "false"

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rbsf_slam_toolbox_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={
            "slam_params_file": resolved_slam_params,
            "use_sim_time":     use_sim_time,
        }.items(),
    )
    nodes.append(slam)

    map_saver = Node(
        package="rbsf_slam_toolbox",
        executable="map_saver",
        name="map_saver_node",
        parameters=[{
            "map_output_dir": LaunchConfiguration("map_output_dir"),
            "save_button":    LaunchConfiguration("save_button"),
        }],
    )
    nodes.append(map_saver)

    if mode == "mvsim":
        # Bridge Ackermann commands to the Twist interface used by MVSim.
        nodes.append(
            Node(
                package="stack_launcher",
                executable="ackermann_to_twist",
                name="ackermann_to_twist",
                parameters=[{"input_topic": "/drive"}],
            )
        )
        # Keyboard map trigger — 'm' saves the map, drive keys disabled.
        nodes.append(
            Node(
                package="stack_launcher",
                executable="keyboard_teleop",
                name="keyboard_map_trigger",
                prefix="xterm -e",
                parameters=[{"teleop_enabled": False}],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                description=(
                    "Bringup mode: 'real' (full stack on this machine), "
                    "'mvsim' (simulation), or 'distributed' (compute node only — "
                    "hardware runs on a separate embedded device via "
                    "00_distributed_bringup.launch.py)"
                ),
                choices=["real", "mvsim", "distributed"],
            ),

            # --- mvsim arguments ------------------------------------------- #
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("rbsf_mvsim"),
                    "maps",
                    "f1tenth_catalunya.world.xml",
                ]),
                description="MVSim world file  [mode:=mvsim]",
            ),
            DeclareLaunchArgument(
                "vehicle_config_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("rbsf_mvsim"),
                    "config",
                    "mvsim_vehicle.yaml",
                ]),
                description="MVSim vehicle config  [mode:=mvsim]",
            ),
            DeclareLaunchArgument(
                "world_config_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("rbsf_mvsim"),
                    "config",
                    "mvsim_worlds.yaml",
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
                "startup_delay",
                default_value="2.5",
                description="Seconds to wait before starting the wall_follower node",
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
                description="Gamepad button index that triggers a map save",
            ),

            OpaqueFunction(function=launch_setup),
        ]
    )
