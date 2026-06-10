"""
00_teleop_slam — Teleoperated mapping session.

Drive the car manually while SLAM Toolbox builds a map.  Works in
simulation (mvsim) and on the physical car (real).  Map is saved by
pressing a gamepad button (controller mode) or calling the SLAM Toolbox
service directly (keyboard mode).

Quick start:
    ros2 launch stack_launcher 00_teleop_slam.launch.py                          # sim + keyboard
    ros2 launch stack_launcher 00_teleop_slam.launch.py teleop:=controller       # sim + gamepad
    ros2 launch stack_launcher 00_teleop_slam.launch.py mode:=real teleop:=controller
    ros2 launch stack_launcher 00_teleop_slam.launch.py max_speed:=2.0           # set max speed (m/s, default 1.0)

Map save:
    Controller — press button <save_button> (default: 0) on the gamepad.
    Keyboard  — press 'm' in the xterm window (keystrokes are hidden).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Must match speed_to_erpm_gain in f1tenth_stack/config/vesc.yaml.
_SPEED_TO_ERPM_GAIN = 4420.0


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    teleop = LaunchConfiguration("teleop").perform(context)
    max_speed = float(LaunchConfiguration("max_speed").perform(context))
    map_output_dir = LaunchConfiguration("map_output_dir").perform(context)
    save_button = int(LaunchConfiguration("save_button").perform(context))

    rbsf_bringup_share = FindPackageShare("rbsf_bringup").find("rbsf_bringup")
    rbsf_mvsim_share = FindPackageShare("rbsf_mvsim").find("rbsf_mvsim")
    rbsf_slam_toolbox_share = FindPackageShare("rbsf_slam_toolbox").find("rbsf_slam_toolbox")

    # ------------------------------------------------------------------ #
    # World bringup                                                        #
    # ------------------------------------------------------------------ #
    world_launch = {
        "real":  os.path.join(rbsf_bringup_share, "launch", "bringup.launch.py"),
        "mvsim": os.path.join(rbsf_mvsim_share,   "launch", "launch_world.launch.py"),
    }
    if mode not in world_launch:
        raise RuntimeError(f"Unknown mode '{mode}'. Choose from: {list(world_launch)}")

    world_args = {"teleop_device": teleop}

    if mode == "mvsim":
        world_args.update({
            "world_file":         LaunchConfiguration("world_file"),
            "vehicle_config_file": LaunchConfiguration("vehicle_config_file"),
            "world_config_file":  LaunchConfiguration("world_config_file"),
            "init_pose":          LaunchConfiguration("init_pose"),
        })

    if mode == "real":
        erpm = max_speed * _SPEED_TO_ERPM_GAIN
        world_args.update({
            "speed_max_erpm": str(erpm),
            "speed_min_erpm": str(-erpm),
        })

    world_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch[mode]),
        launch_arguments=world_args.items(),
    )

    # ------------------------------------------------------------------ #
    # SLAM Toolbox — online async mapping                                  #
    # ------------------------------------------------------------------ #
    slam_config = {
        "real":  os.path.join(rbsf_slam_toolbox_share, "config", "mapper_params_real.yaml"),
        "mvsim": os.path.join(rbsf_slam_toolbox_share, "config", "mapper_params_sim.yaml"),
    }
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rbsf_slam_toolbox"), "launch", "mapping.launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_config[mode],
            "use_sim_time":     "true" if mode == "mvsim" else "false",
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # Map saver — triggered by gamepad button press                        #
    # ------------------------------------------------------------------ #
    map_saver = Node(
        package="rbsf_slam_toolbox",
        executable="map_saver",
        name="map_saver_node",
        parameters=[{
            "map_output_dir": map_output_dir,
            "save_button": save_button,
        }],
    )

    nodes = [world_bringup, slam, map_saver]

    # ------------------------------------------------------------------ #
    # mvsim only: bridge AckermannDriveStamped → Twist for the simulator  #
    # ------------------------------------------------------------------ #
    if mode == "mvsim":
        nodes.append(Node(
            package="stack_launcher",
            executable="ackermann_to_twist",
            name="ackermann_to_twist",
            parameters=[{"input_topic": "/ackermann_drive"}],
        ))

    # ------------------------------------------------------------------ #
    # Keyboard teleop — opens in its own xterm window                     #
    # ------------------------------------------------------------------ #
    if teleop == "keyboard":
        nodes.append(Node(
            package="stack_launcher",
            executable="keyboard_teleop",
            name="keyboard_teleop",
            prefix="xterm -e",
            parameters=[{
                "output_topic":    "teleop",
                "forward_speed":   max_speed,
                "backward_speed":  max_speed,
            }],
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
        DeclareLaunchArgument(
            "teleop",
            default_value="keyboard",
            choices=["controller", "keyboard"],
            description="Input device: 'controller' (gamepad) or 'keyboard' (WASD / arrow keys)",
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

        # --- real arguments -------------------------------------------- #
        DeclareLaunchArgument(
            "max_speed",
            default_value="1.0",
            description="Maximum longitudinal speed in m/s — caps keyboard teleop commands (all modes) and VESC ERPM limits (mode:=real)",
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
