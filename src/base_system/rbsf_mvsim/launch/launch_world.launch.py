# Generic ROS2 launch file
# Read: https://mvsimulator.readthedocs.io/en/latest/mvsim_node.html

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
import os

from ament_index_python.packages import get_package_share_directory
from rbsf_mvsim.world_generator import build_configured_world_file


def launch_setup(context, *args, **kwargs):
    teleop_device = LaunchConfiguration('teleop_device').perform(context)
    world_file = LaunchConfiguration("world_file").perform(context)
    vehicle_config_file = LaunchConfiguration("vehicle_config_file").perform(context)
    world_config_file = LaunchConfiguration("world_config_file").perform(context)
    init_pose = LaunchConfiguration("init_pose").perform(context)

    configured_world_file = build_configured_world_file(
        world_file,
        vehicle_config_file,
        world_config_file,
        init_pose,
    )

    joy_teleop_config = os.path.join(
        get_package_share_directory('rbsf_mvsim'),
        'config',
        'joy_teleop.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('rbsf_mvsim'),
        'config',
        'mux.yaml'
    )

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[{
            "world_file": configured_world_file,
            "headless": LaunchConfiguration('headless'),
            "do_fake_localization": LaunchConfiguration('do_fake_localization'),
            "publish_tf_odom2baselink": LaunchConfiguration('publish_tf_odom2baselink'),
            "force_publish_vehicle_namespace": LaunchConfiguration('force_publish_vehicle_namespace'),
        }]
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_config],
        remappings=[('ackermann_cmd', 'ackermann_drive')],
    )

    nodes = [mvsim_node, ackermann_mux_node]

    if teleop_device == 'controller':
        nodes.append(Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[joy_teleop_config]
        ))
        nodes.append(Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[joy_teleop_config],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop_device',
            default_value='controller',
            description="Teleop input device: 'controller' (gamepad) or 'keyboard'",
            choices=['controller', 'keyboard'],
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=os.path.join(
                get_package_share_directory('rbsf_mvsim'),
                'maps',
                'f1tenth_catalunya.world.xml'
            ),
            description='Path to the *.world.xml file to load',
        ),
        DeclareLaunchArgument(
            "vehicle_config_file",
            default_value=os.path.join(
                get_package_share_directory('rbsf_mvsim'),
                'config',
                'mvsim_vehicle.yaml'
            ),
            description='Path to the YAML file that defines the MVSim vehicle model',
        ),
        DeclareLaunchArgument(
            "world_config_file",
            default_value=os.path.join(
                get_package_share_directory('rbsf_mvsim'),
                'config',
                'mvsim_worlds.yaml'
            ),
            description='Path to the YAML file that defines world-specific parameters',
        ),
        DeclareLaunchArgument(
            "init_pose",
            default_value="",
            description='Vehicle initial pose override: x y yaw_deg',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(
                get_package_share_directory('rbsf_mvsim'),
                'rviz',
                'traxxas_sim.rviz'
            ),
            description='Path to the configuration file for rviz',
        ),
        DeclareLaunchArgument("headless", default_value="False"),
        DeclareLaunchArgument(
            "do_fake_localization",
            default_value="True",
            description='publish fake identity tf "map" -> "odom"',
        ),
        DeclareLaunchArgument(
            "publish_tf_odom2baselink",
            default_value="True",
            description='publish tf "odom" -> "base_link"',
        ),
        DeclareLaunchArgument(
            "force_publish_vehicle_namespace",
            default_value="False",
            description="Use vehicle name namespace even if there is only one vehicle",
        ),
        DeclareLaunchArgument('use_rviz', default_value='True', description='Whether to launch RViz2'),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[] if LaunchConfiguration('rviz_config_file') == '' else [
                '-d', LaunchConfiguration('rviz_config_file')]
        ),
        OpaqueFunction(function=launch_setup),
    ])
