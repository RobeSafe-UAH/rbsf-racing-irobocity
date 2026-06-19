# MIT License

# Copyright (c) 2025 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    teleop_device = LaunchConfiguration('teleop_device').perform(context)

    joy_teleop_config = LaunchConfiguration('joy_config').perform(context)
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            'enable_color': 'true',
            'enable_depth': 'false',
            'align_depth.enable': 'false',
            'pointcloud.enable': 'false',
            'rgb_camera.color_profile': '424,240,15',
        }.items()
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[vesc_config]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[vesc_config]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[
            vesc_config,
            {
                'speed_max': LaunchConfiguration('speed_max_erpm'),
                'speed_min': LaunchConfiguration('speed_min_erpm'),
            },
        ]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[sensors_config]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_config],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    nodes = [
        realsense_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node,
        vesc_driver_node,
        urg_node,
        ackermann_mux_node,
        static_tf_node,
    ]

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
            parameters=[joy_teleop_config]
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
            'joy_config',
            default_value=os.path.join(
                get_package_share_directory('f1tenth_stack'), 'config', 'joy_teleop.yaml'
            ),
            description='Descriptions for joy and joy_teleop configs'),
        DeclareLaunchArgument(
            'vesc_config',
            default_value=os.path.join(
                get_package_share_directory('f1tenth_stack'), 'config', 'vesc.yaml'
            ),
            description='Descriptions for vesc configs'),
        DeclareLaunchArgument(
            'sensors_config',
            default_value=os.path.join(
                get_package_share_directory('f1tenth_stack'), 'config', 'sensors.yaml'
            ),
            description='Descriptions for sensor configs'),
        DeclareLaunchArgument(
            'mux_config',
            default_value=os.path.join(
                get_package_share_directory('f1tenth_stack'), 'config', 'mux.yaml'
            ),
            description='Descriptions for ackermann mux configs'),
        DeclareLaunchArgument(
            'speed_max_erpm',
            default_value='23250.0',
            description='VESC speed upper limit in ERPM; set via max_speed (m/s) in teleop_mapping.launch.py'),
        DeclareLaunchArgument(
            'speed_min_erpm',
            default_value='-23250.0',
            description='VESC speed lower limit in ERPM; set via max_speed (m/s) in teleop_mapping.launch.py'),
        OpaqueFunction(function=launch_setup),
    ])
