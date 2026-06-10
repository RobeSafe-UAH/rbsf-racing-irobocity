from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
        Launch file description: Orchest a bunch of nodes and its configuration.
        Import from 'general launch' from ROS2 and adapted due the project 
        'Diseño de un robot autónomo de carreras con ROS2'.
    """
      
    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization",
        default_value='False',
        description='publish fake identity tf "map" -> "odom"'
    )

    publish_tf_odom2baselink_arg = DeclareLaunchArgument(
        "publish_tf_odom2baselink",
        default_value='True',
        description='publish tf "odom" -> "base_link"'
    )

    force_publish_vehicle_namespace_arg = DeclareLaunchArgument(
        "force_publish_vehicle_namespace",
        default_value='True',
        description='Use vehicle name namespace even if there is only one vehicle'
    )

    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value="0.1",
        description='Minimun working distance of LiDAR'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value="30.0",
        description='Maximum working distance of LiDAR'
    )

    laser_topic_arg = DeclareLaunchArgument(
        'laser_topic',
        default_value="/scan",
        description='Topic where the detector extract the LiDAR information'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("rbsf_mvsim"),
                                   'config', 'mapper_params_real.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    centerline_csv_arg = DeclareLaunchArgument(
        'centerline_csv',
        default_value='',
        description='Absolute path to a centerline CSV file. When set, the centerline '
                    'publisher node is launched and publishes nav_msgs/Path on /centerline.'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
                LaunchConfiguration('slam_params_file'),
                {"use_sim_time": LaunchConfiguration('use_sim_time')}
            ]
    )

    centerline_publisher_node = Node(
        package='rbsf_slam_toolbox',
        executable='centerline_publisher',
        name='centerline_publisher',
        output='screen',
        parameters=[{'csv_path': LaunchConfiguration('centerline_csv')}],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('centerline_csv'), "' != ''"])
        ),
    )

    return LaunchDescription([
        do_fake_localization_arg,
        publish_tf_odom2baselink_arg,
        force_publish_vehicle_namespace_arg,
        max_range_arg,
        min_range_arg,
        laser_topic_arg,
        slam_params_file_arg,
        centerline_csv_arg,
        slam_toolbox_node,
        centerline_publisher_node,
    ])