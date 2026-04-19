import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    map_dir = LaunchConfiguration('map_dir')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('map_dir', description='Directory for map storage'),

        Node(
            package='agv_map_manager',
            executable='map_manager_node',
            name='map_manager',
            namespace=ns,
            parameters=[{
                'map_dir': map_dir,
                'map_topic': '/agv/map',
            }],
            output='screen',
        ),
    ])
