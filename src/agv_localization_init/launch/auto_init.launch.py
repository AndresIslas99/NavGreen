"""
Launch the auto_init_orchestrator_node under the AGV namespace.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_localization_init')
    default_params = os.path.join(pkg_dir, 'config', 'auto_init_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='agv',
            description='Robot namespace'),
        DeclareLaunchArgument(
            'map_dir', default_value=os.path.expanduser('~/agv_data/maps'),
            description='Directory where map artifacts live (*.pgm, *_cuvslam/, *_meta.json)'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to parameter YAML'),

        Node(
            package='agv_localization_init',
            executable='auto_init_orchestrator_node',
            name='auto_init_orchestrator',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                LaunchConfiguration('params_file'),
                {'map_dir': LaunchConfiguration('map_dir')},
            ],
            output='screen',
        ),
    ])
