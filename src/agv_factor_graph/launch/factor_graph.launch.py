"""
Factor graph launch — sliding window iSAM2 sensor fusion.

Runs in PARALLEL with ekf_global by default (publish_tf:=false).
Set publish_tf:=true after validation to perform the cutover.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_factor_graph')
    config_file = os.path.join(pkg_dir, 'config', 'factor_graph_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='agv',
            description='Robot namespace'),
        DeclareLaunchArgument(
            'publish_tf', default_value='false',
            description='Publish map->odom TF (set true for cutover from ekf_global)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (sim_time) — must match the clock domain of '
                        'the input topics (/agv/wheel_odom, /agv/imu/filtered, etc).'),

        Node(
            package='agv_factor_graph',
            executable='factor_graph_node',
            name='factor_graph',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                config_file,
                {'publish_tf': LaunchConfiguration('publish_tf')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='log',
        ),
    ])
