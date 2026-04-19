import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_odrive')
    params_file = os.path.join(pkg_dir, 'config', 'odrive_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='agv',
            description='Robot namespace'),

        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='Path to parameter YAML file'),

        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel',
            description='cmd_vel input topic (use cmd_vel_safe when velocity smoother + collision monitor active)'),

        Node(
            package='agv_odrive',
            executable='odrive_can_node',
            name='agv_odrive_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
            output='screen',
        ),
    ])
