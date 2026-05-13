import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_odrive')
    desc_dir = get_package_share_directory('agv_description')

    # Geometry SSOT — declared in specs/persistence.yaml#config_artifacts.robot_geometry.
    # MUST be listed BEFORE odrive_params.yaml so that wheel_radius,
    # track_width and gear_ratio resolve from the SSOT. The geometry keys
    # were deleted from odrive_params.yaml in the same commit; if they ever
    # reappear there they would override the SSOT and silently mask drift.
    geometry_file = os.path.join(desc_dir, 'config', 'robot_geometry.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'odrive_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='agv',
            description='Robot namespace'),

        DeclareLaunchArgument(
            'geometry_file', default_value=geometry_file,
            description='Kinematic geometry SSOT YAML (specs/persistence.yaml#config_artifacts.robot_geometry)'),

        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='ODrive motor-tuning parameter YAML (geometry keys live in geometry_file)'),

        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel',
            description='cmd_vel input topic (use cmd_vel_safe when velocity smoother + collision monitor active)'),

        Node(
            package='agv_odrive',
            executable='odrive_can_node',
            name='agv_odrive_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                LaunchConfiguration('geometry_file'),
                LaunchConfiguration('params_file'),
            ],
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
            output='screen',
        ),
    ])
