"""
Dual EKF Sensor Fusion Launch

Launches local and global EKF filters from robot_localization.

IMPORTANT: When running this alongside agv_slam, cuVSLAM must have
its TF publishing DISABLED to avoid duplicate transforms:
  - publish_odom_to_base_tf: false
  - publish_map_to_odom_tf: false

cuVSLAM still publishes /visual_slam/tracking/odometry as a topic,
which the global EKF reads.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_sensor_fusion')
    ekf_local_config = os.path.join(pkg_dir, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_dir, 'config', 'ekf_global.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='agv',
            description='Robot namespace'),

        # ── Local EKF: wheel_odom + IMU → odom→base_link ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            namespace=LaunchConfiguration('namespace'),
            parameters=[ekf_local_config],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
            ],
            output='screen',
        ),

        # ── Global EKF: local + cuVSLAM → map→odom ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            namespace=LaunchConfiguration('namespace'),
            parameters=[ekf_global_config],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
            ],
            output='screen',
        ),

        # ── Fusion monitor: /agv/pose publisher + localization diagnostics ──
        Node(
            package='agv_sensor_fusion',
            executable='fusion_monitor_node',
            name='fusion_monitor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'pose_rate_hz': 10.0,
                'covariance_warn_threshold': 0.5,
                'covariance_error_threshold': 2.0,
                'stale_timeout_s': 2.0,
            }],
            output='screen',
        ),
    ])
