"""
AGV Local EKF Test — wheel odom + optional IMU → odom→base_link

Launches:
  - robot_state_publisher (URDF/TF)
  - odrive_can_node (motor control + wheel odom at 50 Hz)
  - ekf_local ONLY (odom→base_link from robot_localization)
  - teleop_server (web UI on :8090)
  - optionally: ZED 2i camera (for IMU data)

TF OWNERSHIP (local EKF test mode):
  odom → base_link:  ekf_local
  map → odom:        NOT PUBLISHED (no global filter)
  cuVSLAM:           NOT RUNNING

This launch validates the local EKF in isolation before activating
the full dual-EKF fusion pipeline.

Usage:
  # Wheel odom only (no camera):
  ros2 launch agv_bringup agv_ekf_local_test.launch.py

  # Wheel odom + ZED IMU:
  ros2 launch agv_bringup agv_ekf_local_test.launch.py enable_zed:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    enable_zed = LaunchConfiguration('enable_zed')

    fusion_dir = get_package_share_directory('agv_sensor_fusion')
    ekf_local_config = os.path.join(fusion_dir, 'config', 'ekf_local.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('enable_zed', default_value='false',
                              description='Launch ZED camera for IMU data'),

        # Robot description (URDF → static TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_description'), 'launch', 'description.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # ODrive motor control + wheel odometry (50 Hz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_odrive'), 'launch', 'odrive.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # Local EKF only: wheel_odom + IMU → odom→base_link
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_local',
                    namespace=ns,
                    parameters=[ekf_local_config],
                    remappings=[
                        ('odometry/filtered', 'odometry/local'),
                    ],
                    output='screen',
                ),
            ],
        ),

        # ZED 2i camera (optional — for IMU data only, no SLAM)
        TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
                        ])),
                    launch_arguments={
                        'camera_model': 'zed2i',
                        'camera_name': 'zed',
                        'publish_tf': 'false',
                        'publish_map_tf': 'false',
                        'publish_imu_tf': 'true',
                    }.items(),
                    condition=IfCondition(enable_zed),
                ),
            ],
        ),

        # Teleop web server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_ui_backend'), 'launch', 'teleop_web.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),
    ])
