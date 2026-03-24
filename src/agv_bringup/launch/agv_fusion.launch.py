"""
AGV Fusion Mode — teleop + SLAM + dual EKF validation

Launches:
  - robot_state_publisher (URDF/TF)
  - odrive_can_node (motor control + wheel odom)
  - agv_slam (cuVSLAM, TF publishing DISABLED)
  - dual EKF (local: odom→base_link, global: map→odom)
  - teleop_server (web UI on :8090)

IMPORTANT: cuVSLAM TF is disabled because the EKF nodes own the transforms.
cuVSLAM still publishes /visual_slam/tracking/odometry as a topic.

Usage:
  ros2 launch agv_bringup agv_fusion.launch.py
  Then open http://agv.local from tablet
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetParametersFromFile
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    bringup_dir = get_package_share_directory('agv_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_description'), 'launch', 'description.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # ODrive motor control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_odrive'), 'launch', 'odrive.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # SLAM pipeline (TF disabled — EKF owns transforms)
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agv_slam'), 'launch', 'agv_slam.launch.py'
                        ])),
                    launch_arguments={
                        'enable_foxglove': 'false',
                        'enable_gui': 'false',
                    }.items(),
                ),
            ],
        ),

        # Dual EKF sensor fusion
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agv_sensor_fusion'), 'launch', 'fusion.launch.py'
                        ])),
                    launch_arguments={'namespace': ns}.items(),
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
