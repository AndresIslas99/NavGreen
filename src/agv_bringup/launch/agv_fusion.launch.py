"""
AGV Fusion Mode — teleop + SLAM + dual EKF

Launches:
  - robot_state_publisher (URDF/TF)
  - odrive_can_node (motor control + wheel odom)
  - agv_slam (cuVSLAM, TF publishing DISABLED via cuvslam_no_tf.yaml)
  - dual EKF (local: odom→base_link, global: map→odom)
  - teleop_server (web UI on :8090)

TF OWNERSHIP (fusion mode):
  odom → base_link:  ekf_local   (from robot_localization)
  map → odom:        ekf_global  (from robot_localization)
  cuVSLAM:           publishes /visual_slam/tracking/odometry topic ONLY
                     TF publishing is DISABLED via cuvslam_no_tf.yaml

See docs/dual_ekf_validation.md for the full TF authority table across all launch modes.

Usage:
  ros2 launch agv_bringup agv_fusion.launch.py
  Then open http://agv.local from tablet
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    bringup_dir = get_package_share_directory('agv_bringup')

    # Path to cuVSLAM TF override (disables odom→base_link and map→odom from cuVSLAM)
    cuvslam_no_tf = os.path.join(bringup_dir, 'config', 'cuvslam_no_tf.yaml')

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

        # SLAM pipeline (TF DISABLED — EKF owns all transforms)
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
                        'slam_params_override': cuvslam_no_tf,
                    }.items(),
                ),
            ],
        ),

        # Dual EKF sensor fusion (t=4s: wait for SLAM + wheel_odom to be publishing)
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
