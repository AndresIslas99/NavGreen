"""
AGV Full Stack — Single-command launch for real hardware

Launches the complete autonomy stack on real hardware:
  - robot_state_publisher (URDF → static TF)
  - odrive_can_node (motor control + wheel odom at 50 Hz)
  - agv_slam (cuVSLAM, TF DISABLED via cuvslam_no_tf.yaml) — delayed 2s
  - dual EKF (local: odom→base_link, global: map→odom) — delayed 4s
  - Nav2 stack (planner, controller, costmaps, lifecycle) — delayed 6s
  - Operator backend (dashboard + teleop + REST + WebSocket on :8090)

TF OWNERSHIP:
  odom → base_link:  ekf_local
  map → odom:        ekf_global
  cuVSLAM:           topic-only (/visual_slam/tracking/odometry)

Usage:
  ros2 launch agv_bringup agv_full.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')

    bringup_dir = get_package_share_directory('agv_bringup')
    cuvslam_no_tf = os.path.join(bringup_dir, 'config', 'cuvslam_no_tf.yaml')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('map', description='Path to map YAML file (required)'),

        # ── Robot description (URDF → static TF) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_description'), 'launch', 'description.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # ── ODrive motor control (immediate) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_odrive'), 'launch', 'odrive.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # ── SLAM pipeline (t=2s, TF DISABLED — EKF owns transforms) ──
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

        # ── Dual EKF sensor fusion (t=4s, waits for SLAM + wheel_odom) ──
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

        # ── Ground-filtered LaserScan from ZED point cloud ──
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace=ns,
            parameters=[{
                'min_height': 0.05,
                'max_height': 1.20,
                'angle_min': -1.0472,
                'angle_max': 1.0472,
                'angle_increment': 0.005,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'target_frame': 'base_link',
            }],
            remappings=[
                ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
                ('scan', 'scan'),
            ],
            output='log',
        ),

        # ── Nav2 stack (t=6s, waits for EKF to produce TF) ──
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agv_navigation'), 'launch', 'navigation.launch.py'
                        ])),
                    launch_arguments={
                        'namespace': ns,
                        'use_sim_time': 'false',
                        'map': map_yaml,
                    }.items(),
                ),
            ],
        ),

        # ── Operator backend (immediate — shows "connecting" until stack is up) ──
        Node(
            package='agv_ui_backend',
            executable='teleop_server.py',
            name='teleop_server',
            namespace=ns,
            parameters=[{'port': 8090}],
            output='log',
        ),
    ])
