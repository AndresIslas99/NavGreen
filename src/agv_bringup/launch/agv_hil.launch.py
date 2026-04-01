"""
AGV HIL Mode — Jetson autonomy brain against simulated sensors from PC

The PC runs the simulated world and publishes sensor topics.
The Jetson runs the real autonomy stack consuming those topics.

Launches (on Jetson):
  - robot_state_publisher (URDF → static TF)
  - ekf_local  (sim wheel_odom + sim IMU → odom→base_link)
  - ekf_global (local + sim visual odom → map→odom)
  - teleop_server (optional web UI)
  - Nav2 stack (optional, gated by enable_nav2)

Does NOT launch:
  - agv_odrive (no CAN hardware)
  - agv_slam / cuVSLAM (no ZED GPU pipeline)
  - ZED camera driver

External topics expected from PC:
  /clock                    rosgraph_msgs/Clock      sim time
  /agv/wheel_odom           nav_msgs/Odometry        50 Hz  (odom → base_link)
  /agv/joint_states         sensor_msgs/JointState   50 Hz
  /agv/imu/data             sensor_msgs/Imu          100+ Hz (frame: imu_link)
  /agv/sim_odom             nav_msgs/Odometry        10 Hz  (map → base_link)
  /agv/scan                 sensor_msgs/LaserScan    10 Hz  (frame: laser_frame)

PC must subscribe to /agv/cmd_vel to close the control loop.

TF OWNERSHIP (HIL mode — same as fusion mode):
  odom → base_link:  ekf_local
  map → odom:        ekf_global

Networking:
  Both machines must share ROS_DOMAIN_ID and RMW_IMPLEMENTATION.
  Set CYCLONEDDS_URI to cyclonedds_hil.xml for cross-machine discovery.

Usage:
  export CYCLONEDDS_URI=$(ros2 pkg prefix agv_bringup)/share/agv_bringup/config/cyclonedds_hil.xml
  ros2 launch agv_bringup agv_hil.launch.py

  # With Nav2 (Phase 2):
  ros2 launch agv_bringup agv_hil.launch.py enable_nav2:=true map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_teleop = LaunchConfiguration('enable_teleop')
    map_yaml = LaunchConfiguration('map')

    fusion_dir = get_package_share_directory('agv_sensor_fusion')
    ekf_local_hil = os.path.join(fusion_dir, 'config', 'ekf_local_hil.yaml')
    ekf_global_hil = os.path.join(fusion_dir, 'config', 'ekf_global_hil.yaml')

    nav_dir = get_package_share_directory('agv_navigation')
    nav2_params_hil = os.path.join(nav_dir, 'config', 'nav2_params_hil.yaml')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from PC sim (should always be true for HIL)'),
        DeclareLaunchArgument('enable_nav2', default_value='false',
                              description='Launch Nav2 stack (Phase 2)'),
        DeclareLaunchArgument('enable_teleop', default_value='true',
                              description='Launch teleop web UI'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML for Nav2 (required if enable_nav2:=true)'),

        # ── Robot description (URDF → static TF) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_description'), 'launch', 'description.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': use_sim_time,
                'use_joint_state_publisher': 'false',
            }.items(),
        ),

        # ── Local EKF: sim wheel_odom + sim IMU → odom→base_link ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            namespace=ns,
            parameters=[ekf_local_hil, {'use_sim_time': True}],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
            ],
            output='screen',
        ),

        # ── Global EKF: local + sim visual odom → map→odom ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            namespace=ns,
            parameters=[ekf_global_hil, {'use_sim_time': True}],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
            ],
            output='screen',
        ),

        # ── Teleop web server (optional) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_ui_backend'), 'launch', 'teleop_web.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
            condition=IfCondition(enable_teleop),
        ),

        # ── Nav2 stack (Phase 2, gated) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_navigation'), 'launch', 'navigation.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'nav2_params': nav2_params_hil,
            }.items(),
            condition=IfCondition(enable_nav2),
        ),
    ])
