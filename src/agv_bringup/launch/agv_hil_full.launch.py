"""
AGV HIL Full Stack — Single-command launch for HIL simulation mode

Launches the complete Jetson autonomy brain against simulated sensors from PC:
  - robot_state_publisher (URDF → static TF)
  - ekf_local  (sim wheel_odom + sim IMU → odom→base_link, 40 Hz)
  - ekf_global (local + sim visual odom → map→odom, 10 Hz)
  - Nav2 stack (planner, controller, costmaps, lifecycle)
  - Operator backend (dashboard + teleop + REST + WebSocket on :8090)

Does NOT launch (provided by PC sim):
  - agv_odrive (no CAN hardware)
  - agv_slam / cuVSLAM (no ZED GPU pipeline)

External topics expected from PC:
  /clock                    rosgraph_msgs/Clock      sim time
  /agv/wheel_odom           nav_msgs/Odometry        50 Hz  (odom → base_link)
  /agv/joint_states         sensor_msgs/JointState   50 Hz
  /agv/imu/data             sensor_msgs/Imu          100+ Hz (frame: imu_link)
  /agv/scan                 sensor_msgs/LaserScan    10 Hz  (frame: laser_frame)
  /visual_slam/tracking/odometry  nav_msgs/Odometry  10 Hz  (map → base_link)

PC must subscribe to /agv/cmd_vel to close the control loop.

Usage:
  ros2 launch agv_bringup agv_hil_full.launch.py \\
    map:=/path/to/map.yaml

  # Or with CycloneDDS for cross-machine discovery:
  export CYCLONEDDS_URI=$(ros2 pkg prefix agv_bringup)/share/agv_bringup/config/cyclonedds_hil.xml
  ros2 launch agv_bringup agv_hil_full.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')

    fusion_dir = get_package_share_directory('agv_sensor_fusion')
    nav_dir = get_package_share_directory('agv_navigation')

    ekf_local_hil = os.path.join(fusion_dir, 'config', 'ekf_local_hil.yaml')
    ekf_global_hil = os.path.join(fusion_dir, 'config', 'ekf_global_hil.yaml')
    nav2_params_hil = os.path.join(nav_dir, 'config', 'nav2_params_hil.yaml')

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
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true',
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
            remappings=[('odometry/filtered', 'odometry/local')],
            output='log',
        ),

        # ── Global EKF: local + sim visual odom → map→odom ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            namespace=ns,
            parameters=[ekf_global_hil, {'use_sim_time': True}],
            remappings=[('odometry/filtered', 'odometry/global')],
            output='log',
        ),

        # ── /agv/scan ──
        # In HIL mode, the sim provides /agv/scan already filtered
        # (pointcloud_to_laserscan with min_height/max_height runs on the sim PC,
        # same pipeline as production on the Jetson).
        # In production (agv_full.launch.py), pointcloud_to_laserscan runs locally.

        # ── Nav2 stack ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_navigation'), 'launch', 'navigation.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true',
                'map': map_yaml,
                'nav2_params': nav2_params_hil,
            }.items(),
        ),

        # ── Operator backend (dashboard + teleop + REST + WS) ──
        Node(
            package='agv_ui_backend',
            executable='teleop_server.py',
            name='teleop_server',
            namespace=ns,
            parameters=[{
                'port': 8090,
                'use_sim_time': True,
            }],
            output='log',
        ),
    ])
