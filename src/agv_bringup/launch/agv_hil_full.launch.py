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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')

    fusion_dir = get_package_share_directory('agv_sensor_fusion')
    nav_dir = get_package_share_directory('agv_navigation')

    # Use production configs as base, override only what differs for HIL
    ekf_local_base = os.path.join(fusion_dir, 'config', 'ekf_local.yaml')
    ekf_global_base = os.path.join(fusion_dir, 'config', 'ekf_global.yaml')
    nav2_params_base = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    nav2_hil_overrides = os.path.join(nav_dir, 'config', 'nav2_hil_overrides.yaml')
    slam_toolbox_config = os.path.join(nav_dir, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('map', description='Path to map YAML file (required)'),
        DeclareLaunchArgument('enable_slam_toolbox', default_value='true',
                              description='Enable SLAM Toolbox for mapping'),

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

        # ── Covariance override (fixes zero-covariance from Gazebo bridge) ──
        # Sim publishes wheel_odom and imu/data with all-zero covariances.
        # This relay reads the raw topics and publishes with realistic covariance
        # on _cov topics that the EKF subscribes to.
        Node(
            package='agv_sensor_fusion',
            executable='covariance_override_node',
            name='covariance_override',
            namespace=ns,
            parameters=[{
                'use_sim_time': True,
                'odom_pos_cov_xy': 0.001,
                'odom_pos_cov_yaw': 0.01,
                'odom_twist_cov_xy': 0.001,
                'odom_twist_cov_yaw': 0.01,
                'vslam_pos_cov_xy': 0.01,
                'vslam_pos_cov_yaw': 0.05,
                'imu_orient_cov': 0.001,
                'imu_gyro_cov': 0.0005,
                'imu_accel_cov': 0.01,
            }],
            remappings=[
                ('wheel_odom_raw', 'wheel_odom'),        # reads sim odom
                ('wheel_odom', 'wheel_odom_cov'),        # publishes with covariance
                ('imu_raw', 'imu/data'),                 # reads sim IMU
                ('imu/data', 'imu/data_cov'),            # publishes with covariance
            ],
            output='log',
        ),

        # ── Local EKF: sim wheel_odom + sim IMU → odom→base_link ──
        # Base config + HIL overrides:
        #   frequency: 20Hz (sim rates ~11Hz odom, ~17Hz IMU — 40Hz was too high)
        #   imu0: /agv/imu/data (sim IMU, not /zed/zed_node/imu/data)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            namespace=ns,
            parameters=[ekf_local_base, {
                'use_sim_time': True,
                'frequency': 50.0,               # high freq so EKF processes data on arrival, not timer-bound
                'odom0': 'wheel_odom_cov',        # from covariance_override relay
                'imu0': '/agv/imu/data_cov',      # from covariance_override relay
            }],
            remappings=[('odometry/filtered', 'odometry/local')],
            output='log',
        ),

        # ── Global EKF: local + sim visual odom → map→odom ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            namespace=ns,
            parameters=[ekf_global_base, {
                'use_sim_time': True,
                'frequency': 20.0,
                'odom1': '/visual_slam/tracking/odometry_cov',
            }],
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
                'nav2_params': nav2_params_base,
                'nav2_params_override': nav2_hil_overrides,
            }.items(),
        ),

        # ── Scan grid mapper (live occupancy grid for commissioning) ──
        Node(
            package='agv_scan_mapper',
            executable='scan_grid_mapper_node',
            name='scan_grid_mapper',
            namespace=ns,
            parameters=[{
                'resolution': 0.05,
                'width': 400,
                'height': 400,
                'origin_x': -10.0,
                'origin_y': -10.0,
                'publish_rate_hz': 1.0,
                'map_frame': 'map',
                'use_sim_time': True,
            }],
            output='log',
        ),

        # ── SLAM Toolbox (loop-closed occupancy grid for commissioning) ──
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            parameters=[slam_toolbox_config, {'use_sim_time': True}],
            output='log',
            condition=IfCondition(LaunchConfiguration('enable_slam_toolbox')),
        ),

        # ── AprilTag detection + marker correction (drift correction) ──
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            namespace=ns,
            parameters=[{
                'family': '36h11',
                'size': 0.2,
                'max_hamming': 0,
                'detector.threads': 2,
                'detector.quad_decimate': 2.0,
                'use_sim_time': True,
            }],
            remappings=[
                ('image_rect', '/zed/zed_node/left/image_rect_color'),
                ('camera_info', '/zed/zed_node/left/camera_info'),
            ],
            output='log',
        ),

        Node(
            package='agv_markers',
            executable='marker_correction_node',
            name='marker_correction',
            namespace=ns,
            parameters=[{
                'markers_registry_file': os.path.join(
                    get_package_share_directory('agv_markers'), 'config', 'markers_registry.yaml'),
                'max_detection_range': 5.0,
                'tag_size': 0.2,
                'covariance_xy': 0.01,
                'covariance_yaw': 0.03,
                'relocalization_threshold': 1.5,
                'min_confidence': 50.0,
                'use_sim_time': True,
            }],
            output='log',
        ),

        # ── C++ Image Server (camera + depth MJPEG on port 8091) ──
        Node(
            package='agv_image_server',
            executable='image_server_node',
            name='image_server',
            namespace=ns,
            parameters=[{
                'port': 8091,
                'camera_topic': '/zed/zed_node/right/image_rect_color',
                'depth_topic': '/zed/zed_node/depth/depth_registered',
                'jpeg_quality': 70,
                'max_width': 640,
                'use_sim_time': True,
            }],
            output='log',
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
