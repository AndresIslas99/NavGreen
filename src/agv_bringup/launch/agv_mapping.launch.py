"""
AGV Mapping Mode — teleop + SLAM for initial greenhouse mapping

Launches:
  - robot_state_publisher (URDF/TF)
  - odrive_can_node (motor control + wheel odom)
  - agv_slam (cuVSLAM + nvblox, TF publishing ON)
  - teleop_server (web UI on :8090)

cuVSLAM publishes odom→base_link and map→odom directly.
No EKF — cuVSLAM's visual-inertial tracking is sufficient for
careful commissioning speed (0.3-0.5 m/s).

Usage:
  ros2 launch agv_bringup agv_mapping.launch.py
  Then open http://agv.local from tablet
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),

        # Robot description (URDF → TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_description'), 'launch', 'description.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # ODrive motor control + wheel odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_odrive'), 'launch', 'odrive.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # Ground-filtered LaserScan from ZED point cloud
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace=ns,
            parameters=[{
                'min_height': 0.03,
                'max_height': 2.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.003,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 8.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'target_frame': 'base_link',
            }],
            remappings=[
                ('cloud_in', '/agv/zed/point_cloud/cloud_registered'),
                ('scan', 'scan'),
            ],
            output='log',
        ),

        # C++ Image Server (camera + depth MJPEG on port 8091)
        Node(
            package='agv_image_server',
            executable='image_server_node',
            name='image_server',
            namespace=ns,
            parameters=[{
                'port': 8091,
                'camera_topic': '/agv/zed/left/image_rect_color',
                'depth_topic': '/agv/zed/depth/depth_registered',
                'jpeg_quality': 70,
                'max_width': 640,
            }],
            output='log',
        ),

        # Live occupancy grid from scan data
        Node(
            package='agv_scan_mapper',
            executable='scan_grid_mapper_node',
            name='scan_grid_mapper',
            namespace=ns,
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('agv_scan_mapper'), 'config', 'scan_mapper_params.yaml'
                ]),
            ],
            respawn=True,
            respawn_delay=2.0,
            output='log',
        ),

        # SLAM pipeline (cuVSLAM + nvblox) — delayed 3s for TF settle + ZED auto-exposure
        TimerAction(
            period=3.0,
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

        # Operator backend (TypeScript) — delayed 5s for DDS discovery
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='agv_ui_backend',
                    executable='teleop_backend',
                    name='teleop_server',
                    namespace=ns,
                    additional_env={
                        'AGV_PORT': '8090',
                        'AGV_NAMESPACE': 'agv',
                        'AGV_DATA_DIR': '/home/orza/agv_data',
                    },
                    output='log',
                ),
            ],
        ),
    ])
