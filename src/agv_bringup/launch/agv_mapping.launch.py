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

        # SLAM pipeline (cuVSLAM + nvblox)
        # Delayed 2s to let TF settle
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

        # Teleop web server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_ui_backend'), 'launch', 'teleop_web.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),
    ])
