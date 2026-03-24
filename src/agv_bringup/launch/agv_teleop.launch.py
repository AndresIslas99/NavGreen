"""
AGV Teleop Mode — drive robot from tablet, no SLAM

Launches:
  - robot_state_publisher (URDF/TF)
  - odrive_can_node (motor control + wheel odom)
  - teleop_server (web UI on :8090)

Usage:
  ros2 launch agv_bringup agv_teleop.launch.py
  Then open http://agv.local from tablet
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

        # Teleop web server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_ui_backend'), 'launch', 'teleop_web.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),
    ])
