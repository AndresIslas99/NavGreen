"""
AGV Robot Core — description + motor control only

Launches:
  - robot_state_publisher (URDF/TF)
  - odrive_can_node (CAN motor control + wheel odom)

Does NOT launch web UI (that runs as a systemd service).
Use this when agv-teleop.service is already providing the web UI.

Usage:
  ros2 launch agv_bringup agv_robot_core.launch.py
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
    ])
