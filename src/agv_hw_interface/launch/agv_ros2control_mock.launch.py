"""
agv_hw_interface — agv_ros2control_mock.launch.py

Brings up the ros2_control stack with mock_components/GenericSystem (no
hardware required). This is the entry point for navigation and behaviors
development without the physical AGV.

Usage:
  ros2 launch agv_hw_interface agv_ros2control_mock.launch.py

Verify:
  ros2 control list_controllers
  ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \\
    '{header: {frame_id: ""}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}'
  ros2 topic echo /joint_states
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_hw_interface'),
                    'launch', 'agv_ros2control.launch.py'
                ])),
            launch_arguments={'use_mock_hardware': 'true'}.items(),
        ),
    ])
