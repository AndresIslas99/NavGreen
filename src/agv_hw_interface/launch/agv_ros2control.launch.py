"""
agv_hw_interface — agv_ros2control.launch.py

Brings up the ros2_control stack against REAL ODrive CAN hardware:
  - controller_manager loads AgvDiffDriveSystem (the plugin in this package)
  - joint_state_broadcaster
  - diff_drive_controller (subscribes to /agv/diff_drive_controller/cmd_vel,
    publishes /agv/diff_drive_controller/odom)

This is opt-in. The legacy standalone agv_odrive node is unchanged and
continues to be the production motor driver until this flow is field-validated
on the real robot.

For development without hardware, use agv_ros2control_mock.launch.py instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_hw_interface')
    controllers_yaml = os.path.join(pkg_share, 'config', 'agv_controllers.yaml')

    use_mock = LaunchConfiguration('use_mock_hardware')
    can_interface = LaunchConfiguration('can_interface')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('agv_hw_interface'),
            'urdf', 'agv_ros2control.urdf.xacro'
        ]),
        ' use_mock_hardware:=', use_mock,
        ' can_interface:=', can_interface,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('can_interface', default_value='can0'),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controllers_yaml],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='log',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
