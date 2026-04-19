from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='agv',
            description='Robot namespace'),

        DeclareLaunchArgument(
            'port', default_value='8090',
            description='Web server port'),

        Node(
            package='agv_ui_backend',
            executable='teleop_server.py',
            name='teleop_server',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'port': LaunchConfiguration('port')}],
            output='screen',
        ),
    ])
