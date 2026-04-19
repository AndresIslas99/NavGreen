from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # CAN setup is handled by systemd service (can-setup.service)
    # No need for sudo commands here

    odrive_node = Node(
        package='agv_odrive',
        executable='odrive_can_node',
        name='odrive_can_node',
        output='screen',
        parameters=[{
            'can_interface': 'can0',
            'node_id': 0,
            'can_bitrate': 250000,
            'can_retry_interval': 2.0,
            'can_max_retries': 15,
        }],
    )

    gui_node = Node(
        package='agv_odrive',
        executable='odrive_gui',
        name='odrive_gui',
        output='screen',
    )

    return LaunchDescription([
        odrive_node,
        gui_node,
    ])
