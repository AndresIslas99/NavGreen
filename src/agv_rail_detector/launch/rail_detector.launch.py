from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('agv_rail_detector'),
        'config', 'rail_detector_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='agv_rail_detector',
            executable='rail_detector_node',
            name='rail_detector',
            namespace=LaunchConfiguration('namespace'),
            parameters=[params_file,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='log',
        ),
    ])
