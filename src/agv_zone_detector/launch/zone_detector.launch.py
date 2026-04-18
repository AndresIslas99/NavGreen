from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('agv_zone_detector'),
        'config', 'zone_detector_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='agv_zone_detector',
            executable='zone_detector_node',
            name='zone_detector',
            namespace=LaunchConfiguration('namespace'),
            parameters=[params_file,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])
