from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('agv_rail_driver'),
        'config', 'rail_driver_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='agv_rail_driver',
            executable='rail_driver_node',
            name='rail_driver',
            namespace=LaunchConfiguration('namespace'),
            parameters=[params_file,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])
