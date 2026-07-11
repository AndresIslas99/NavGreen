import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_geometry(pkg_dir):
    """Load the kinematic geometry SSOT.

    Single source of truth declared in
    specs/persistence.yaml#config_artifacts.robot_geometry. The xacro
    `<xacro:arg>` defaults in agv_full.urdf.xacro remain as fallbacks for
    standalone `xacro` testing, but this launch always overrides them at
    runtime with the values below.
    """
    geom_path = os.path.join(pkg_dir, 'config', 'robot_geometry.yaml')
    with open(geom_path, 'r') as f:
        doc = yaml.safe_load(f)
    return doc['/**']['ros__parameters']


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'agv_full.urdf.xacro')
    geom = _load_geometry(pkg_dir)

    # Build the xacro args. Names must match <xacro:arg name="..."/> in
    # agv_full.urdf.xacro. Values come from the SSOT, so the URDF reflects
    # robot_geometry.yaml at every boot — no per-launch drift.
    xacro_args = [
        ' wheel_radius:=', str(geom['wheel_radius']),
        ' left_wheel_y:=', str(geom['left_wheel_y']),
        ' right_wheel_y:=', str(geom['right_wheel_y']),
        ' wheel_z:=', str(geom['wheel_z']),
        ' wheel_width:=', str(geom['wheel_width']),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, *xacro_args]),
        value_type=str)

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='agv',
            description='Robot namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock for time (set true for HIL/sim)'),

        DeclareLaunchArgument(
            'use_joint_state_publisher', default_value='false',
            description='Use joint_state_publisher (set true if no real joint states)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
            output='screen',
        ),
    ])
