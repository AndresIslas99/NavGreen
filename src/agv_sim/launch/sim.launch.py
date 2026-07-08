"""
agv_sim -- sim.launch.py

Hardware-free Gazebo Classic simulation of the AGV. Lets a contributor with no
robot spawn the AGV, drive it (publish geometry_msgs/Twist on /cmd_vel) and
watch odometry (/odom, /joint_states, /tf). Sensor-free drivetrain only:
Gazebo provides the physics and the two-wheel differential drive; there are NO
cameras or lidar, so headless gzserver needs no GPU and runs in CI.

Targets ROS 2 Humble + Gazebo Classic (gazebo_ros / gazebo_ros2_control).

What it starts:
  - gzserver (always) via gazebo_ros/gzserver.launch.py, loading the world.
    gzserver.launch.py adds the gazebo_ros init (/clock) and factory
    (/spawn_entity) system plugins by default.
  - gzclient (only if gui:=true) via gazebo_ros/gzclient.launch.py.
  - robot_state_publisher fed by the xacro-expanded agv_sim.urdf.xacro.
  - spawn_entity.py, spawning the robot from the /robot_description topic.
  - joint_state_broadcaster spawner, then diff_drive_controller spawner
    (chained so they start only after the model -- and thus the
    gazebo_ros2_control controller_manager -- exists).
  - RViz2 (only if rviz:=true).

Launch args (all optional):
  gui:=false     Headless gzserver only (default, CI-friendly). true also
                 starts gzclient.
  world:=<path>  World file. Default: this package's worlds/greenhouse.world.
  rviz:=false    Start RViz2 when true.
  x, y, yaw      Robot spawn pose (default 0.0, 0.0, 0.0).

Examples:
  ros2 launch agv_sim sim.launch.py                        # headless (CI)
  ros2 launch agv_sim sim.launch.py gui:=true rviz:=true   # interactive
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_sim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(pkg_share, 'worlds', 'greenhouse.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'agv_sim.urdf.xacro')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str),
        'use_sim_time': True,
    }

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'agv',
            '-x', x, '-y', y, '-Y', yaw,
            # base_link sits 0.20 m above base_footprint (ground), so spawning
            # base_link at z=0.20 rests the wheels exactly on the ground.
            '-z', '0.2',
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            # Newcomer-friendly, un-namespaced topics: the controller consumes
            # /cmd_vel (geometry_msgs/Twist, use_stamped_vel=false) and
            # publishes /odom (nav_msgs/Odometry). TF (odom->base_link) already
            # goes to /tf because enable_odom_tf is true in sim_controllers.yaml.
            '--controller-ros-args',
            '-r /diff_drive_controller/cmd_vel_unstamped:=/cmd_vel '
            '-r /diff_drive_controller/odom:=/odom',
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Start gzclient GUI (headless if false)'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world file'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start RViz2'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        rviz_node,

        # Spawn controllers only after the entity (and thus the
        # gazebo_ros2_control controller_manager) exists, then chain
        # diff_drive after joint_state_broadcaster.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),
    ])
