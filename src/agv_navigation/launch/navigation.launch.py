"""
AGV Navigation Launch — Nav2 stack without localization

Launches Nav2 planning, control, behavior, and costmap nodes.
Does NOT launch AMCL — the dual EKF from agv_sensor_fusion
owns map→odom and odom→base_link.

Launches map_server separately to load the occupancy grid.

Usage (standalone, real hardware):
  ros2 launch agv_navigation navigation.launch.py use_sim_time:=false map:=/path/to/map.yaml

Usage (via HIL):
  ros2 launch agv_bringup agv_hil.launch.py enable_nav2:=true map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _build_nav2(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() in ('true', '1', 'yes')
    map_yaml = LaunchConfiguration('map').perform(context)
    nav2_params_path = LaunchConfiguration('nav2_params').perform(context)
    nav2_override_path = LaunchConfiguration('nav2_params_override').perform(context)
    nav_dir = get_package_share_directory('agv_navigation')
    vel_smoother_params = os.path.join(nav_dir, 'config', 'velocity_smoother.yaml')
    collision_monitor_params = os.path.join(nav_dir, 'config', 'collision_monitor.yaml')

    # Build nav2 params list: base + optional override
    nav2_params_list = [nav2_params_path]
    if nav2_override_path and os.path.isfile(nav2_override_path):
        nav2_params_list.append(nav2_override_path)

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]
    # collision_monitor excluded from lifecycle — it runs independently
    # and must not block the entire Nav2 stack on config errors

    return [GroupAction(
        actions=[
            PushRosNamespace(ns),

            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[
                    nav2_params_path,
                    {'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml},
                ],
                output='screen',
            ),

            # Controller server (RegulatedPurePursuit)
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                parameters=nav2_params_list + [{'use_sim_time': use_sim_time}],
                output='screen',
                remappings=[('cmd_vel', 'cmd_vel')],
            ),

            # Planner server (SmacPlanner2D)
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=nav2_params_list + [{'use_sim_time': use_sim_time}],
                output='screen',
            ),

            # Behavior server (spin, backup, wait)
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=nav2_params_list + [{'use_sim_time': use_sim_time}],
                output='screen',
            ),

            # BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                parameters=nav2_params_list + [{'use_sim_time': use_sim_time}],
                output='screen',
            ),

            # Velocity smoother (cmd_vel → cmd_vel_smoothed)
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                parameters=[vel_smoother_params, {'use_sim_time': use_sim_time}],
                output='screen',
                remappings=[
                    ('cmd_vel', 'cmd_vel'),
                    ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
                ],
            ),

            # Collision monitor (cmd_vel_smoothed → cmd_vel_safe)
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                parameters=[collision_monitor_params, {'use_sim_time': use_sim_time}],
                output='screen',
            ),

            # Lifecycle manager — brings up all Nav2 nodes
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                }],
                output='screen',
            ),
        ],
    )]


def generate_launch_description():
    nav_dir = get_package_share_directory('agv_navigation')
    default_params = os.path.join(nav_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use sim time (true for HIL, false for real)'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML file'),
        DeclareLaunchArgument('nav2_params', default_value=default_params,
                              description='Path to Nav2 base params YAML'),
        DeclareLaunchArgument('nav2_params_override', default_value='',
                              description='Optional override YAML (HIL differences)'),

        OpaqueFunction(function=_build_nav2),
    ])
