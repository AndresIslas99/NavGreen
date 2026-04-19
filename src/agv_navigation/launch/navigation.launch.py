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
    hil_mode_str = LaunchConfiguration('hil_mode').perform(context)
    hil_mode = hil_mode_str.lower() in ('true', '1', 'yes')
    map_yaml = LaunchConfiguration('map').perform(context)
    nav2_params_path = LaunchConfiguration('nav2_params').perform(context)
    nav2_override_path = LaunchConfiguration('nav2_params_override').perform(context)
    # Phase 2 routes Nav2's cmd_vel through agv_mode_arbiter, which selects
    # between Nav2, rail_approach, and rail_driver. The arbiter publishes the
    # final /agv/cmd_vel. In production (no arbiter), keep the legacy direct
    # path where controller_server publishes straight to cmd_vel.
    controller_cmd_vel_topic = LaunchConfiguration(
        'controller_cmd_vel_topic').perform(context)
    nav_dir = get_package_share_directory('agv_navigation')
    vel_smoother_params = os.path.join(nav_dir, 'config', 'velocity_smoother.yaml')
    collision_monitor_params = os.path.join(nav_dir, 'config', 'collision_monitor.yaml')
    # In HIL, drop collision_monitor's pointcloud_source (the 3D defense-in-depth
    # layer). The sim PC does not publish /agv/zed/point_cloud over the network
    # without saturating WiFi (~180 Mbps), and even if we accepted the cost the
    # Jetson already drops the frames. scan_source alone is sufficient for HIL
    # validation — the 3D coverage is a production-only feature requiring the
    # local ZED IPC.
    collision_monitor_hil_override = os.path.join(
        nav_dir, 'config', 'collision_monitor_hil_overrides.yaml')
    collision_monitor_params_list = [collision_monitor_params]
    if hil_mode and os.path.isfile(collision_monitor_hil_override):
        collision_monitor_params_list.append(collision_monitor_hil_override)
    # Custom forward-only BT — see behavior_trees/navigate_to_pose_forward_only.xml
    # for the rationale (no rear sensor → no BackUp recovery action). Injected
    # here as an absolute path because YAML param files don't expand
    # $(find-pkg-share ...) substitutions.
    forward_only_bt_xml = os.path.join(
        nav_dir, 'behavior_trees', 'navigate_to_pose_forward_only.xml')

    # Build nav2 params list: base + optional override
    nav2_params_list = [nav2_params_path]
    if nav2_override_path and os.path.isfile(nav2_override_path):
        nav2_params_list.append(nav2_override_path)

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'collision_monitor',
    ]
    # collision_monitor IS a lifecycle node (nav2_util::LifecycleNode). It must
    # be activated by the lifecycle_manager — otherwise on_activate() never runs,
    # the cmd_vel_smoothed subscriber and cmd_vel_safe publisher are never
    # created, and the entire Nav2 → odrive cmd_vel chain is broken in nav mode.

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
                remappings=[('cmd_vel', controller_cmd_vel_topic)],
            ),

            # Planner server (SmacPlanner2D)
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=nav2_params_list + [{'use_sim_time': use_sim_time}],
                output='screen',
            ),

            # Smoother server (post-plan SimpleSmoother)
            # Accepts the raw plan from planner_server and produces a
            # curvature-continuous smoothed path before it reaches MPPI.
            # Invoked from the BT via SmoothPath action.
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
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
                parameters=nav2_params_list + [
                    {'use_sim_time': use_sim_time},
                    {'default_nav_to_pose_bt_xml': forward_only_bt_xml},
                ],
                output='screen',
            ),

            # Velocity smoother — input is always `cmd_vel` (the post-arbiter
            # topic). Phase 2 keeps this unchanged; only controller_server's
            # OUTPUT was rerouted through the arbiter.
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
            # In HIL, the list includes collision_monitor_hil_overrides.yaml
            # (drops pointcloud_source); in production it is only the base
            # config (both scan_source and pointcloud_source active).
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                parameters=collision_monitor_params_list + [{'use_sim_time': use_sim_time}],
                output='screen',
            ),

            # Lifecycle manager — brings up all Nav2 nodes.
            # Iter-17: under Round-44 HIL load (sim host + brain + sim-shim
            # + pytest harness + multiple rclpy consumers all contending
            # for the Jetson's CPU), the default Nav2 lifecycle_manager
            # service-call timeout (~3 s) consistently fires before
            # collision_monitor / velocity_smoother / controller_server
            # return from their Configuring callback. The result is
            # "Failed to change state for node X" followed by
            # "Failed to bring up all requested nodes. Aborting bringup" —
            # Nav2 never reaches `active`, cmd_vel_nav stays silent, and
            # every nav2-dispatch waypoint stalls.
            #
            # Increasing `bond_timeout` relaxes the bond heartbeat window,
            # letting nodes with slow initial tick survive; attempting
            # respawn reconnection tolerates individual node restarts.
            # 20 s matches the upper-bound configure time observed for
            # controller_server under cold cache + costmap allocation.
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                    'bond_timeout': 20.0,
                    'attempt_respawn_reconnection': True,
                    'bond_respawn_max_duration': 20.0,
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
        DeclareLaunchArgument(
            'hil_mode', default_value='false',
            description=(
                'HIL mode: layer collision_monitor_hil_overrides.yaml on top '
                'of the base collision_monitor config (drops pointcloud_source).'
            ),
        ),
        DeclareLaunchArgument(
            'controller_cmd_vel_topic', default_value='cmd_vel',
            description=(
                'Topic that controller_server publishes velocity to. Default '
                "'cmd_vel' preserves the legacy direct path. Phase 2 routes "
                "this through agv_mode_arbiter via 'cmd_vel_nav'."
            ),
        ),

        OpaqueFunction(function=_build_nav2),
    ])
