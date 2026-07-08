"""
agv_safety — Safety supervisor + cmd_vel gate.

Wired into agv_full.launch.py (included at t=6.5 s, only when a map is
available — mapping-first mode bypasses the safety chain). Can also be
brought up in isolation:
  ros2 launch agv_safety safety.launch.py

Topology:
  /agv/cmd_vel_collision_safe (Twist, from collision_monitor)
       -> cmd_vel_gate
       -> /agv/cmd_vel_safe (Twist, consumed by odrive_can_node)

  safety_supervisor monitors a configurable list of critical topics and
  publishes /agv/safety/status (SafetyStatus). cmd_vel_gate consumes that
  status plus /agv/hardware_estop (Bool) and forces zero output when either
  is unsafe.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = os.path.join(
        get_package_share_directory('agv_safety'), 'config', 'safety_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (sim_time) for the supervisor watchdog and '
                        'gate timeout. Must match the clock domain of the monitored '
                        'topics (/agv/wheel_odom, /agv/odometry/global, /agv/scan).'),

        Node(
            package='agv_safety',
            executable='safety_supervisor_node',
            name='safety_supervisor',
            namespace=ns,
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('software_estop', 'software_estop'),
                ('~/status', 'safety/status'),
            ],
            output='log',
        ),

        Node(
            package='agv_safety',
            executable='cmd_vel_gate_node',
            name='cmd_vel_gate',
            namespace=ns,
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel_in', 'cmd_vel_collision_safe'),
                ('cmd_vel_out', 'cmd_vel_safe'),
                ('safety_status', 'safety/status'),
                ('hardware_estop', 'hardware_estop'),
            ],
            output='log',
        ),
    ])
