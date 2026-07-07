"""
agv_sim -- teleop_sim.launch.py

Includes sim.launch.py and adds teleop_twist_keyboard so you can drive the AGV
interactively with the keyboard.

teleop_twist_keyboard NEEDS AN INTERACTIVE TERMINAL to read keystrokes. This
launch runs it inside an xterm window (prefix 'xterm -e'), which requires the
`xterm` package to be installed. If you are headless, or prefer your own
terminal, launch the sim and run teleop yourself in a second terminal:

  ros2 launch agv_sim sim.launch.py gui:=true
  # then, in another terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
      --ros-args -r cmd_vel:=/cmd_vel

Launch args:
  gui:=true   Start gzclient GUI (default true here -- you want to see the
              robot while driving). Forwarded to sim.launch.py.

Example:
  ros2 launch agv_sim teleop_sim.launch.py            # GUI + xterm teleop
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_sim')

    gui = LaunchConfiguration('gui')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui}.items(),
    )

    # Keyboard teleop. Runs in its own xterm so it has a real TTY for stdin.
    # Publishes geometry_msgs/Twist; remapped from cmd_vel to /cmd_vel to feed
    # the diff_drive_controller.
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        remappings=[('cmd_vel', '/cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start gzclient GUI (recommended for teleop)'),
        sim,
        teleop,
    ])
