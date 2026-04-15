"""
agv_hw_interface — launch_testing smoke test for the mock_components flow.

Brings up the mock launch (no real CAN required) and verifies that the
controller_manager comes up. This is the second exemplar for Gap 4 (per-package
launch_testing) of docs/architectural_gaps.md.

Run via:
  colcon test --packages-select agv_hw_interface --event-handlers console_direct+

Note: this test depends on `mock_components` and `controller_manager` being
installed in the environment. On a Jetson with ros-humble-desktop they are
present by default.
"""

import time
import unittest

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


@pytest.mark.launch_test
def generate_test_description():
    mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agv_hw_interface'),
                'launch', 'agv_ros2control_mock.launch.py',
            ])),
    )
    return LaunchDescription([
        mock_launch,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestControllerManagerComesUp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_mock_smoke')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_joint_states_topic_exists(self):
        # Wait up to 15s for /joint_states to appear (controller_manager +
        # joint_state_broadcaster need time to spawn).
        deadline = time.monotonic() + 15.0
        seen = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            topics = dict(self.node.get_topic_names_and_types())
            if '/joint_states' in topics:
                seen = True
                break
        self.assertTrue(seen, "joint_state_broadcaster did not publish /joint_states within 15s")
