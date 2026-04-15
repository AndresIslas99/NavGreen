"""
agv_safety — launch_testing smoke test for cmd_vel_gate.

Brings up cmd_vel_gate in isolation (no supervisor) and verifies that, with
no SafetyStatus arriving, the gate blocks all cmd_vel input. This is the
'fail-safe by default' behavior.

Run via:
  colcon test --packages-select agv_safety --event-handlers console_direct+

This is the exemplar for Gap 4 (per-package launch_testing) of
docs/architectural_gaps.md.
"""

import time
import unittest

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    gate = Node(
        package='agv_safety',
        executable='cmd_vel_gate_node',
        name='cmd_vel_gate',
        parameters=[{
            'max_linear': 0.5,
            'max_angular': 1.5,
            'safety_timeout_s': 0.3,
        }],
        output='screen',
    )
    return LaunchDescription([
        gate,
        launch_testing.actions.ReadyToTest(),
    ]), {'gate': gate}


class TestGateBlocksWithoutSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_gate_smoke')
        cls.received = []
        cls.sub = cls.node.create_subscription(
            Twist, '/cmd_vel_out',
            lambda m: cls.received.append(m), 10)
        cls.pub = cls.node.create_publisher(Twist, '/cmd_vel_in', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_blocks_when_safety_unknown(self):
        # Wait for the gate to be discoverable.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.pub.get_subscription_count() > 0:
                break
        self.assertGreater(self.pub.get_subscription_count(), 0,
                           "cmd_vel_gate did not appear within 5 seconds")

        # Send a clearly non-zero input.
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.4
        for _ in range(5):
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Drain any pending output messages.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertGreater(len(self.received), 0,
                           "expected at least one Twist on /cmd_vel_out")
        for out in self.received:
            self.assertAlmostEqual(out.linear.x, 0.0, places=6,
                                   msg="gate must zero linear when safety unknown")
            self.assertAlmostEqual(out.angular.z, 0.0, places=6,
                                   msg="gate must zero angular when safety unknown")


@launch_testing.post_shutdown_test()
class TestGateExitsCleanly(unittest.TestCase):
    def test_clean_shutdown(self, proc_info, gate):
        launch_testing.asserts.assertExitCodes(proc_info, process=gate)
