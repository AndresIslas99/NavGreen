"""Validate dual EKF frame ownership (acceptance.yaml).

Uses a tf2_ros buffer instead of shelling out to tf2_echo: tf2_echo has
no bounded mode in Humble, so a subprocess timeout always fired before
its output could be inspected.

Skips unless AGV_STACK_TEST=1 is set with the full stack running.
"""
import os
import time

import pytest

try:
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
except ImportError:
    pytest.skip("rclpy / tf2_ros not available", allow_module_level=True)

if os.environ.get("AGV_STACK_TEST") != "1":
    pytest.skip(
        "stack-required test: set AGV_STACK_TEST=1 with the full stack "
        "running (agv_full.launch.py)", allow_module_level=True)

TF_WAIT_S = 8.0


@pytest.fixture(scope='module')
def tf_buffer():
    rclpy.init()
    node = rclpy.create_node('test_ekf_frames')
    buf = Buffer()
    listener = TransformListener(buf, node, spin_thread=True)
    yield buf
    del listener
    node.destroy_node()
    rclpy.shutdown()


def wait_for_transform(buf, target, source, timeout_s=TF_WAIT_S):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if buf.can_transform(target, source, Time()):
            return True
        time.sleep(0.1)
    return False


def test_odom_to_base_link_exists(tf_buffer):
    """ekf_local must publish odom -> base_link."""
    assert wait_for_transform(tf_buffer, 'odom', 'base_link'), \
        f"odom->base_link TF not available within {TF_WAIT_S} s"


def test_map_to_odom_exists(tf_buffer):
    """ekf_global must publish map -> odom."""
    assert wait_for_transform(tf_buffer, 'map', 'odom'), \
        f"map->odom TF not available within {TF_WAIT_S} s"
