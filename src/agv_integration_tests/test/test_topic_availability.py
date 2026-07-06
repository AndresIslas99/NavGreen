#!/usr/bin/env python3
"""Integration test: verify all required ROS2 topics are being published.

Skips unless AGV_STACK_TEST=1 is set with the full stack running
(agv_full.launch.py); asserts hard when it is.
"""
import os
import subprocess

import pytest

if os.environ.get("AGV_STACK_TEST") != "1":
    pytest.skip(
        "stack-required test: set AGV_STACK_TEST=1 with the full stack "
        "running (agv_full.launch.py)", allow_module_level=True)

NS = os.environ.get("AGV_NAMESPACE", "agv")

REQUIRED_TOPICS = [
    f'/{NS}/wheel_odom',
    f'/{NS}/joint_states',
    f'/{NS}/motor_state',
    f'/{NS}/drive_debug',
    f'/{NS}/cmd_vel',
    f'/{NS}/e_stop',
    f'/{NS}/scan',
    f'/{NS}/odometry/local',
    f'/{NS}/odometry/global',
    f'/{NS}/map',
    f'/{NS}/plan',
    '/tf',
    '/tf_static',
    '/visual_slam/tracking/odometry',
]


def test_topics_available():
    """Check that all required topics exist."""
    result = subprocess.run(
        ['ros2', 'topic', 'list'],
        capture_output=True, text=True, timeout=10)

    available = set(result.stdout.strip().splitlines())
    missing = [topic for topic in REQUIRED_TOPICS if topic not in available]

    if missing:
        print(f"MISSING topics ({len(missing)}/{len(REQUIRED_TOPICS)}):")
        for t in missing:
            print(f"  - {t}")
    assert not missing, f"required topics missing from ROS graph: {missing}"
    print(f"All {len(REQUIRED_TOPICS)} topics available.")


if __name__ == '__main__':
    test_topics_available()
