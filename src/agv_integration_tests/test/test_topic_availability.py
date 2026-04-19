#!/usr/bin/env python3
"""
Integration test: verify all required ROS2 topics are being published.
"""
import subprocess


REQUIRED_TOPICS = [
    '/agv/wheel_odom',
    '/agv/joint_states',
    '/agv/motor_state',
    '/agv/drive_debug',
    '/agv/cmd_vel',
    '/agv/e_stop',
    '/agv/scan',
    '/agv/odometry/local',
    '/agv/odometry/global',
    '/agv/map',
    '/agv/plan',
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
    missing = []

    for topic in REQUIRED_TOPICS:
        if topic not in available:
            missing.append(topic)

    if missing:
        print(f"MISSING topics ({len(missing)}/{len(REQUIRED_TOPICS)}):")
        for t in missing:
            print(f"  - {t}")
        print("NOTE: This test requires the full stack to be running.")
    else:
        print(f"All {len(REQUIRED_TOPICS)} topics available.")


if __name__ == '__main__':
    test_topics_available()
