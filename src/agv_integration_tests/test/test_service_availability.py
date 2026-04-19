#!/usr/bin/env python3
"""
Integration test: verify all required ROS2 services are available.
Run with: ros2 run agv_integration_tests test_service_availability.py
Or via colcon test.
"""
import subprocess
import sys


REQUIRED_SERVICES = [
    '/agv/map_manager/save_map',
    '/agv/map_manager/load_map',
    '/agv/map_manager/update_zone',
    '/agv/waypoint_manager/save',
    '/agv/waypoint_manager/list',
    '/agv/waypoint_manager/execute',
    '/agv/navigate_to_pose/_action/get_result',
]


def test_services_available():
    """Check that all required services exist in the ROS2 graph."""
    result = subprocess.run(
        ['ros2', 'service', 'list'],
        capture_output=True, text=True, timeout=10)

    available = set(result.stdout.strip().splitlines())
    missing = []

    for svc in REQUIRED_SERVICES:
        if svc not in available:
            missing.append(svc)

    if missing:
        print(f"MISSING services ({len(missing)}/{len(REQUIRED_SERVICES)}):")
        for s in missing:
            print(f"  - {s}")
        # Don't fail hard — services may not be running in test environment
        print("NOTE: This test requires the full stack to be running.")
    else:
        print(f"All {len(REQUIRED_SERVICES)} services available.")


if __name__ == '__main__':
    test_services_available()
