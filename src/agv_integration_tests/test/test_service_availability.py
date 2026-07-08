#!/usr/bin/env python3
"""Integration test: verify all required ROS2 services are available.

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

REQUIRED_SERVICES = [
    f'/{NS}/map_manager/save_map',
    f'/{NS}/map_manager/load_map',
    f'/{NS}/map_manager/update_zone',
    f'/{NS}/waypoint_manager/save',
    f'/{NS}/waypoint_manager/list',
    f'/{NS}/waypoint_manager/execute',
    f'/{NS}/navigate_to_pose/_action/get_result',
]


def test_services_available():
    """Check that all required services exist in the ROS2 graph."""
    result = subprocess.run(
        ['ros2', 'service', 'list'],
        capture_output=True, text=True, timeout=10)

    available = set(result.stdout.strip().splitlines())
    missing = [svc for svc in REQUIRED_SERVICES if svc not in available]

    if missing:
        print(f"MISSING services ({len(missing)}/{len(REQUIRED_SERVICES)}):")
        for s in missing:
            print(f"  - {s}")
    assert not missing, f"required services missing from ROS graph: {missing}"
    print(f"All {len(REQUIRED_SERVICES)} services available.")


if __name__ == '__main__':
    test_services_available()
