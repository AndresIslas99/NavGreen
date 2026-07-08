"""Validate that all required TF frames exist within 5 seconds (acceptance.yaml).

Skips unless AGV_STACK_TEST=1 is set with the full stack running
(agv_full.launch.py); asserts hard when it is.
"""
import os
import subprocess
import time

import pytest

if os.environ.get("AGV_STACK_TEST") != "1":
    pytest.skip(
        "stack-required test: set AGV_STACK_TEST=1 with the full stack "
        "running (agv_full.launch.py)", allow_module_level=True)

REQUIRED_FRAMES = [
    'map', 'odom', 'base_link',
    'left_wheel', 'right_wheel', 'base_footprint',
]
TIMEOUT_S = 5


def test_tf_frames_exist():
    """All required TF frames must appear within 5 seconds."""
    deadline = time.time() + TIMEOUT_S
    found = set()

    # No --no-arr: the transforms field is an array, so suppressing arrays
    # would hide every frame_id from the echoed output.
    while time.time() < deadline and len(found) < len(REQUIRED_FRAMES):
        for topic in ('/tf', '/tf_static'):
            try:
                result = subprocess.run(
                    ['ros2', 'topic', 'echo', topic, '--once'],
                    capture_output=True, text=True, timeout=3)
            except subprocess.TimeoutExpired:
                continue
            for frame in REQUIRED_FRAMES:
                if frame in result.stdout:
                    found.add(frame)

    missing = set(REQUIRED_FRAMES) - found
    assert not missing, f"TF frames not found within {TIMEOUT_S}s: {missing}"
