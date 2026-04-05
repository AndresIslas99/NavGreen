"""Validate that all required TF frames exist within 5 seconds (acceptance.yaml)."""
import subprocess
import time
import pytest

REQUIRED_FRAMES = [
    'map', 'odom', 'base_link',
    'left_wheel', 'right_wheel', 'base_footprint',
]
TIMEOUT_S = 5


def get_frames():
    result = subprocess.run(
        ['ros2', 'run', 'tf2_tools', 'view_frames', '--no-wait'],
        capture_output=True, text=True, timeout=10)
    return result.stdout


def test_tf_frames_exist():
    """All required TF frames must appear within 5 seconds."""
    deadline = time.time() + TIMEOUT_S
    found = set()

    while time.time() < deadline and len(found) < len(REQUIRED_FRAMES):
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/tf', '--once', '--no-arr'],
            capture_output=True, text=True, timeout=3)
        for frame in REQUIRED_FRAMES:
            if frame in result.stdout:
                found.add(frame)
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/tf_static', '--once', '--no-arr'],
            capture_output=True, text=True, timeout=3)
        for frame in REQUIRED_FRAMES:
            if frame in result.stdout:
                found.add(frame)

    missing = set(REQUIRED_FRAMES) - found
    assert not missing, f"TF frames not found within {TIMEOUT_S}s: {missing}"
