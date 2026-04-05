"""Validate dual EKF frame ownership (acceptance.yaml)."""
import subprocess
import pytest


def run_cmd(cmd, timeout=5):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def test_odom_to_base_link_exists():
    """ekf_local must publish odom -> base_link."""
    result = run_cmd([
        'ros2', 'run', 'tf2_ros', 'tf2_echo', 'odom', 'base_link'],
        timeout=8)
    # tf2_echo prints "At time ..." on success
    assert 'At time' in result.stdout or result.returncode == 0, \
        f"odom->base_link TF not available: {result.stderr}"


def test_map_to_odom_exists():
    """ekf_global must publish map -> odom."""
    result = run_cmd([
        'ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', 'odom'],
        timeout=8)
    assert 'At time' in result.stdout or result.returncode == 0, \
        f"map->odom TF not available: {result.stderr}"
