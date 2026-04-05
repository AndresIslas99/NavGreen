# Copyright 2026 AGV Greenhouse
# Licensed under MIT

import os
import subprocess
import pytest


PKG_DIR = os.path.join(
    os.path.dirname(__file__), os.pardir
)
URDF_DIR = os.path.join(PKG_DIR, 'urdf')


def _run(cmd, **kwargs):
    return subprocess.run(cmd, capture_output=True, text=True, cwd=PKG_DIR, **kwargs)


def test_xacro_expands_without_errors():
    """Verify that the main xacro file expands without parse errors."""
    result = _run(['xacro', os.path.join(URDF_DIR, 'agv_full.urdf.xacro')])
    assert result.returncode == 0, (
        f"Xacro expansion failed:\n{result.stderr}"
    )
    assert '<robot' in result.stdout, "Expanded URDF missing <robot> element"


def test_urdf_has_expected_links():
    """Verify that required links are present in the expanded URDF."""
    result = _run(['xacro', os.path.join(URDF_DIR, 'agv_full.urdf.xacro')])
    assert result.returncode == 0, f"Xacro failed: {result.stderr}"

    urdf = result.stdout
    required_links = ['base_link', 'base_footprint', 'left_wheel', 'right_wheel']
    for link in required_links:
        assert f'name="{link}"' in urdf, f"Missing link: {link}"


def test_urdf_has_expected_joints():
    """Verify that required joints are present."""
    result = _run(['xacro', os.path.join(URDF_DIR, 'agv_full.urdf.xacro')])
    assert result.returncode == 0, f"Xacro failed: {result.stderr}"

    urdf = result.stdout
    required_joints = ['left_wheel_joint', 'right_wheel_joint', 'base_footprint_joint']
    for joint in required_joints:
        assert f'name="{joint}"' in urdf, f"Missing joint: {joint}"


def test_check_urdf():
    """Run check_urdf on the expanded URDF if available."""
    xacro_result = _run(['xacro', os.path.join(URDF_DIR, 'agv_full.urdf.xacro')])
    assert xacro_result.returncode == 0, f"Xacro failed: {xacro_result.stderr}"

    check_result = _run(['check_urdf', '/dev/stdin'], input=xacro_result.stdout)
    if check_result.returncode == 127:
        pytest.skip("check_urdf not installed")
    assert check_result.returncode == 0, (
        f"check_urdf validation failed:\n{check_result.stderr}"
    )
