#!/usr/bin/env bash
# One-time workspace setup inside the dev container.
# Uses the exact dependency recipe CI runs (see .github/workflows/ci.yaml):
# the whole workspace is resolved so cross-package depends count as local,
# and vendor-only keys (Isaac ROS, ZED, GTSAM) are skipped — the packages
# that need them are documented in README "Vendor SDK dependencies".
set -euo pipefail

source /opt/ros/humble/setup.bash

rosdep update || true
rosdep install --from-paths src --ignore-src -y \
  --skip-keys="isaac_ros_visual_slam isaac_ros_visual_slam_interfaces isaac_ros_nvblox isaac_ros_apriltag_interfaces zed_msgs gtsam"

mkdir -p "${AGV_DATA_DIR:-$PWD/.agv_data}"

echo
echo "Setup complete. Common commands:"
echo "  colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS=\"-Werror\""
echo "  colcon test && colcon test-result --verbose"
echo "  bash tools/verify_specs/all.sh        # SSOT verifier suite"
echo "  cd web/agv_dashboard && npm ci && npm run build"
