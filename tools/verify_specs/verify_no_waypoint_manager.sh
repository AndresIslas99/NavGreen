#!/bin/bash
# verify_no_waypoint_manager.sh
#
# Sprint Section-0 / HIGH-11-B-01 / G_F7. Prevents accidental re-introduction
# of `agv_waypoint_manager` as a Node() in any production launch file.
# The package was removed from production launch in Sprint B because
# `agv_ui_backend` runs its own gated mission executor (HIGH-11-B-01).
#
# Fails BLOCKING if it finds an executable `package='agv_waypoint_manager'`
# or `executable='waypoint_manager'` line in any agv_bringup launch file
# that is not inside a python comment block.
#
# Allowed:
#   - Comments mentioning the removal (`# agv_waypoint_manager removed ...`)
#   - The package directory itself (its CMakeLists, src/, etc.)
# Forbidden:
#   - A live Node(package='agv_waypoint_manager', ...) declaration
#   - An IncludeLaunchDescription pointing at agv_waypoint_manager/launch/

set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

LAUNCH_DIR="src/agv_bringup/launch"
FOUND=0

if [ ! -d "$LAUNCH_DIR" ]; then
  echo "verify_no_waypoint_manager: SKIP — $LAUNCH_DIR not present"
  exit 0
fi

# Look at every line that mentions waypoint_manager in production launches.
# Strip python comment lines (^\s*#). A surviving hit is a regression.
HITS=$(grep -nE "(package=['\"]agv_waypoint_manager['\"]|executable=['\"]waypoint_manager['\"]|FindPackageShare\\(['\"]agv_waypoint_manager['\"]\\))" \
        "$LAUNCH_DIR"/*.launch.py 2>/dev/null | grep -vE ":\s*#" || true)

if [ -n "$HITS" ]; then
  echo "verify_no_waypoint_manager: FAIL — live references found"
  echo "$HITS"
  echo
  echo "agv_waypoint_manager was removed from production launch in Sprint B"
  echo "(HIGH-11-B-01). The dashboard backend (agv_ui_backend) executes"
  echo "missions through a gated path. If you genuinely need this node back,"
  echo "discuss with audit owner first and update this verifier."
  exit 1
fi

echo "verify_no_waypoint_manager: OK"
exit 0
