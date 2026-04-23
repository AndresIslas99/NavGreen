#!/usr/bin/env bash
# kill_hil_stack.sh — clean teardown of the HIL brain stack.
#
# Why this exists (iter-43 Bug 2 research):
#
# `pkill -9 -f "ros2 launch"` kills the launcher but the 25-28 nodes it
# spawned survive as orphans adopted by init (PPID=1). Those orphans
# then hold DDS participant slots in domain 42 until (a) the 10 s
# CycloneDDS LeaseDuration expires AND (b) the kernel releases the UDP
# ports. On a USB-eth link with AllowMulticast=spdp, stale participants
# accumulate across iterations — after ~6 kill/relaunch cycles the
# MaxAutoParticipantIndex=120 cap is hit (which is the RTPS protocol
# hard cap, NOT a tunable we can raise) and every subsequent launch
# SIGABRTs with "Failed to find a free participant index for domain 42".
#
# Root cause: SIGKILL skips the rclcpp Node destructor, so Cyclone never
# sends the DATA(p) dispose+unregister that lets peers purge the proxy.
# pkill with a pattern that matches "ros2 launch" doesn't match the
# spawned children, whose cmdlines are things like
# /opt/ros/humble/lib/robot_localization/ekf_node etc.
#
# This script does the proper cascade (SIGINT → SIGTERM → SIGKILL last)
# and then cleans up orphans adopted by init.
#
# Usage:
#   bash scripts/kill_hil_stack.sh [--hard]
#     --hard  skip the SIGINT wait (only for stuck processes)

set -euo pipefail

HARD=0
for arg in "$@"; do
  [[ "$arg" == "--hard" ]] && HARD=1
done

ROS_PAT='ros2|agv|tf2_ros|ekf_node|robot_state_publisher|component_container|nav2|controller_server|planner_server|bt_navigator|collision_monitor|lifecycle_manager|apriltag|slam_toolbox|pointcloud_to_laserscan|vslam|mode_arbiter|rail_|marker_correction|covariance_override|teleop|scan_grid|image_server|smoother|velocity_smoother|behavior_server|map_server|zone_detector|gt_to_wheel|sim_obstacle|apriltag_sim|static_transform'

list_ros_pids() {
  # PPID=1 catches orphans; PPID>1 catches the launcher hierarchy.
  ps -u "$USER" -eo pid,ppid,cmd --no-headers | \
    awk -v pat="$ROS_PAT" '$3 ~ pat && $3 !~ /kill_hil_stack|grep|awk|claude|ros-mcp-server/ {print $1}'
}

count() { list_ros_pids | wc -l; }

echo "kill_hil_stack: initial ROS processes = $(count)"

if (( HARD == 0 )); then
  # Stage 1: SIGINT to launcher process group. rclcpp Node destructors
  # run, DDS participants deregister properly. 5 s max.
  echo "kill_hil_stack: SIGINT to 'ros2 launch' launchers (5 s grace)..."
  pkill -INT -u "$USER" -f "ros2 launch" 2>/dev/null || true
  for i in $(seq 1 5); do
    sleep 1
    n=$(count)
    if (( n == 0 )); then
      echo "kill_hil_stack: clean exit after ${i}s SIGINT"
      exit 0
    fi
  done
  echo "kill_hil_stack: $n processes still alive after SIGINT, escalating"

  # Stage 2: SIGTERM broad.
  list_ros_pids | xargs -r kill -TERM 2>/dev/null || true
  sleep 3
fi

# Stage 3: SIGKILL everything left, including init-adopted orphans.
pids=$(list_ros_pids)
if [[ -n "$pids" ]]; then
  echo "kill_hil_stack: SIGKILL $(echo $pids | wc -w) stragglers"
  echo "$pids" | xargs -r kill -9 2>/dev/null || true
  sleep 2
fi

# Stage 4: ros2 daemon may itself hold a participant.
ros2 daemon stop 2>/dev/null || true

final=$(count)
echo "kill_hil_stack: final ROS processes = $final"
if (( final > 0 )); then
  echo "WARNING: $final processes still alive:"
  list_ros_pids | xargs -r ps -f -p 2>/dev/null | head -10
  exit 1
fi
echo "kill_hil_stack: clean"
