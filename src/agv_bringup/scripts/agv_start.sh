#!/bin/bash
# AGV boot launcher — called by agv.service
#
# Environment variables (set in systemd unit or override):
#   AGV_MODE  — "hil" (default) or "real"
#   AGV_MAP   — absolute path to map YAML file
#
# To change mode:
#   sudo systemctl edit agv.service
#   Add: [Service]
#        Environment=AGV_MODE=real
#        Environment=AGV_MAP=/path/to/greenhouse.yaml

set -eo pipefail

# Wait for eno1 to get an IP address (DHCP from router may be slow)
echo "Waiting for eno1 IPv4 address (up to 60s)..."
for i in $(seq 1 60); do
    if ip -4 addr show eno1 2>/dev/null | grep -q 'inet '; then
        echo "  eno1: has IPv4 (attempt $i)"
        break
    fi
    sleep 1
done

# l4tbr0 is always-on USB bridge, just verify it's up
if ip link show l4tbr0 2>/dev/null | grep -q "state UP"; then
    echo "  l4tbr0: UP"
fi
sleep 1

# ROS2 setup.bash references unset variables — -u would fail on AMENT_TRACE_SETUP_FILES
source /opt/ros/humble/setup.bash
source /home/orza/ros2_ws/install/setup.bash

MODE="${AGV_MODE:-hil}"
MAP="${AGV_MAP:-}"

echo "AGV starting: mode=$MODE"

if [ "$MODE" = "mapping" ]; then
    # Mapping commissioning: teleop + SLAM, no Nav2, no pre-existing map needed.
    # Drive at 0.3-0.5 m/s through greenhouse corridors, save map when done.
    export CYCLONEDDS_URI="file:///home/orza/ros2_ws/src/agv_bringup/config/cyclonedds_production.xml"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    exec ros2 launch agv_bringup agv_mapping.launch.py

elif [ "$MODE" = "real" ]; then
    export CYCLONEDDS_URI="file:///home/orza/ros2_ws/src/agv_bringup/config/cyclonedds_production.xml"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    if [ -n "$MAP" ]; then
        echo "  map=$MAP"
        exec ros2 launch agv_bringup agv_full.launch.py "map:=$MAP"
    else
        echo "  No map — start in mapping-first mode (Nav2 disabled until map loaded)"
        exec ros2 launch agv_bringup agv_full.launch.py
    fi

elif [ "$MODE" = "hil" ]; then
    if [ -z "$MAP" ]; then
        echo "ERROR: AGV_MAP not set. Set it in agv.service or override."
        exit 1
    fi
    export CYCLONEDDS_URI="file://$(ros2 pkg prefix agv_slam)/share/agv_slam/config/cyclonedds.xml"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    exec ros2 launch agv_bringup agv_hil_full.launch.py "map:=$MAP"

else
    echo "ERROR: Unknown AGV_MODE=$MODE (expected: mapping, real, hil)"
    exit 1
fi
