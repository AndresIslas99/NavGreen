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

# Wait for network interface to be available (WiFi takes seconds after boot)
IFACE="${AGV_IFACE:-wlP1p1s0}"
echo "Waiting for network interface $IFACE..."
for i in $(seq 1 30); do
    if ip link show "$IFACE" 2>/dev/null | grep -q "state UP"; then
        echo "Interface $IFACE is UP (attempt $i)"
        break
    fi
    if [ "$i" -eq 30 ]; then
        echo "WARNING: $IFACE not UP after 30s, launching anyway"
    fi
    sleep 1
done

# Extra wait for IP assignment
sleep 2

# ROS2 setup.bash references unset variables — -u would fail on AMENT_TRACE_SETUP_FILES
source /opt/ros/humble/setup.bash
source /home/orza/ros2_ws/install/setup.bash

MODE="${AGV_MODE:-hil}"
MAP="${AGV_MAP:-}"

if [ -z "$MAP" ]; then
    echo "ERROR: AGV_MAP not set. Set it in agv.service or override."
    exit 1
fi

echo "AGV starting: mode=$MODE map=$MAP"

if [ "$MODE" = "hil" ]; then
    export CYCLONEDDS_URI="file://$(ros2 pkg prefix agv_slam)/share/agv_slam/config/cyclonedds.xml"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    exec ros2 launch agv_bringup agv_hil_full.launch.py "map:=$MAP"

elif [ "$MODE" = "real" ]; then
    exec ros2 launch agv_bringup agv_full.launch.py "map:=$MAP"

else
    echo "ERROR: Unknown AGV_MODE=$MODE (expected: hil or real)"
    exit 1
fi
