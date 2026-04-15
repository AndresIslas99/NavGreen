#!/bin/bash
# AGV boot launcher — called by agv.service
#
# Environment variables (set in systemd unit or override):
#   AGV_MODE    — "real" (default), "mapping", or "hil"
#   AGV_MAP     — absolute path to map YAML file (optional in real mode)
#   AGV_MAP_DIR — base directory for named maps (default: install share)
#
# Real-mode map resolution (first boot ⇒ no map; subsequent boots ⇒ last map):
#   1. If AGV_MAP is set explicitly in the environment, use it as-is.
#   2. Else if ~/.agv/last_map exists, resolve ${AGV_MAP_DIR}/$(cat)/last_map.yaml
#      and use it if the file exists on disk.
#   3. Else start in mapping-first mode (no map arg → Nav2 stays down).
#
# map_manager_node writes ~/.agv/last_map atomically each time it sees a
# successful maps/loaded event. Rebooting the robot then picks up where the
# operator left off automatically.
#
# To change mode:
#   sudo systemctl edit agv.service
#   Add: [Service]
#        Environment=AGV_MODE=real

set -eo pipefail

# ── Network readiness + DDS config generation ──────────────────────────────
#
# CycloneDDS treats every <NetworkInterface> declared in its XML as required:
# if any listed interface is not in operstate=up at rmw_create_node time,
# the call fails with "does not match an available interface" and every ROS
# node SIGABRTs. That coupling is fatal in a production robot where a WiFi
# radio can be rfkill-blocked, unassociated, or physically absent.
#
# Two historical incidents (2026-04-13):
#   1. cyclonedds.xml listed l4tbr0 (USB host bridge). In the field there is
#      no dev PC attached → no l4tbr0 → every boot failed.
#   2. After removing l4tbr0, cyclonedds.xml still listed wlP1p1s0. The WiFi
#      radio is rfkill-soft-blocked on this Jetson → wlP1p1s0 stays DOWN →
#      every node SIGABRTs, identical symptom, different interface.
#
# Fix: generate /tmp/agv_cyclonedds_runtime.xml at boot from the whitelist
# (eno1, wlP1p1s0), including only interfaces currently in operstate=up.
# Cyclone then sees a config that matches reality; it never tries to bind
# against a missing interface. The static XMLs under config/ remain as
# canonical templates for manual debug use.

# Opportunistic: rfkill-unblock WiFi if we have permission. Fails silently
# for unprivileged users — the dynamic XML gen below makes rfkill state
# irrelevant to boot success, so this is pure belt-and-suspenders.
rfkill unblock wifi 2>/dev/null || true

network_ready() {
    for iface in eno1 wlP1p1s0; do
        if ip -4 addr show "$iface" 2>/dev/null | grep -q 'inet '; then
            echo "  $iface: has IPv4"
            return 0
        fi
    done
    return 1
}

echo "Waiting for a usable network interface (eno1 or wlP1p1s0), up to 90 s..."
for i in $(seq 1 90); do
    if network_ready; then
        echo "  network ready on attempt $i"
        break
    fi
    sleep 1
done
if ! network_ready; then
    echo "  WARNING: no eno1/wlP1p1s0 IPv4 after 90 s — continuing anyway."
    echo "           DDS will still try to bind; healthcheck will surface any fault."
fi
sleep 1

# ── Generate runtime cyclonedds XML ────────────────────────────────────────
# Enumerate whitelisted interfaces currently in operstate=up. Never include
# l4tbr0/usb*/docker*/can* — those are dev artifacts or non-IP transports.
CYCLONE_CANDIDATES="eno1 wlP1p1s0"
CYCLONE_RUNTIME_XML="/tmp/agv_cyclonedds_runtime.xml"
CYCLONE_DETECTED=""
for iface in $CYCLONE_CANDIDATES; do
    if [ -f "/sys/class/net/${iface}/operstate" ] && \
       [ "$(cat /sys/class/net/${iface}/operstate 2>/dev/null)" = "up" ]; then
        CYCLONE_DETECTED="${CYCLONE_DETECTED}${iface} "
    fi
done
CYCLONE_DETECTED="${CYCLONE_DETECTED% }"

{
    echo '<?xml version="1.0" encoding="UTF-8"?>'
    echo '<!-- Generated at boot by agv_start.sh — do not edit by hand.'
    echo '     Canonical template: src/agv_bringup/config/cyclonedds_production.xml -->'
    echo '<CycloneDDS xmlns="https://cdds.io/config"'
    echo '            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"'
    echo '            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">'
    echo '  <Domain id="any">'
    echo '    <General>'
    if [ -n "$CYCLONE_DETECTED" ]; then
        echo '      <Interfaces>'
        for iface in $CYCLONE_DETECTED; do
            echo "        <NetworkInterface name=\"${iface}\" priority=\"default\" multicast=\"true\" />"
        done
        echo '      </Interfaces>'
        echo '      <AllowMulticast>true</AllowMulticast>'
    else
        # Fallback: no whitelisted iface up → localhost-only, let Cyclone auto-select.
        echo '      <AllowMulticast>spdp</AllowMulticast>'
    fi
    echo '      <MaxMessageSize>65500B</MaxMessageSize>'
    echo '      <FragmentSize>32768B</FragmentSize>'
    echo '    </General>'
    echo '    <Discovery>'
    echo '      <LeaseDuration>30s</LeaseDuration>'
    echo '      <ParticipantIndex>auto</ParticipantIndex>'
    echo '      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>'
    echo '      <Peers>'
    echo '        <Peer address="localhost"/>'
    echo '      </Peers>'
    echo '    </Discovery>'
    echo '    <Internal>'
    echo '      <SocketReceiveBufferSize min="26MB" />'
    echo '      <Watermarks>'
    echo '        <WhcHigh>500kB</WhcHigh>'
    echo '      </Watermarks>'
    echo '    </Internal>'
    echo '    <Tracing>'
    echo '      <OutputFile>/tmp/cyclonedds.log</OutputFile>'
    echo '      <Verbosity>warning</Verbosity>'
    echo '    </Tracing>'
    echo '  </Domain>'
    echo '</CycloneDDS>'
} > "$CYCLONE_RUNTIME_XML"

if [ -n "$CYCLONE_DETECTED" ]; then
    echo "  cyclonedds runtime xml generated with interfaces: ${CYCLONE_DETECTED}"
else
    echo "  cyclonedds runtime xml generated with NO interfaces (localhost-only fallback)"
fi

# ROS2 setup.bash references unset variables — -u would fail on AMENT_TRACE_SETUP_FILES
source /opt/ros/humble/setup.bash
source /home/orza/ros2_ws/install/setup.bash

# Ensure the maps directory exists (this is where per-map .yaml/.pgm/.area
# and cuVSLAM keyframe folders live). The ZED Area Memory "landing pad"
# .current.area file is NOT pre-created here — the ZED SDK rejects empty
# .area files and the whole pos_tracking module crashes. Instead we let the
# wrapper start without an area file (mAreaMemoryDbPath is cleared on the
# fs::exist check) and the (patched) startPosTracking re-reads the param on
# each reset_pos_tracking call, so as soon as map_manager writes a real
# file via save_area_memory the next reset picks it up.
AGV_DATA_MAPS="/home/orza/agv_data/maps"
mkdir -p "$AGV_DATA_MAPS"
# If a leftover empty landing pad exists from a previous broken boot,
# remove it so the wrapper treats the state as "no area memory yet".
AGV_AREA_LANDING="${AGV_DATA_MAPS}/.current.area"
if [ -f "$AGV_AREA_LANDING" ] && [ ! -s "$AGV_AREA_LANDING" ]; then
    rm -f "$AGV_AREA_LANDING"
    echo "  removed empty stale landing pad: $AGV_AREA_LANDING"
fi

MODE="${AGV_MODE:-real}"
MAP="${AGV_MAP:-}"
MAP_DIR="${AGV_MAP_DIR:-/home/orza/ros2_ws/install/agv_navigation/share/agv_navigation/maps}"
LAST_MAP_FILE="${HOME}/.agv/last_map"
DEFAULT_MAP="${MAP_DIR}/default_empty.yaml"

echo "AGV starting: mode=$MODE"

# Map resolution priority:
#   1. AGV_MAP explicitly set in the environment
#   2. ~/.agv/last_map (the last map the operator saved)
#   3. default_empty.yaml — 20m × 20m all-free placeholder so Nav2 + safety
#      chain still come up. Operator can then use the dashboard "Load Map"
#      button to replace it at runtime without rebooting the stack.
if [ -z "$MAP" ] && [ -f "$LAST_MAP_FILE" ]; then
    LAST_NAME="$(head -n1 "$LAST_MAP_FILE" | tr -d '[:space:]')"
    if [ -n "$LAST_NAME" ]; then
        CANDIDATE="${MAP_DIR}/${LAST_NAME}.yaml"
        if [ -f "$CANDIDATE" ]; then
            MAP="$CANDIDATE"
            echo "  restoring last map: $LAST_NAME"
        else
            echo "  last_map='$LAST_NAME' but $CANDIDATE missing — using default_empty"
        fi
    fi
fi
if [ -z "$MAP" ]; then
    MAP="$DEFAULT_MAP"
    echo "  no saved map — booting with default_empty (operator can Load Map at runtime)"
fi

# All production modes use the dynamically generated runtime XML. The static
# config/cyclonedds_production.xml and src/agv_slam/config/cyclonedds.xml are
# templates only — see the header block above for rationale.
export CYCLONEDDS_URI="file://${CYCLONE_RUNTIME_XML}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ "$MODE" = "mapping" ]; then
    # Mapping commissioning: teleop + SLAM, no Nav2, no pre-existing map needed.
    # Drive at 0.3-0.5 m/s through greenhouse corridors, save map when done.
    exec ros2 launch agv_bringup agv_mapping.launch.py

elif [ "$MODE" = "real" ]; then
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
    exec ros2 launch agv_bringup agv_hil_full.launch.py "map:=$MAP"

else
    echo "ERROR: Unknown AGV_MODE=$MODE (expected: mapping, real, hil)"
    exit 1
fi
