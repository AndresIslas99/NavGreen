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

if [ "${AGV_SKIP_NETWORK_WAIT:-0}" = "1" ]; then
    echo "Skipping network wait (AGV_SKIP_NETWORK_WAIT=1) — DDS will use whatever is up."
elif network_ready; then
    echo "  network already ready"
else
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
fi

# ── Generate runtime cyclonedds XML ────────────────────────────────────────
# Enumerate whitelisted interfaces currently in operstate=up AND with a
# carrier (link-up at L2). operstate alone is insufficient: a wifi card
# with NO-CARRIER reports operstate=up, but Cyclone treats it as missing
# and SIGABRTs every node it tries to bind. Never include
# l4tbr0/usb*/docker*/can* — those are dev artifacts or non-IP transports.
# Sprint 1 (2026-04-25): carrier check added after observing nodes failing
# to spawn when WiFi went idle between agv_start.sh detect and node init.
CYCLONE_CANDIDATES="eno1 wlP1p1s0"
CYCLONE_RUNTIME_XML="/tmp/agv_cyclonedds_runtime.xml"
CYCLONE_DETECTED=""
for iface in $CYCLONE_CANDIDATES; do
    operstate_file="/sys/class/net/${iface}/operstate"
    carrier_file="/sys/class/net/${iface}/carrier"
    [ -f "$operstate_file" ] || continue
    [ "$(cat "$operstate_file" 2>/dev/null)" = "up" ] || continue
    # carrier=1 means L2 link is good. Missing/0 means cable unplugged or
    # WiFi unassociated — Cyclone will reject this interface at node init.
    [ "$(cat "$carrier_file" 2>/dev/null || echo 0)" = "1" ] || continue
    CYCLONE_DETECTED="${CYCLONE_DETECTED}${iface} "
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
    # Sprint 1 Fase A2 (2026-04-24): SocketReceiveBufferSize 26MB→64MB and
    # WhcHigh 500kB→4MB. ZED RGB HD frames are ~900 KB each; ZED + cuVSLAM
    # + nvblox + image_server publishing simultaneously was overflowing the
    # 500 kB writer history cache and causing silent message drops. The new
    # values match the worst-case burst payload of the AGV stack with
    # headroom for image_server MJPEG re-encode latency. Revert by editing
    # this block; runtime regenerated next agv_start.sh boot.
    echo '      <SocketReceiveBufferSize min="64MB" />'
    echo '      <Watermarks>'
    echo '        <WhcHigh>4MB</WhcHigh>'
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

# Iter-37: CAN pre-flight for modes that bring up the real ODrive. If
# can0 is missing or down, odrive_can_node fails silently (error spam,
# no motor commands) — surface the root cause here instead. Only fires
# in real / mapping modes; hil / hil_full skip (no CAN needed).
if [ "$MODE" = "real" ] || [ "$MODE" = "mapping" ]; then
    if ! ip link show can0 > /dev/null 2>&1; then
        echo "ERROR: CAN interface 'can0' not found. Check:"
        echo "  - SocketCAN kernel module loaded (lsmod | grep can)"
        echo "  - Pinmux applied (docs/hardware_setup.md)"
        echo "  - Transceiver powered"
        exit 1
    fi
    # Note: `ip -details link show can0` matches twice for 'state' — once
    # for the link operstate (UP) and once for the CAN controller state
    # ("ERROR-ACTIVE" etc). Keep only the first (link-level) match.
    CAN_STATE=$(ip -details link show can0 | grep -oE 'state [A-Z]+' | head -1 | awk '{print $2}')
    if [ "$CAN_STATE" != "UP" ] && [ "$CAN_STATE" != "UNKNOWN" ]; then
        echo "ERROR: can0 state=$CAN_STATE (expected UP). Bring up with:"
        echo "  sudo ip link set can0 up type can bitrate 500000"
        exit 1
    fi
    echo "  can0: $CAN_STATE"
fi

# Sprint 1 Fase A4 (2026-04-24): optional foxglove_bridge for engineer-side
# diagnostics. Off by default in production; enable per-session by exporting
# AGV_ENABLE_FOXGLOVE=true before agv_start.sh, or via `systemctl edit
# --runtime agv.service` adding `Environment=AGV_ENABLE_FOXGLOVE=true`.
EXTRA_LAUNCH_ARGS=""
if [ "${AGV_ENABLE_FOXGLOVE:-false}" = "true" ]; then
    # Note: launch arg is `enable_foxglove_bridge`, distinct from the
    # `enable_foxglove` arg passed to agv_slam.launch.py. See comment in
    # agv_full.launch.py — same name would be overwritten by the include.
    EXTRA_LAUNCH_ARGS="$EXTRA_LAUNCH_ARGS enable_foxglove_bridge:=true"
    echo "  foxglove_bridge enabled on :8765 (diagnostic only)"
fi

if [ "$MODE" = "mapping" ]; then
    # Mapping commissioning: teleop + SLAM, no Nav2, no pre-existing map needed.
    # Drive at 0.3-0.5 m/s through greenhouse corridors, save map when done.
    exec ros2 launch agv_bringup agv_mapping.launch.py $EXTRA_LAUNCH_ARGS

elif [ "$MODE" = "real" ]; then
    if [ -n "$MAP" ]; then
        echo "  map=$MAP"
        exec ros2 launch agv_bringup agv_full.launch.py "map:=$MAP" $EXTRA_LAUNCH_ARGS
    else
        echo "  No map — start in mapping-first mode (Nav2 disabled until map loaded)"
        exec ros2 launch agv_bringup agv_full.launch.py $EXTRA_LAUNCH_ARGS
    fi

elif [ "$MODE" = "hil" ]; then
    if [ -z "$MAP" ]; then
        echo "ERROR: AGV_MAP not set. Set it in agv.service or override."
        exit 1
    fi
    exec ros2 launch agv_bringup agv_hil_full.launch.py "map:=$MAP" $EXTRA_LAUNCH_ARGS

elif [ "$MODE" = "hil_full" ]; then
    # Same brain as production (agv_full.launch.py) but with hil_mode:=true
    # to skip nodes that need physical hardware (ZED+cuVSLAM, ODrive CAN, IMU
    # filter, pointcloud_to_laserscan). Use this mode when HIL validation
    # needs the full brain stack: safety chain, auto_init_orchestrator,
    # map_manager, and/or the complete Nav2 pipeline as deployed in production.
    # agv_hil_full.launch.py is the simpler alternative that skips those too.
    if [ -z "$MAP" ]; then
        echo "ERROR: AGV_MAP not set. hil_full requires a map. Set it in agv.service or override."
        exit 1
    fi
    exec ros2 launch agv_bringup agv_full.launch.py "map:=$MAP" hil_mode:=true $EXTRA_LAUNCH_ARGS

else
    echo "ERROR: Unknown AGV_MODE=$MODE (expected: mapping, real, hil, hil_full)"
    exit 1
fi
