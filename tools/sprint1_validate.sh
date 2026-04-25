#!/usr/bin/env bash
# Sprint 1 Sprint validation runbook — Fase 3.A5 + 1c smoke checks.
#
# Run this on the Jetson dev box AFTER agv_start.sh is up. It exercises:
#   1. image_server thread cap (Fase A1)  — opens 8 MJPEG clients, expects
#      4 served + 4 receiving HTTP 503.
#   2. foxglove_bridge presence (Fase A4) — only checked when launched with
#      enable_foxglove:=true.
#   3. Critical topic rates (Fase A5)     — /agv/cmd_vel, /agv/safety/status,
#      /agv/odometry/global must hold their target Hz under simultaneous
#      GUI + perception load.
#   4. /tmp/cyclonedds.log scan           — looks for WHC overflow warnings.
#
# This script does NOT dispatch nav goals automatically — that's a manual
# step from the dashboard or `ros2 action send_goal`. The intent is to
# instrument the stack and report whether the throttling/watermark/cap
# changes from Sprint 1 actually held under load.
#
# Usage:
#   bash tools/sprint1_validate.sh [<jetson-host>]
# Defaults to localhost when no host is given.

set -u
HOST="${1:-localhost}"
IMG_PORT=8091
WS_PORT_FOXGLOVE=8765
HZ_SAMPLE_S=8
PASS=0; FAIL=0
status() { printf '%-50s %s\n' "$1" "$2"; }

echo "=== Sprint 1 validation against ${HOST} ==="

# 1. image_server stream cap (parallel — sequential never overlaps)
echo
echo "[1/4] image_server max_concurrent_streams cap (8 concurrent, account for existing clients)"
# Count existing ESTABLISHED streams to image_server BEFORE the test. If a
# real dashboard is open elsewhere, those slots are already taken; we need
# to subtract them from the expected served count so the test is robust on
# a busy system. CAP is the configured ceiling (default 4 in the binary).
CAP=4
existing=$(ss -tn 2>/dev/null | awk -v p=":${IMG_PORT}" '$1=="ESTAB" && $4 ~ p {n++} END {print n+0}')
expected_served=$(( CAP - existing < 0 ? 0 : CAP - existing ))
expected_rejected=$(( 8 - expected_served ))

TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT
for i in $(seq 1 8); do
    (curl -s -o /dev/null -w '%{http_code}\n' --max-time 5 \
        "http://${HOST}:${IMG_PORT}/camera/stream" \
        > "${TMP_DIR}/result_${i}.txt" 2>/dev/null) &
done
wait
ok_count=$(grep -l '^200$' "${TMP_DIR}"/result_*.txt 2>/dev/null | wc -l)
rejected_count=$(grep -l '^503$' "${TMP_DIR}"/result_*.txt 2>/dev/null | wc -l)
if [ "$ok_count" = "$expected_served" ] && [ "$rejected_count" = "$expected_rejected" ]; then
    status "  cap=${CAP} enforced (existing=${existing}, ok=${ok_count}, 503=${rejected_count})" "PASS"; PASS=$((PASS + 1))
else
    status "  cap=${CAP} (existing=${existing}, expected ok=${expected_served}/503=${expected_rejected}; got ok=${ok_count}/503=${rejected_count})" "FAIL"; FAIL=$((FAIL + 1))
fi

# 2. foxglove_bridge probe
echo
echo "[2/4] foxglove_bridge ws probe (only relevant if enable_foxglove:=true)"
if (echo > /dev/tcp/${HOST}/${WS_PORT_FOXGLOVE}) 2>/dev/null; then
    status "  TCP ${WS_PORT_FOXGLOVE} reachable" "PASS"; PASS=$((PASS + 1))
else
    status "  TCP ${WS_PORT_FOXGLOVE} unreachable (off or not enabled)" "SKIP"
fi

# 3. Critical topic rates (require ROS2 env sourced)
echo
echo "[3/4] critical topic rates (sampling ${HZ_SAMPLE_S}s each)"
if ! command -v ros2 >/dev/null 2>&1; then
    status "  ros2 not in PATH (source install/setup.bash)" "SKIP"
else
    check_hz() {
        topic="$1"; expected="$2"; allow_idle="${3:-no}"
        avg=$(timeout "${HZ_SAMPLE_S}" ros2 topic hz --window 50 "$topic" 2>&1 \
               | awk '/average rate/ {print $3; exit}')
        if [ -z "$avg" ]; then
            if [ "$allow_idle" = "idle_ok" ]; then
                status "  ${topic} (no data — idle stack)" "SKIP"
            else
                status "  ${topic}" "FAIL (no data)"; FAIL=$((FAIL + 1))
            fi
            return
        fi
        # Pass if avg >= 0.7 * expected (allow 30% slack under load).
        ratio=$(awk -v a="$avg" -v e="$expected" 'BEGIN { print (a/e) }')
        ok=$(awk -v r="$ratio" 'BEGIN { print (r >= 0.7) ? 1 : 0 }')
        if [ "$ok" = "1" ]; then
            status "  ${topic} avg=${avg}Hz target=${expected}Hz" "PASS"; PASS=$((PASS + 1))
        else
            status "  ${topic} avg=${avg}Hz target=${expected}Hz" "FAIL"; FAIL=$((FAIL + 1))
        fi
    }
    # /agv/cmd_vel: when idle (no operator input), mode_arbiter still relays
    # at ~10 Hz. The 20 Hz spec applies under active teleop/nav. Treat 10 Hz
    # idle baseline as PASS; absence as SKIP.
    check_hz /agv/cmd_vel              10  idle_ok
    check_hz /agv/safety/status        10
    check_hz /agv/odometry/global      10
fi

# 4. Cyclone log scan
echo
echo "[4/4] cyclonedds.log: WHC overflow / drop warnings"
if [ -f /tmp/cyclonedds.log ]; then
    overflow=$(grep -ciE 'WhcHigh|history.*overflow|writer.*dropp' /tmp/cyclonedds.log || true)
    if [ "$overflow" = "0" ]; then
        status "  no overflow/drop warnings" "PASS"; PASS=$((PASS + 1))
    else
        status "  ${overflow} suspicious lines (review log)" "FAIL"; FAIL=$((FAIL + 1))
    fi
else
    status "  /tmp/cyclonedds.log not present" "SKIP"
fi

echo
echo "=== summary: ${PASS} passed, ${FAIL} failed ==="
exit "$FAIL"
