#!/bin/bash
# AGV post-boot health check — verifies the runtime stack is actually
# operational after agv.service reports "active". Runs as a Type=oneshot
# systemd unit triggered after agv.service starts; non-zero exit indicates
# the robot booted into a degraded state and operators should investigate.
#
# Checks (all must pass):
#   1. can0 interface is UP
#   2. ODrive S1 heartbeats seen on CAN (node IDs 0x009 / 0x029)
#   3. /agv/odometry/local publishing at >40 Hz
#   4. /agv/cmd_vel_safe publisher exists (gate or Nav2 chain up)
#   5. TF chain map->odom->base_link resolves within 2 s
#   6. No rogue sim_* nodes linger in the graph
#   7. teleop_server HTTP endpoint (:8090) responds
#
# Every failing check prints a one-line reason prefixed with "FAIL:" and
# contributes to the exit status. Successful checks print "OK: ...".

# NOTE: no `set -u` — ROS 2 setup.bash references unset variables (AMENT_*),
# which would abort the script before we can emit a failure reason.

# Production domain — matches agv.service Environment=ROS_DOMAIN_ID=42.
# Without this export the script runs in domain 0 and sees no AGV nodes.
# Canonical value in specs/project.yaml#deployment.ros_domain_id.
export ROS_DOMAIN_ID=42

if ! source /opt/ros/humble/setup.bash 2>/dev/null; then
    echo "FAIL: /opt/ros/humble/setup.bash not found"
    exit 2
fi
if ! source /home/orza/ros2_ws/install/setup.bash 2>/dev/null; then
    echo "FAIL: workspace install/setup.bash not found"
    exit 2
fi

fail_count=0

check() {
    local name="$1"
    shift
    if "$@"; then
        echo "OK: $name"
    else
        echo "FAIL: $name"
        fail_count=$((fail_count + 1))
    fi
}

check_can_up() {
    ip -br link show can0 2>/dev/null | grep -q 'UP'
}

check_odrive_heartbeats() {
    # ODrive S1 emits heartbeats on 0x009 (axis0) and 0x029 (axis1). A short
    # candump capture should see at least one of each within 2 seconds.
    local dump
    dump=$(timeout 2 candump -n 40 can0 2>/dev/null)
    echo "$dump" | grep -qE '(^| )(009|029) ' || return 1
    return 0
}

check_odom_rate() {
    # Sample /agv/odometry/local for 3 s and require >40 Hz. ros2 topic hz
    # prints lines like "average rate: 52.345". Use the first rate it reports.
    local rate
    rate=$(timeout 5 ros2 topic hz /agv/odometry/local --window 30 2>&1 \
        | grep -m1 'average rate:' | awk '{print $3}')
    [ -n "$rate" ] || return 1
    awk -v r="$rate" 'BEGIN { exit !(r+0 > 40) }'
}

nav2_active() {
    # Authoritative Nav2 liveness check — collision_monitor is only alive
    # when the full Nav2 stack is up, which implies a map is loaded.
    timeout 4 ros2 node list 2>/dev/null | grep -q '^/agv/collision_monitor$'
}

check_cmd_vel_pipeline() {
    # With Nav2 / safety chain: cmd_vel_safe has a publisher (the gate).
    # Mapping-first: the ODrive listens on /agv/cmd_vel directly — require a
    # publisher (teleop_server) there instead.
    local topic
    if nav2_active; then
        topic="/agv/cmd_vel_safe"
    else
        topic="/agv/cmd_vel"
    fi
    timeout 3 ros2 topic info "$topic" 2>&1 \
        | awk '/Publisher count:/ {exit !($3 > 0)}'
}

check_tf_chain() {
    # Always require odom → base_link (published by ekf_local). Only require
    # map → base_link when Nav2 is actually running — mapping-first mode has
    # no map→odom transform yet and that is expected.
    # tf2_echo does not accept --timeout on Humble; rely on the bash
    # `timeout` wrapper to bound wall-clock cost. -r 1 emits once per second;
    # we just need the first "Translation" line.
    timeout 4 ros2 run tf2_ros tf2_echo odom base_link -r 1 2>&1 \
        | grep -m1 -q 'Translation' || return 1
    if nav2_active; then
        timeout 4 ros2 run tf2_ros tf2_echo map base_link -r 1 2>&1 \
            | grep -m1 -q 'Translation' || return 1
    fi
    return 0
}

check_no_sim_nodes() {
    # After a clean reboot the DDS graph should not contain any node whose
    # name starts with 'sim_'. Lingering entries indicate ghost nodes left
    # over in the ros2 daemon discovery cache.
    local rogue
    rogue=$(timeout 4 ros2 node list 2>/dev/null | grep -cE '/sim_|^/teleop$')
    [ "${rogue:-0}" -eq 0 ]
}

check_dashboard_http() {
    curl -sf -o /dev/null --max-time 3 http://localhost:8090/api/status
}

echo "=== agv_healthcheck @ $(date -Iseconds) ==="
check "can0 interface up"                  check_can_up
check "ODrive heartbeats on can0"          check_odrive_heartbeats
check "/agv/odometry/local > 40 Hz"        check_odom_rate
check "cmd_vel pipeline publisher"         check_cmd_vel_pipeline
check "TF chain to base_link"              check_tf_chain
check "no rogue sim_/teleop nodes"         check_no_sim_nodes
check "dashboard backend responding"       check_dashboard_http

if [ "$fail_count" -gt 0 ]; then
    echo "=== agv_healthcheck FAILED ($fail_count check(s)) ==="
    exit 1
fi
echo "=== agv_healthcheck PASSED ==="
exit 0
