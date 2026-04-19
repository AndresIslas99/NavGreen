#!/bin/bash
# AGV runtime watchdog — observes key liveness signals every WATCH_INTERVAL
# seconds and emits a non-zero exit (restarting the service via systemd's
# Restart=on-failure) when any of them stall long enough to matter.
#
# Signals watched:
#   - /agv/odometry/local publish rate (the dual-EKF local filter is the
#     heartbeat of the motion stack; if it dies, nothing else is trustworthy)
#   - /agv/cmd_vel_safe publisher presence (safety gate or Nav2 chain alive)
#   - RSS of the main ros2 launch process (catches the 28GB leak class of
#     failures before the Jetson OOM-kills random processes)
#
# Designed to be launched as a peer ExecStartPost=- of agv.service, or as a
# separate systemd unit with BindsTo=agv.service. It does NOT talk to
# sd_notify — systemd already handles Restart=on-failure correctly when we
# exit with a non-zero status.

# NOTE: no `set -u` — ROS 2 setup.bash references unset variables (AMENT_*),
# which would abort the script before the loop ever runs.

# Production domain — matches agv.service Environment=ROS_DOMAIN_ID=42.
# Without this export the watchdog runs in domain 0 and sees no AGV nodes.
# Canonical value in specs/project.yaml#deployment.ros_domain_id.
export ROS_DOMAIN_ID=42

source /opt/ros/humble/setup.bash 2>/dev/null || exit 2
source /home/orza/ros2_ws/install/setup.bash 2>/dev/null || exit 2

WATCH_INTERVAL="${AGV_WATCH_INTERVAL:-30}"
ODOM_MIN_RATE="${AGV_WATCH_ODOM_HZ:-30}"
RSS_MAX_MB="${AGV_WATCH_RSS_MAX_MB:-12288}"
STALL_BUDGET="${AGV_WATCH_STALL_BUDGET:-3}"
# Initial grace so we do not fire during bringup. agv_full sequences nodes up
# to ~8 s; give it 60 s to settle before the watchdog starts enforcing.
STARTUP_GRACE="${AGV_WATCH_STARTUP_GRACE:-60}"

echo "[watchdog] grace ${STARTUP_GRACE}s, then interval ${WATCH_INTERVAL}s"
sleep "$STARTUP_GRACE"

stall_count=0

tick() {
    local ok=1

    # Odometry rate — capture one rate sample from a 4 s window.
    local rate
    rate=$(timeout 6 ros2 topic hz /agv/odometry/local --window 30 2>&1 \
        | grep -m1 'average rate:' | awk '{print $3}')
    if [ -z "$rate" ]; then
        echo "[watchdog] odometry/local: no sample"
        ok=0
    elif ! awk -v r="$rate" -v m="$ODOM_MIN_RATE" 'BEGIN { exit !(r+0 >= m+0) }'; then
        echo "[watchdog] odometry/local: ${rate} Hz < ${ODOM_MIN_RATE} Hz"
        ok=0
    fi

    # cmd_vel_safe publisher presence.
    if ! timeout 3 ros2 topic info /agv/cmd_vel_safe 2>&1 \
            | awk '/Publisher count:/ {exit !($3 > 0)}'; then
        echo "[watchdog] /agv/cmd_vel_safe: no publisher"
        ok=0
    fi

    # RSS of the ros2 launch process (MainPID of agv.service).
    local main_pid rss_kb rss_mb
    main_pid=$(systemctl show -p MainPID --value agv.service 2>/dev/null)
    if [ -n "$main_pid" ] && [ "$main_pid" != "0" ]; then
        # Sum RSS across the launch process and every descendant — systemd's
        # MemoryMax is enforced on the cgroup, but for logging we want the
        # same number systemd sees.
        rss_kb=$(ps --ppid "$main_pid" -o rss= 2>/dev/null \
            | awk -v own="$(ps -o rss= -p "$main_pid" 2>/dev/null)" \
                  '{s+=$1} END {print s+own}')
        if [ -n "$rss_kb" ] && [ "$rss_kb" -gt 0 ]; then
            rss_mb=$((rss_kb / 1024))
            if [ "$rss_mb" -gt "$RSS_MAX_MB" ]; then
                echo "[watchdog] RSS ${rss_mb} MB > ${RSS_MAX_MB} MB"
                ok=0
            fi
        fi
    fi

    if [ "$ok" -eq 0 ]; then
        stall_count=$((stall_count + 1))
        echo "[watchdog] stall ${stall_count}/${STALL_BUDGET}"
    else
        stall_count=0
    fi

    if [ "$stall_count" -ge "$STALL_BUDGET" ]; then
        echo "[watchdog] stall budget exceeded — exiting nonzero"
        exit 1
    fi
}

while true; do
    tick
    sleep "$WATCH_INTERVAL"
done
