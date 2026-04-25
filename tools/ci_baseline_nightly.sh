#!/usr/bin/env bash
# Nightly baseline regression harness.
#
# Phase 5 of the diff-drive calibration plan. Runs a HIL session,
# captures a baseline CSV with the same script the operator uses
# manually (tools/calib_diff_drive_baseline.py), then compares the
# median Δodom/Δtag against the most recent committed baseline. Fails
# (exit non-zero) if the regression exceeds a threshold.
#
# This script is intended to run from CI on a Jetson with the HIL
# stack available. It is not a unit test — it requires a live ROS
# graph + a virtual operator (HIL controller) able to drive the robot.
#
# At the time of writing, the HIL operator integration is not yet
# automated; this script therefore documents the intended interface
# and is gated behind AGV_CI_BASELINE_LIVE=1. Without that flag, it
# performs only the static linting / parsing checks.

set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)
RUNS_DIR="${ROOT}/tools/calib_runs"
HIST="${ROOT}/docs/calibration/history.md"

REGRESSION_PCT_THRESHOLD="${REGRESSION_PCT_THRESHOLD:-5.0}"  # bias change >5% wrt last row → fail

PASS=0; FAIL=0
status() { printf '%-60s %s\n' "$1" "$2"; }

echo "=== Phase 5 nightly baseline harness ==="

# 1. Static checks: history file exists, has rows, latest CSV is parsable
if [ ! -f "$HIST" ]; then
    status "history file present" "FAIL"
    FAIL=$((FAIL + 1))
else
    status "history file present" "PASS"; PASS=$((PASS + 1))
fi

# 2. Most recent CSV in calib_runs/ (excluding bag dirs)
latest_csv=$(ls -1t "${RUNS_DIR}"/baseline_*.csv 2>/dev/null | head -1)
if [ -z "$latest_csv" ]; then
    status "latest baseline CSV present" "SKIP (no baseline yet)"
else
    status "latest baseline CSV present: $(basename $latest_csv)" "PASS"; PASS=$((PASS + 1))
    # Quick sanity: header line exists, at least one data row
    rows=$(grep -cv '^#\|^leg,\|^$' "$latest_csv" 2>/dev/null || echo 0)
    if [ "${rows:-0}" -lt 1 ]; then
        status "  CSV has data rows ($rows)" "FAIL"
        FAIL=$((FAIL + 1))
    else
        status "  CSV has ${rows} data rows" "PASS"; PASS=$((PASS + 1))
    fi
fi

# 3. Live HIL run, only if explicitly requested
if [ "${AGV_CI_BASELINE_LIVE:-0}" = "1" ]; then
    echo
    echo "=== Live HIL run ==="
    # TODO when HIL operator is automated: launch HIL stack, run virtual
    # operator path, run calib_diff_drive_baseline.py, compare against
    # last row of history.md, fail if regression exceeds threshold.
    status "live HIL run not implemented yet" "SKIP"
fi

echo
echo "=== summary: ${PASS} passed, ${FAIL} failed ==="
exit "$FAIL"
