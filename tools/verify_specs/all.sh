#!/bin/bash
# all.sh — run every verify_* script in order.
#
# Exit code is 0 only if every BLOCKING script returned 0. WARNING scripts
# print their issues but do not cause a non-zero exit.

set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

DIR="tools/verify_specs"

BLOCKING=(
  "$DIR/verify_canonical_sources.sh"
  "$DIR/verify_no_hardcoded_paths.sh"
  "$DIR/verify_werror.sh"
  "$DIR/verify_dev_only.py"
  "$DIR/verify_interfaces.py"
  "$DIR/verify_geometry_ssot.py"
)

WARNING=(
  "$DIR/verify_claude_md_coverage.sh"
  "$DIR/verify_persistence.py"
  "$DIR/verify_state_machine.py"
  "$DIR/verify_launch_sequence.py"
)

# Color helpers (no external dependencies).
red() { printf '\033[31m%s\033[0m' "$1"; }
green() { printf '\033[32m%s\033[0m' "$1"; }
yellow() { printf '\033[33m%s\033[0m' "$1"; }
bold() { printf '\033[1m%s\033[0m' "$1"; }

blocking_failures=0
warning_hits=0
ran=0

run_one() {
  local script="$1"
  local severity="$2"  # BLOCKING or WARNING
  if [ ! -r "$script" ]; then
    echo "$(yellow SKIP): $script not found"
    return 0
  fi
  echo
  echo "$(bold "── $(basename "$script") ──")"
  ran=$((ran + 1))

  local output
  local rc=0
  if [[ "$script" == *.py ]]; then
    output=$(python3 "$script" 2>&1) || rc=$?
  else
    output=$(bash "$script" 2>&1) || rc=$?
  fi

  echo "$output"

  if [ "$rc" -ne 0 ]; then
    if [ "$severity" = "BLOCKING" ]; then
      blocking_failures=$((blocking_failures + 1))
      echo "$(red "RESULT: BLOCKING FAIL")"
    else
      warning_hits=$((warning_hits + 1))
      echo "$(yellow "RESULT: WARNING")"
    fi
  else
    if echo "$output" | grep -qE '^WARN:'; then
      warning_hits=$((warning_hits + 1))
      echo "$(yellow "RESULT: WARNING")"
    else
      echo "$(green "RESULT: OK")"
    fi
  fi
}

for s in "${BLOCKING[@]}"; do
  run_one "$s" "BLOCKING"
done

for s in "${WARNING[@]}"; do
  run_one "$s" "WARNING"
done

echo
echo "$(bold "──────────── SUMMARY ────────────")"
echo "scripts run:       $ran"
echo "blocking failures: $blocking_failures"
echo "warnings:          $warning_hits"

if [ "$blocking_failures" -gt 0 ]; then
  echo
  echo "$(red "all.sh: FAILED — $blocking_failures blocking script(s) failed")"
  exit 1
fi

echo
echo "$(green "all.sh: OK (warnings=$warning_hits)")"
exit 0
