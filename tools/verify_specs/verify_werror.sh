#!/bin/bash
# verify_werror — Rule 5 enforcement
#
# Every C++ package (src/*/CMakeLists.txt that references a .cpp source)
# MUST compile with -Werror. This is explicit in CLAUDE.md and
# /policies/engineering_rules.md.

set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

# Packages that are intentionally exempt (Python-only, external, or experimental).
EXEMPT=(
  agv_bringup          # launch-only, no C++
  agv_description      # URDF-only, no C++
  agv_integration_tests # test package
  zed-ros2-wrapper     # external submodule
  zed_components       # external
  zed_wrapper          # external
  zed_ros2             # external
  isaac_ros_common
  isaac_ros_nitros
  isaac_ros_nova
  isaac_ros_nvblox
  isaac_ros_visual_slam
  ethz_nvblox
  negotiated
)

is_exempt() {
  local pkg="$1"
  for e in "${EXEMPT[@]}"; do
    [[ "$pkg" == "$e" ]] && return 0
  done
  return 1
}

violations=0

# Only check AGV-owned packages (src/agv_*). External submodules (isaac_ros_*,
# ethz_nvblox, zed-ros2-wrapper, negotiated) are governed by upstream.
for cmake in $(find src -maxdepth 4 -name CMakeLists.txt 2>/dev/null); do
  pkg_dir=$(dirname "$cmake")
  pkg=$(basename "$pkg_dir")

  # Skip if not an agv_* package.
  case "$pkg" in
    agv_*) : ;;
    *) continue ;;
  esac

  is_exempt "$pkg" && continue

  # Does this package have .cpp sources?
  if ! find "$pkg_dir/src" -maxdepth 2 -name '*.cpp' 2>/dev/null | grep -q .; then
    continue
  fi

  if ! grep -q -- '-Werror' "$cmake"; then
    echo "FAIL: $cmake is missing -Werror"
    echo "       Fix: add -Werror to target_compile_options or add_compile_options"
    violations=$((violations + 1))
  fi
done

if [ "$violations" -gt 0 ]; then
  echo "verify_werror: $violations violation(s)"
  exit 1
fi

echo "verify_werror: OK"
exit 0
