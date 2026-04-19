#!/bin/bash
# verify_claude_md_coverage — every AGV package must have CLAUDE.md and TASK.yaml.

set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

# Packages that are internal to the AGV project. External submodules/third-party
# packages are excluded.
EXTERNAL=(
  zed-ros2-wrapper
  zed_components
  zed_wrapper
  zed_ros2
  isaac_ros_common
  isaac_ros_nitros
  isaac_ros_nova
  isaac_ros_nvblox
  isaac_ros_visual_slam
  ethz_nvblox
  negotiated
  zed-ros2-interfaces
)

is_external() {
  local p="$1"
  for e in "${EXTERNAL[@]}"; do
    [[ "$p" == "$e" ]] && return 0
  done
  return 1
}

warnings=0

for pkg_dir in src/*/; do
  pkg=$(basename "$pkg_dir")

  is_external "$pkg" && continue

  # Only check packages that look like real ROS packages (have package.xml).
  [ -f "${pkg_dir}package.xml" ] || continue

  if [ ! -f "${pkg_dir}CLAUDE.md" ]; then
    echo "WARN: $pkg_dir missing CLAUDE.md"
    warnings=$((warnings + 1))
  fi
  if [ ! -f "${pkg_dir}TASK.yaml" ]; then
    echo "WARN: $pkg_dir missing TASK.yaml"
    warnings=$((warnings + 1))
  fi
done

if [ "$warnings" -gt 0 ]; then
  echo "verify_claude_md_coverage: $warnings warning(s)"
  # WARNING severity — do not block commit, but report
  exit 0
fi

echo "verify_claude_md_coverage: OK"
exit 0
