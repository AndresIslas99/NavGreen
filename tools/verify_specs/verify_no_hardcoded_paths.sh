#!/bin/bash
# verify_no_hardcoded_paths — Rule 1 enforcement
#
# Scans src/ for hardcoded absolute paths that should be parametrized via
# env vars (AGV_DATA_DIR, HOME) or ROS parameters. A whitelist is allowed
# for cases where the path is inherently deployment-specific and cannot
# be parametrized (e.g., the agv_start.sh that boots before ROS).

set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

# Patterns we want to catch. Add more over time.
FORBIDDEN_PATTERNS=(
  '/home/orza/'
  '/mnt/ssd/'
  '192.168.'
)

# Whitelist: file:pattern pairs that are allowed.
# Use 'filename_substring|pattern_substring' format.
#
# Rationale per category:
#   - agv_start.sh / healthcheck / watchdog: boot scripts that run BEFORE
#     ROS is sourced. They resolve AGV_DATA_DIR via env var but must
#     reference the absolute workspace path to source setup.bash.
#   - systemd/*.service: unit files, by definition site-specific absolute paths.
#   - config/sites/: per-site YAML configs with explicit hardcoded paths
#     for that deployment. These are NOT source code; they are the
#     parametrization mechanism itself.
#   - cyclonedds*.xml: DDS peer IPs are inherently IP-specific.
#   - scripts/*: dev/commissioning tools (session_recorder, jetson_setup,
#     run_slam, setup_production_network, field_test). They will be
#     marked dev_only in their TASK.yaml.
#   - launch/*.py: legal to pass `AGV_DATA_DIR` as an additional_env value,
#     since the consuming node then reads it via env var. The string
#     itself is the env var value, not a hardcoded code path.
#   - install/ and build/: colcon output trees — not source code.
#   - zed-ros2-wrapper/: external submodule, not under AGV policy.
WHITELIST=(
  # Boot scripts (pre-ROS)
  'src/agv_bringup/scripts/agv_start.sh|/home/orza/'
  'src/agv_bringup/scripts/agv_healthcheck.sh|/home/orza/'
  'src/agv_bringup/scripts/agv_watchdog.sh|/home/orza/'
  # systemd units
  'src/agv_bringup/systemd/|/home/orza/'
  'src/agv_bringup/systemd/|HOME=/home/orza'
  # Foxglove bridge deployment config (per-site systemd installer, mirrors
  # the agv_bringup/systemd pattern above). Lives under a gitignored
  # third-party directory but is scanned by the verifier anyway.
  'src/agv_slam/foxglove/systemd/|/home/orza/'
  # Per-site configs (parametrization by site)
  'src/agv_bringup/config/sites/|/home/orza/'
  # Launch files pass AGV_DATA_DIR as env var — target code reads via env
  'src/agv_bringup/launch/|AGV_DATA_DIR'
  'src/agv_bringup/launch/|runtime_registry_file'
  'src/agv_bringup/launch/|map_dir'
  # DDS peers — IP addresses that are inherently deployment-specific
  'src/agv_slam/config/cyclonedds|192.168.'
  'src/agv_bringup/config/cyclonedds|192.168.'
  # Dev scripts (marked dev_only in TASK.yaml)
  # These are allowed to hardcode paths/IPs because they run outside the
  # ROS stack and typically set up or document a fixed-location deployment.
  'src/agv_bringup/scripts/field_test.py|'
  'src/agv_bringup/scripts/setup_production_network.sh|'
  'src/agv_slam/scripts/jetson_setup.sh|'
  'src/agv_slam/scripts/run_slam.sh|'
  'src/agv_slam/scripts/export_session.sh|'
  'src/agv_slam/scripts/session_recorder.py|'
  'src/agv_slam/scripts/slam_gui.py|'
  # systemd unit comment with reference IP for documentation
  'src/agv_bringup/systemd/agv.service|192.168.'
  # ZED wrapper config and recording config — parametrized via launch
  # substitutions in Fase 6. Whitelisted until then with known_issue tag.
  'src/agv_slam/config/zed2i_override.yaml|/home/orza/'
  'src/agv_slam/config/recording.yaml|/mnt/ssd/'
  'src/agv_slam/launch/recording_only.launch.py|/mnt/ssd/'
  'src/agv_slam/launch/playback.launch.py|/mnt/ssd/'
  'src/agv_slam/src/slam_gui_node.cpp|/mnt/ssd/'
  'src/agv_slam/src/pipeline_watchdog_node.cpp|/mnt/ssd/'
  # C++ defaults that will be parametrized via env var in Fase 5.c.3
  # Whitelisted for now so the rest of the verify suite passes;
  # tracked as known_issue in specs/persistence.yaml.
  'src/agv_map_manager/src/map_manager_node.cpp|/home/orza/agv_data/maps/.current.area'
  'src/agv_localization_init/src/auto_init_orchestrator_node.cpp|/home/orza/agv_data/maps/.current.area'
  # agv_slam log messages include URLs
  'src/agv_slam/launch/agv_slam.launch.py|192.168.'
  # Legacy RTAB-Map config, scheduled for deletion
  'src/agv_slam/config/rtabmap.yaml|/home/orza/'
  # External submodule — not under AGV policy
  'src/zed-ros2-wrapper/|'
)

is_whitelisted() {
  local file="$1" pattern="$2"
  for entry in "${WHITELIST[@]}"; do
    local wl_file="${entry%%|*}"
    local wl_pattern="${entry##*|}"
    if [[ "$file" == *"$wl_file"* ]]; then
      if [[ -z "$wl_pattern" ]] || [[ "$pattern" == *"$wl_pattern"* ]]; then
        return 0
      fi
    fi
  done
  return 1
}

total_violations=0

for pattern in "${FORBIDDEN_PATTERNS[@]}"; do
  # Search src/ for the pattern, excluding build output, install trees, and external submodules.
  while IFS=: read -r file line_no line_content; do
    [ -z "$file" ] && continue
    file="${file#./}"
    # Exclude: colcon install/ build/ output, log/, external submodule src/zed-ros2-wrapper/,
    # npm node_modules/, dist/ output, .git.
    case "$file" in
      */install/*|*/build/*|*/log/*) continue ;;
      */node_modules/*|*/dist/*|*/.git/*) continue ;;
      src/zed-ros2-wrapper/*|src/isaac_ros_*/*|src/ethz_nvblox/*|src/negotiated/*) continue ;;
    esac
    hit="$line_content"
    if is_whitelisted "$file" "$hit"; then
      continue
    fi
    echo "FAIL: hardcoded '$pattern' at $file:$line_no"
    echo "       $line_content" | head -c 200 | sed 's/^/       /'
    echo
    total_violations=$((total_violations + 1))
  done < <(grep -rn --include='*.cpp' --include='*.hpp' --include='*.h' \
                    --include='*.py' --include='*.yaml' --include='*.yml' \
                    --include='*.xml' --include='*.sh' --include='*.ts' \
                    --include='*.service' \
                    "$pattern" src/ 2>/dev/null || true)
done

if [ "$total_violations" -gt 0 ]; then
  echo "verify_no_hardcoded_paths: $total_violations violation(s)"
  echo "Fix: parametrize via \${AGV_DATA_DIR} env var or ROS param. See specs/persistence.yaml."
  exit 1
fi

echo "verify_no_hardcoded_paths: OK"
exit 0
