#!/bin/bash
# ROS2 executable wrapper for the TypeScript backend.
# Installed to lib/agv_ui_backend/ by CMakeLists.txt so that
# launch_ros.actions.Node() can launch it like any other ROS2 node.
#
# The compiled entry point (dist/index.js) must sit next to a node_modules/
# directory for Node's module resolution to work. Candidates are tried in
# priority order:
#   1. $AGV_UI_BACKEND_ROOT              — explicit deployment override
#   2. <prefix>/share/agv_ui_backend     — install space (CMake installs dist/;
#                                          node_modules must be provisioned there
#                                          for an install-only deployment)
#   3. <ws>/src/agv_ui_backend           — source tree, resolved for both
#                                          isolated and merged colcon installs
#
# In the install space SCRIPT_DIR is <prefix>/lib/agv_ui_backend, where
# <prefix> is <ws>/install/agv_ui_backend (isolated) or <ws>/install (merged).

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

CANDIDATES=()
[ -n "$AGV_UI_BACKEND_ROOT" ] && CANDIDATES+=("$AGV_UI_BACKEND_ROOT")
CANDIDATES+=("$SCRIPT_DIR/../../share/agv_ui_backend")   # install space (both layouts)
CANDIDATES+=("$SCRIPT_DIR/../../../../src/agv_ui_backend") # source tree, isolated install
CANDIDATES+=("$SCRIPT_DIR/../../../src/agv_ui_backend")    # source tree, merged install

for base in "${CANDIDATES[@]}"; do
    if [ -f "$base/dist/index.js" ] && [ -d "$base/node_modules" ]; then
        exec node "$base/dist/index.js" "$@"
    fi
done

echo "ERROR: Backend entry point not found (need dist/index.js + node_modules/)." >&2
echo "Searched:" >&2
for base in "${CANDIDATES[@]}"; do
    echo "  - $base" >&2
done
echo "Run 'cd src/agv_ui_backend && npm ci && npm run build', or set" >&2
echo "AGV_UI_BACKEND_ROOT to a directory containing dist/ and node_modules/." >&2
exit 1
