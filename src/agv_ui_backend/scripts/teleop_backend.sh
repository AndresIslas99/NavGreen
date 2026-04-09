#!/bin/bash
# ROS2 executable wrapper for the TypeScript backend.
# Installed to lib/agv_ui_backend/ by CMakeLists.txt so that
# launch_ros.actions.Node() can launch it like any other ROS2 node.
#
# The wrapper resolves the compiled TypeScript entry point relative
# to the package source tree and execs node with any remaining args.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# In the install space, SCRIPT_DIR = <ws>/install/agv_ui_backend/lib/agv_ui_backend
# The source dist lives at <ws>/src/agv_ui_backend/dist/index.js
WS_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
ENTRY="$WS_ROOT/src/agv_ui_backend/dist/index.js"

if [ ! -f "$ENTRY" ]; then
    echo "ERROR: Backend entry point not found: $ENTRY" >&2
    echo "Run 'cd src/agv_ui_backend && npx tsc' to build." >&2
    exit 1
fi

exec node "$ENTRY" "$@"
