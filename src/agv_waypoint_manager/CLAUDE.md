# agv_waypoint_manager

C++17 ROS2 node for mission storage, listing, and sequential waypoint execution.
Persists missions as line-delimited JSON and dispatches goals via Nav2's
`/navigate_to_pose` action.

## ⚠️ NOT in the production launch (since Sprint B, 2026-05-13)

This node is **no longer started by `agv_full.launch.py`**. The dashboard
executes missions through `agv_ui_backend/src/index.ts::executeMission`,
which loops over waypoints and calls `ros.sendNavGoal` — that path
enforces the localization / motors-armed / collision-monitor gates per
`specs/state_machine.yaml#nav_goal_requires_localization`.

This package keeps building and its `/agv/waypoint_manager/execute`
service exists for CLI / integration-test invocation, but **no
production caller** uses it. A CLI caller invoking the service directly
bypasses every gate, so use it only in tests where that is acceptable.

If a future feature needs a ROS-side mission executor, refactor this
node's execute path to call the backend's HTTP endpoint instead, or
delete this package. Tracked as **HIGH-11-B-01** in
`docs/audit/2026-05-13-greenhouse-hardening/SUMMARY.md`. See also
`HIGH-11-B-02` (deadlock risk: `rclcpp::spin_until_future_complete`
from a worker thread on a single-threaded executor).

## Nodes

- **waypoint_manager_node** (C++17): Mission CRUD operations and sequential goal dispatch

## Services

- `waypoint_manager/save` (SaveWaypoint) — Persist mission (auto-generates ID if empty)
- `waypoint_manager/list` (ListMissions) — Return all stored missions
- `waypoint_manager/execute` (ExecuteMission) — Execute mission sequentially via navigate_to_pose

## Action Clients

- `/navigate_to_pose` (NavigateToPose) — Nav2 goal dispatch (5-minute timeout per waypoint)

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `missions_file` | (required) | Path to missions JSON file |
| `default_speed` | `0.3` | Default execution speed (m/s) |

## Key Implementation Details

- Missions serialized as line-delimited JSON (one mission per line, no external JSON lib)
- Sequential execution: blocks entire service call for duration of mission
- Quaternion from yaw: `z = sin(theta/2)`, `w = cos(theta/2)`
- Auto-generated mission IDs use timestamp-based scheme
- Creates missions directory on startup if it doesn't exist

## Configuration

- `config/waypoint_manager_params.yaml` — Missions file path and defaults

## Dependencies

- nav2_msgs, agv_interfaces, rclcpp, rclcpp_action

## Improvement Opportunities

- Replace manual JSON parsing with nlohmann::json or similar library
- Add waypoint pose validation (NaN, out-of-map-bounds)
- Make execution non-blocking (currently blocks service caller for entire mission)
- Add pause/resume capability for running missions
- Add waypoint action support beyond navigation (pause, signal, custom actions)
