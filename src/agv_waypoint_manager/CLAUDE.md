# agv_waypoint_manager

C++17 ROS2 node for mission storage, listing, and sequential waypoint execution.
Persists missions as line-delimited JSON and dispatches goals via Nav2's
`/navigate_to_pose` action.

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
- Non-blocking execution: `execute` responds immediately; goals run in a worker
  thread that waits on action futures completed by the main executor (the
  worker never spins the node itself)
- `waypoint_manager/cancel` (std_msgs/Bool) preempts the in-flight
  `/navigate_to_pose` goal via `async_cancel_goal`, not just between waypoints
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
- Add pause/resume capability for running missions
- Add waypoint action support beyond navigation (pause, signal, custom actions)
