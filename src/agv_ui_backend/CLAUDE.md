# agv_ui_backend

TypeScript ROS2 bridge server using Express + rclnodejs. Provides WebSocket real-time
telemetry and REST API for the operator dashboard. Includes state machine for robot
mode management and action guards.

## Architecture

- **Express HTTP server** (:8090) — REST endpoints for maps, missions, status
- **WebSocket** — Real-time telemetry streaming and control commands
- **rclnodejs** — ROS2 bridge (publishers, subscribers, action clients)
- **State machine** — Derives robot state from sensor data and controls allowed actions

## ROS2 Publishers

- `/{namespace}/cmd_vel` (Twist) — Teleop joystick commands
- `/{namespace}/e_stop` (Bool) — Emergency stop signal
- `/{namespace}/motor_enable` (Bool) — Motor arm/disarm

## ROS2 Action Clients

- `navigate_to_pose` (NavigateToPose) — Goal dispatch from dashboard

## ROS2 Service Clients

- All agv_map_manager services (save/load map, update zone)
- All agv_waypoint_manager services (save/list/execute mission)

## State Machine

**States**: `offline`, `idle`, `ready`, `mapping`, `navigating`, `executing_mission`, `blocked`, `e_stop`, `fault`

**Priority order**: e_stop > fault > idle (no odom) > mapping > executing_mission > navigating > ready > idle

**Action guards** (per state): canTeleop, canStartMapping, canSendGoal, canExecuteMission, canSaveMap, canLoadMap, canMotorEnable, canCancelNav

## Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `AGV_PORT` | `8090` | HTTP server port |
| `AGV_NAMESPACE` | `"agv"` | ROS2 namespace |
| `AGV_DATA_DIR` | `"/tmp/agv_data"` | Data persistence directory |
| `AGV_MAPS_DIR` | `$DATA_DIR/maps` | Maps directory |
| `AGV_RETENTION_DAYS` | `30` | Telemetry retention |

## Key Modules

**TypeScript (src/):**
- `index.ts` — Main server, ROS2 node setup, WebSocket handlers
- `state_machine.ts` — Robot state derivation and action guards
- `event_log.ts` — Circular event log (JSONL persistence)
- `scan_accumulator.ts` — LaserScan buffering for live map
- `telemetry_store.ts` — Time-series metrics with retention
- `routes/` — REST API route handlers (maps, missions, nav, camera, status, auth, analytics, events, recording)

**Python legacy (agv_ui_backend/):**
- `py_state_machine.py`, `py_event_log.py`, `py_camera_handler.py`, `py_scan_accumulator.py`
- These are the original Python implementations, superseded by TypeScript

## REST Endpoints

- `GET /api/status` — Robot state + health
- `GET/POST /api/maps` — Map CRUD
- `GET/POST/DELETE /api/missions` — Mission CRUD
- `POST /api/nav/cancel` — Cancel navigation
- `GET /api/events` — Event log
- `GET /api/analytics` — Telemetry data

## Configuration

- `launch/teleop_web.launch.py` — Launch with namespace and port

## Dependencies

- rclnodejs, express, ws (WebSocket), nav2_msgs, agv_interfaces

## Improvement Opportunities

- Remove Python legacy modules (superseded by TypeScript but still in package)
- Add WebSocket authentication (currently open)
- Add rate limiting on REST endpoints
- Add health check endpoint for monitoring
- Add TypeScript unit tests for state machine logic
