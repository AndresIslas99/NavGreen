# agv_ui_backend

TypeScript ROS2 bridge server using Express + rclnodejs. Provides WebSocket real-time
telemetry and REST API for the operator dashboard. Includes state machine for robot
mode management and action guards.

## Architecture

- **Express HTTP server** (:8090) — REST endpoints for maps, missions, status
- **WebSocket** (5Hz broadcast) — Real-time telemetry, scan points, live map, control commands
- **rclnodejs** — ROS2 bridge (publishers, subscribers, action clients)
- **State machine** — Derives robot state from sensor data and controls allowed actions
- **Dashboard** — Served as static files from `web/agv_dashboard/dist/` at `/dashboard`

## Live Map Pipeline

```
scan_grid_mapper (C++) → /agv/live_map (OccupancyGrid, transient_local)
  → rclnodejs subscription (direct, no subprocess)
    → RGBA pixel conversion + vertical flip + sharp PNG compression
      → sequence counter guards against async race conditions
    → WebSocket (type: 'acc_map') → React Leaflet overlay
```

No Python subprocess, no file-based bridge, no ScanAccumulator. Single source of truth.

## ROS2 Subscriptions (rclnodejs direct)

- `/{ns}/wheel_odom` (Odometry) — Velocity extraction, odom rate tracking
- `/{ns}/odometry/global` (Odometry) — Robot pose for dashboard (from ekf_global)
- `/{ns}/scan` (LaserScan) — Scan points for real-time visualization (red dots)
- `/{ns}/plan` (Path) — Navigation path display
- `/{ns}/map` (OccupancyGrid, transient_local) — Static navigation map
- `/{ns}/live_map` (OccupancyGrid, transient_local) — Live mapping grid
- `/{ns}/battery` (BatteryState) — Battery percentage
- `/{ns}/zed/imu/data` (Imu) — IMU heartbeat tracking
- `/slam/quality` (String/JSON) — SLAM tracking confidence

**Subprocess workaround (rclnodejs DDS discovery bug):**
- `/{ns}/motor_state` (String/JSON, 2 Hz) — Motor arm state, errors, temps. Uses `ros2 topic echo` subprocess because rclnodejs fails to discover low-frequency C++ publishers.

## ROS2 Publishers

- `/{ns}/cmd_vel` + `/{ns}/cmd_vel_safe` (Twist) — Teleop commands (mode-specific limits: mapping 0.4 lin / 0.2 ang, teleop 0.5 / 0.5)
- `/{ns}/e_stop` (Bool) — Emergency stop
- `/{ns}/motor_enable` (Bool) — Motor arm/disarm
- `/{ns}/mode` (String) — Current mode (teleop/mapping/nav)

## ROS2 Action Clients

- `navigate_to_pose` (NavigateToPose) — Goal dispatch from dashboard

## State Machine

**States**: `offline`, `idle`, `ready`, `mapping`, `navigating`, `executing_mission`, `blocked`, `e_stop`, `fault`

**Priority order**: e_stop > fault > idle (no odom) > mapping > executing_mission > navigating > ready > idle

**Action guards** (per state): canTeleop, canStartMapping, canSendGoal, canExecuteMission, canSaveMap, canLoadMap, canMotorEnable, canCancelNav

## Key Modules

**TypeScript (src/):**
- `index.ts` — Main server, ROS2 node setup, all subscriptions, live map pipeline
- `state_machine.ts` — Robot state derivation and action guards
- `event_log.ts` — Circular event log (JSONL persistence)
- `telemetry_store.ts` — Time-series metrics with retention
- `app_deps.ts` — Shared dependencies interface
- `auth.ts` — JWT authentication manager
- `ws/control.ts` — WebSocket handler (5Hz status broadcast, per-client map versioning)
- `routes/` — REST API route handlers (maps, missions, nav, camera, status, auth, analytics, events, recording)

**Removed:**
- `scan_accumulator.ts` — Deleted. Was a JavaScript approximation of scan_grid_mapper that caused duplicate map overlays with different coordinates/resolution.
- `live_map_bridge.py` — No longer launched. Was a Python subprocess that bridged OccupancyGrid to file-based PNG. Replaced by direct rclnodejs subscription.

## Frontend (web/agv_dashboard/)

- **React + TypeScript + Vite** — Single-page dashboard
- **Leaflet** (CRS.Simple) — Map rendering with ImageOverlay (MapView.tsx)
- **Joystick** — 20Hz send rate, sends latest stick position (no accumulation), repeats zero 5x on release for guaranteed stop
- MapCanvas.tsx removed (was unused Canvas2D fallback)

## REST Endpoints

- `GET /api/status` — Robot state + health
- `GET/POST /api/maps` — Map CRUD
- `GET /api/acc_map/image` — Live map PNG (from state.liveMapPng)
- `DELETE /api/acc_map` — Clear live map (publishes to clear_map topic)
- `GET/POST/DELETE /api/missions` — Mission CRUD
- `POST /api/nav/cancel` — Cancel navigation
- `GET /api/events` — Event log
- `GET /api/analytics` — Telemetry data

## Dependencies

- rclnodejs, express, ws, sharp, nav2_msgs, agv_interfaces

## Improvement Opportunities

- Remove Python legacy modules (py_*.py files still in package, superseded by TypeScript)
- Add rate limiting on REST endpoints
- Add TypeScript unit tests for state machine logic
- Consider replacing motor_state subprocess with rclnodejs fix or rosbridge
