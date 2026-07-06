# agv_ui_backend

TypeScript ROS2 bridge server using Express + rclnodejs. Provides WebSocket real-time
telemetry and REST API for the operator dashboard. Includes state machine for robot
mode management and action guards.

## ⚠️ rclnodejs cache invalidation after agv_interfaces changes

`rclnodejs` caches its IDL bindings in
`src/agv_ui_backend/node_modules/rclnodejs/generated/` on first init.
**The cache is NOT invalidated automatically** when `agv_interfaces/srv/*.srv`
or `agv_interfaces/msg/*.msg` are edited or new fields added.

If you change a custom interface, the backend will crash on startup with
errors like:

```
NotFoundError: Cannot find ROS message: ...
  packageName: 'agv_interfaces',
  type: 'srv',
  messageName: 'RailApproach',
  searchPath: 'node_modules/rclnodejs/generated/'
```

**Fix:** clear the cache and restart the service:

```bash
rm -rf src/agv_ui_backend/node_modules/rclnodejs/generated/*
sudo systemctl restart agv.service
```

The first boot after the clear takes 2-3 min extra to regenerate
bindings for every interface package on the workspace. Subsequent
boots are normal speed because the cache persists.

## Architecture

- **Express HTTP server** (:8090) — REST endpoints for maps, missions, status
- **WebSocket** (5Hz broadcast) — Real-time telemetry, scan points, live map, control commands
- **rclnodejs** — ROS2 bridge (publishers, subscribers, action clients)
- **State machine** — Derives robot state from sensor data and controls allowed actions
- **Dashboard** — Served as static files from `web/agv_dashboard/dist/` at `/dashboard` (legacy/fallback). Sprint 1 Fase 1a decoupled the dashboard so it can be hosted from a different origin (laptop nginx/caddy). The frontend reads `VITE_API_BASE` to find this backend; default empty preserves same-origin.

## CORS for externally-hosted frontend (Sprint 1 Fase 1a)

When the dashboard is served from a host other than this backend, set
`AGV_UI_ALLOWED_ORIGINS` to a comma-separated list of allowed origins
(e.g., `http://laptop.lan:5173,http://192.168.1.42:5173`). Empty (default)
keeps same-origin-only behavior. The middleware lives in `index.ts` after
`express.json()` and handles preflight OPTIONS automatically. The contract
between HMI and this backend is documented in `specs/hmi_api.yaml`.

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
- `/{ns}/scan` (LaserScan) — Scan points for real-time visualization (red dots). **Throttled to 5 Hz** in the callback (Sprint 1 Fase A3): publisher is ~30 Hz but the WS broadcast is 5 Hz, so processing every frame burned CPU for nothing. `/wheel_odom` is intentionally NOT throttled — its callback is trivial and reporting Hz must reflect the publisher.
- `/{ns}/plan` (Path) — Navigation path display
- `/{ns}/map` (OccupancyGrid, transient_local) — Static navigation map
- `/{ns}/live_map` (OccupancyGrid, transient_local) — Live mapping grid
- `/{ns}/battery` (BatteryState) — Battery percentage
- `/{ns}/zed/imu/data` (Imu) — IMU heartbeat tracking
- `/slam/quality` (String/JSON) — SLAM tracking confidence
- `/{ns}/motor_state` (String/JSON, 10 Hz) — Motor arm state, errors, temps. Native rclnodejs subscription; the previous `ros2 topic echo` subprocess workaround was removed because it caused arm/disarm transitions to only reach the UI after a page reload (stdout buffering delayed the first messages by seconds).
- `/{ns}/rail_approach/state` (String/JSON) — rail_approach FSM state for mission waypoint gating and the dashboard rail panel. NOTE: the topic is `state`, not the pre-arbiter `status` name (see `specs/interfaces.yaml`).

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
- `scripts/teleop_server.py`, `scripts/live_map_bridge.py`, `launch/teleop_web.launch.py` — Deleted (superseded Python backend). `teleop_server.py` imported `py_*` modules that no longer existed and was never installed by CMakeLists, so the launch file failed on every machine. The TypeScript backend (`teleop_backend`) is the only entry point; git history preserves the Python originals.

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

## Authentication

- No default accounts ship with the repo. Create users with
  `npm run adduser -- <username> <password> [viewer|operator|engineer]`
  (writes scrypt-hashed credentials to `$AGV_DATA_DIR/users.json`), then set
  `"enabled": true` in that file.
- Legacy unsalted SHA-256 hashes in an existing `users.json` still verify and
  are transparently upgraded to scrypt on the next successful login.
- Mutating REST endpoints (nav goal, missions, maps, mode, e-stop clear, tags,
  recording) require the `operator` role when auth is enabled. Stop-type
  endpoints (`/api/nav/cancel`, `/api/recovery/trigger_estop`,
  `/api/missions/pause`) stay unauthenticated so the robot can always be stopped.
- The backend logs a loud `[SECURITY]` banner at startup when auth is disabled
  or when publicly-known default credentials are still present.

## Improvement Opportunities

- Add rate limiting on REST endpoints
- Add TypeScript unit tests for state machine logic
