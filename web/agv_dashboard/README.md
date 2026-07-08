# AGV Dashboard

Operator HMI for the greenhouse AGV (React + TypeScript + Vite, ISA-101-inspired
industrial design). It provides:

- Live occupancy-grid map with robot pose, planned path, and laser scan overlay
- Teleop joystick and motor arm/disarm controls
- Mapping workflow (start/stop recording, save/load maps)
- Mission creation and execution (waypoint capture on the map)
- Recovery panel with E-stop, health, and nav-cancel
- Analytics (mission history, pose replay), AprilTag management
- Multi-robot fleet overlay fed by the fleet manager

## Relationship to the backend

The dashboard is a pure frontend. All robot state and commands flow through
[`src/agv_ui_backend`](../../src/agv_ui_backend/) (Express + rclnodejs, port
8090 by default):

- REST under `/api/*` (auth, status, mode, maps, missions, nav, analytics)
- WebSocket `/ws/control` for live status, map updates, teleop, and E-stop
- The fleet manager / image server (port 8091 by default) serves the
  `/ws/fleet` stream and the traffic-zone API

In production the build output in `dist/` is served by the backend itself at
`http://<jetson>:8090/dashboard`.

## Development

```bash
npm install
npm run dev      # Vite dev server on :5173, proxies /api and /ws to :8090
npm run build    # type-check + production bundle in dist/
npm run lint
```

The dev-server proxy targets `http://localhost:8090`; set
`VITE_DEV_PROXY_TARGET` to develop against a backend on another host (e.g. the
Jetson).

## Configuration

All configuration is via Vite environment variables read at **build time** —
see [.env.example](.env.example) for the full list:

- `VITE_API_BASE` — backend HTTP/WS origin (empty = same-origin)
- `VITE_FLEET_BASE` — fleet manager / image server origin (default `:8091`)
- `VITE_BASE_PATH` — public path (default `/dashboard/` for the Express mount)

When the backend has authentication enabled, the dashboard shows a login page
and sends the session token as a `Bearer` header on REST calls and as
`?token=` on WebSocket connections.

Package contract: see [TASK.yaml](TASK.yaml).
