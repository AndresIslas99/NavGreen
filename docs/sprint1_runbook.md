# Sprint 1 — HMI separation + Jetson hardening (runbook)

**Status:** code complete 2026-04-24. Field validation pending (steps below).

This sprint implements the HMI architecture changes from the retro analysis
(`/home/orza/.claude/plans/recibi-la-siguiente-retro-quizzical-abelson.md`)
without changing the stack technology. The backend stays as it was validated
(rclnodejs + state machine + gates); the frontend gains the option to live
on a different host.

## What changed (per fase)

| Fase | Deliverable | Files touched |
|------|-------------|---------------|
| 0    | HMI ↔ backend contract spec | `specs/hmi_api.yaml`, `specs/README.md`, `specs/interfaces.yaml` |
| 1a   | Frontend host-agnostic + CORS allowlist | `web/agv_dashboard/src/api/client.ts`, `src/hooks/useWebSocket.ts`, 6× component files, `vite.config.ts`, `.env.example`, `src/agv_ui_backend/src/index.ts` (CORS middleware) |
| 2.A1 | image_server thread cap (`max_concurrent_streams`, default 4) | `src/agv_image_server/src/image_server_node.cpp` |
| 2.A2 | Cyclone WhcHigh 500kB→4MB, SocketReceiveBufferSize 26MB→64MB | `src/agv_bringup/scripts/agv_start.sh` (generator), `src/agv_bringup/config/cyclonedds_*.xml`, `src/agv_slam/config/cyclonedds.xml` |
| 2.A3 | `/scan` callback throttled to 5 Hz in backend | `src/agv_ui_backend/src/index.ts` |
| 3.A4 | foxglove_bridge optional node behind `enable_foxglove_bridge:=true` | `src/agv_bringup/launch/agv_full.launch.py`, `src/agv_bringup/package.xml` |
| 3.A5 | Stress-test runbook | `tools/sprint1_validate.sh` |
| 1c   | Migration script (this doc) | this file |

`/wheel_odom` was deliberately NOT throttled despite the plan: its callback
is trivial (timestamp push + two roundings) and throttling would misreport
`wheel_odom_hz` to the dashboard. Documented in `agv_ui_backend/CLAUDE.md`.

## Verifying on the Jetson dev (Fase 1b)

```bash
# Build (zero warnings is required by Rule 0)
cd ~/ros2_ws
colcon build --packages-select agv_image_server agv_bringup agv_slam
( cd src/agv_ui_backend && npm run build )
( cd web/agv_dashboard && npm run build )

# Specs in sync
bash tools/verify_specs/all.sh
# Expected: blocking failures: 0

# Start the stack with foxglove on for diagnostics
AGV_MODE=hil_full ./src/agv_bringup/scripts/agv_start.sh \
  enable_foxglove_bridge:=true map:=$AGV_DATA_DIR/maps/<your_map>.yaml

# In another shell, run the smoke runbook
bash tools/sprint1_validate.sh localhost
# Expected: 4× PASS (image cap, foxglove TCP, topic rates, no WHC warnings)
```

To verify CORS without the full stack: start the backend in isolation with
`AGV_UI_ALLOWED_ORIGINS=http://localhost:5173` and curl an OPTIONS preflight:

```bash
curl -i -X OPTIONS http://localhost:8090/api/status \
  -H 'Origin: http://localhost:5173' \
  -H 'Access-Control-Request-Method: GET'
# Expected: 204 No Content, Access-Control-Allow-Origin: http://localhost:5173
```

## Validating the stress test (Fase 3.A5)

While `agv_full.launch.py hil_mode:=true` is running:

1. From a second terminal: dispatch a Nav2 goal of 5+ metros (via the
   dashboard or `ros2 action send_goal /agv/navigate_to_pose ...`).
2. Open the dashboard, live map view, camera stream, and scan visualization
   in the browser concurrently while the goal executes.
3. Run `bash tools/sprint1_validate.sh` against the same Jetson.
4. Verify in `/slam/quality` topic that `tracking.confidence` does not drop
   below the iter-46 baseline. If `rail_approach` mean degrades >20%
   relative to iter-46 (4.5 cm), revert Fase A2.

## Migrating the frontend to your laptop (Fase 1c)

On the laptop (Ubuntu 22 x86):

```bash
git clone <repo> ~/ros2_ws-frontend   # only the web/ subdir is needed
cd ~/ros2_ws-frontend/web/agv_dashboard
cp .env.example .env.local
# Edit .env.local: VITE_API_BASE=http://<jetson-ip>:8090
nvm install 20 && nvm use 20          # if needed
npm install
npm run build

# Serve the dist/ with caddy or nginx. Caddy one-liner:
caddy file-server --listen :5173 --root dist
```

On the Jetson, set `AGV_UI_ALLOWED_ORIGINS` to allow your laptop's origin
(e.g., `http://laptop.lan:5173`) before starting the backend. Set this in
`/etc/systemd/system/agv.service` for production or via the launch arg env
for dev runs.

Validation (from the laptop):

```bash
# 1. CORS allowed
curl -i -X OPTIONS http://<jetson-ip>:8090/api/status \
  -H 'Origin: http://laptop.lan:5173' \
  -H 'Access-Control-Request-Method: GET'

# 2. Open dashboard
xdg-open http://laptop.lan:5173

# 3. End-to-end teleop latency: hit space-bar emergency-stop (or click
#    e-stop in the dashboard) and time the response. Target: <150 ms
#    (TASK.yaml).
```

If WiFi jitter persists after the migration, the next palanca is router
hardware (ASUS RT-AX55 ~$80 or Ubiquiti UniFi 6 Lite ~$300 as dedicated
point-to-point AP between laptop and Jetson). See plan §"Hardware paralelo".

## Reverting individual fases

Each fase is independently revertible via `git diff` of the listed files.
Cyclone XML watermarks revert to the previous values without rebuild
(runtime XML is regenerated next boot). The image_server thread cap
reverts by setting `max_concurrent_streams: 9999` at launch, no rebuild.

## What's deliberately out of scope

- Vulkan/Slint/Qt6 nativo HMI (regression of validated UI stack)
- rmw_zenoh adoption (in flux 2025; revisit Sprint 2+)
- Iceoryx PSMX intra-host shared memory (aarch64 fragmentation reports)
- Composing Nav2 nodes (Sprint 2 — high-value but lifecycle-tricky)
- H.264 NVENC pipeline replacing MJPEG (Sprint 2 — biggest BW reduction)
- Migrating state machine + gates to a C++17 `agv_hmi_arbiter` (post-MVP)

These are tracked in the plan's "Sprint 2" section.
