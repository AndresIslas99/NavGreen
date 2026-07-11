# Commissioning Walkthrough — Non-ROS Technician → First Mission

> Companion to `11_hmi_and_commissioning.md`. Traces the end-to-end
> path a field technician follows from "robot box opened" to "first
> autonomous mission". For each step: what is present today, what is
> missing, where the flow forces a terminal.
>
> The acceptance criterion from the prompt ("<2 h, no terminal") is
> not met. This document is the structural diagnosis; the per-step
> fixes are spread across the Phase 11 findings.

## Methodology

For each of 7 commissioning steps:

- **State today**: what exists in the codebase, with file references.
- **Operator experience**: what the technician sees / does.
- **Structural breaks**: places where the operator must drop to a
  terminal, where validation is absent, or where the operator is
  expected to know something the dashboard does not teach.
- **Linked findings**: which Phase 11.A–D findings cover the fix.

## Step 1 — Power-on and first connection

**State today**:
- Robot boots via `agv.service` running `src/agv_bringup/scripts/agv_start.sh`.
- The Jetson exposes the backend on port `:8090` (per
  `specs/project.yaml#deployment.default_dashboard_port`).
- The operator must know the Jetson's IP address.
- No mDNS/Avahi/Bonjour: `agv_start.sh` does not invoke `avahi-publish`.
- No QR-code-on-chassis pointer to the robot's hostname.
- The static IP is set by `setup_production_network.sh` (per Phase 0
  inventory), but that script is run **on the Jetson** by an
  engineer during initial provisioning, not by the operator.

**Operator experience**: open browser → "what URL?" Operator must
have been told the IP, or must hunt through router admin UI, or
must use `nmap` on the LAN. **Drops to terminal.**

**Structural break**: there is no "first-time discovery" mechanism.
The dashboard URL is a tribal knowledge handoff from engineer to
operator.

**Fix path**: add mDNS broadcast in `agv_start.sh` so the dashboard
is reachable at `http://agv.local:8090` from any device. Print a QR
code on the chassis pointing at `agv.local:8090`. **Not a finding
filed in Phase 11 because it is a deployment/packaging concern**;
file as Sprint D follow-up.

## Step 2 — Authentication & first password set

**State after Sprint A.5 + Sprint E.lite (CRITICAL-11-C-01 / HIGH-11-D-01 closed)**:

- `src/agv_ui_backend/src/auth.ts` defaults to `enabled: true`. The
  first boot with no `users.json` on disk generates a random 16-char
  admin password (~95 bits entropy), writes it to the file, and logs
  it ONCE to the systemd journal:

  ```
  ═════════════════════════════════════════════════════
    agv_ui_backend: FIRST BOOT — admin credentials generated.
    username: admin
    password: <random 16 chars>
    Record this password NOW. It is logged only once.
    Change at first login via the dashboard prompt.
  ═════════════════════════════════════════════════════
  ```

- The admin user is created with `must_change_password: true`. The
  login endpoint returns that flag, the dashboard renders an in-place
  "Set a new password" form, and the token is held in component state
  until the change succeeds. Reloading the page does NOT bypass the
  prompt (token isn't persisted until the change commits).

- `App.tsx` fails CLOSED on `/api/auth/status` errors: the operator
  sees "Backend unreachable" with a manual Retry button, never an
  anonymous session.

**Operator experience (new install)**: opens dashboard → "Set a new
password" modal → enters a ≥8-char password → enters the dashboard.

**Migration from legacy deployment** (a Jetson with the pre-fix
`users.json` containing `enabled:false` and/or the legacy hashes for
`engineer:agv2026` / `operator:agv`):

```bash
# 1. Stop the service so the file is quiescent.
sudo systemctl stop agv.service

# 2. Back up the existing users.json. Keep this file off-system; it
#    holds the JWT secret and password hashes for the legacy accounts.
cp /home/orza/agv_data/users.json /home/orza/agv_data/users.json.bak-$(date +%F)

# 3. Delete the live file. The next service start will regenerate it.
rm /home/orza/agv_data/users.json

# 4. Restart.
sudo systemctl start agv.service

# 5. Grab the new random admin password from the journal (logged once):
sudo journalctl -u agv.service --since "1 minute ago" | grep -A 5 "FIRST BOOT"

# 6. Log in via the dashboard with admin / <random>; set a new password.
```

This rotates away from the legacy hardcoded credentials and forces a
new password without re-creating the legacy users.

**Linked findings**: `CRITICAL-11-C-01` (closed Sprint A.5 + E.lite),
`HIGH-11-D-01` (closed Sprint A.5), `HIGH-11-C-02` (salted KDF —
DEFERRED to Sprint future).

## Step 3 — Initial calibration

**State today**:
- Calibration scripts exist in `tools/`:
  - `tools/calib_diff_drive_baseline.py` — wheel radius / track width
  - `tools/calib_umbmark.py` — UMBmark wheel odometry residual
  - `tools/calib_apriltag_probe.py` — AprilTag extrinsic probe
  - `tools/calib_motor_ff_*.py` — motor feed-forward
  - `tools/solvepnp_noise_benchmark.py` — PnP noise characterization
- Manuals in `docs/calibration/`:
  - `baseline_protocol.md`
  - `umbmark_protocol.md`
  - `slip_detector_tuning.md`
  - `caster_dwell_advisor.md`
  - `odrive_nvram_dump_procedure.md` (shipped in Sprint A)
- Sprint A scaffolded the geometry SSOT but explicitly deferred the
  NVRAM dump to a terminal (`odrivetool`).
- **No dashboard panel for calibration. No backend route for
  calibration triggers.**

**Operator experience**: must SSH into the Jetson, navigate to
`tools/`, source ROS, run the script, manually copy the result to
`src/agv_description/config/robot_geometry.yaml`, commit, rebuild.

**Structural break**: the calibration flow is **entirely terminal-based**.
Sprint A's `docs/calibration/odrive_nvram_dump_procedure.md` says
exactly this: "30 minutes from start to a complete dump file" using
`odrivetool` in a terminal. The acceptance criterion <2 h is unreachable
because step 3 alone consumes most of the budget and requires expert
knowledge.

**Linked findings**: `MEDIUM-11-D-05` (no calibration wizards). XL
effort to close. Scope as Sprint E.

## Step 4 — Mapping the first greenhouse

**State today**:
- Dashboard has `MappingPanel` (per `App.tsx:14`).
- Reading the panel structure (App.tsx:135-144) — it accepts:
  - `state` (robot state)
  - `actions` (allowed action map)
  - `motorsArmed`
  - Callbacks: `onModeChange`, `onRecording`, `onCmdVel`
- Mapping mode is selected via `onModeChange('mapping')`, which
  publishes `/agv/mode` → backend → arbiter routes accordingly.
- During mapping, the operator drives via joystick at reduced limits
  (`maxLin: 0.4`, `maxAng: 0.2` per `index.ts:249-250`).
- `scan_grid_mapper` builds a live overlay on `/agv/live_map`,
  rendered in the MapView component.
- Save Map via REST `POST /api/maps/save` → calls
  `agv_map_manager/save_map` ROS service.
- Per `agv_navigation/CLAUDE.md` and the auto_init_orchestrator,
  saving a map also produces:
  - `<X>.yaml` + `<X>.pgm` (Nav2 map)
  - `<X>.area` (ZED Area Memory)
  - `<X>_cuvslam/` (cuVSLAM keyframe DB)
  - `<X>_meta.json` (last-known-pose)

**Operator experience**: this is the **best-supported workflow**. The
operator can:
1. Click "Mapping" in mode selector.
2. Joystick around the greenhouse.
3. Watch the live overlay form.
4. Click "Save Map" with a name.
5. Switch back to "Nav" or "Teleop".

**Structural breaks**:
- No "you've covered N% of the greenhouse" indicator. Operator
  doesn't know when to stop. (Phase 6 missing-decay finding
  `MEDIUM-06-03` relates.)
- No validation that the saved map is **good** — no loop closure
  count, no AprilTag count, no area covered. The operator might
  save an incomplete map and not realize until first nav fail.
- Map naming: free-form string, no enforcement of uniqueness, no
  warning if the name already exists. (Not yet filed; minor.)
- "Save Map" is single click → starts a long-running operation
  (10+ seconds for the cuVSLAM keyframe DB to flush). UI feedback
  during that time is minimal (the panel shows the request as
  pending, but the WS status doesn't indicate "saving").

## Step 5 — Annotating the map

**State today**:
- `agv_map_manager` supports keepout zones via
  `/agv/map_manager/update_zone` service (per
  `specs/interfaces.yaml`).
- Speed zones are mentioned in `specs/project.yaml#scope.in_scope`
  ("map_editing — Edit keepout zones and speed zones from web UI")
  but the dashboard's map editor surface is unclear from the static
  panel list (no `KeepoutPanel.tsx` or similar in
  `web/agv_dashboard/src/components/panels/`).
- AprilTag layout management is handled by `AprilTagsPanel.tsx` and
  the `apriltag_manager.ts` backend module.
  - The flow: operator drives near a tag, the AprilTag detection
    pipeline publishes a `pending_apriltag` WebSocket message with
    the hardware ID, `App.tsx:242` shows the
    `AprilTagAssignmentModal` — operator labels the tag.
- Docking poses are tied to `agv_rail_approach` and the rail_start
  AprilTag layout. There's a "rail_start" tag type
  ([index.ts:489-499](../../../src/agv_ui_backend/src/index.ts))
  with auto-trigger for rail_approach. Setup happens through the
  AprilTag panel.

**Operator experience**:
- AprilTag labeling is **well-supported** via the assignment modal.
- Keepout / speed zone drawing is **unclear**. The static panel list
  shows no zone-editor panel. The backend service exists but the
  frontend may not call it. Need to read `MissionsPanel.tsx` and
  `MapView.tsx` for drawing tools — not done in this audit.

**Structural break**: Keepout-zone editor presence is **TBD without
deeper UI audit**. This audit cycle marks it as an open question.

**Linked findings**: `MEDIUM-11-D-05` (calibration wizards) is the
closest analogue; "no keepout-zone wizard" would be a separate
finding if confirmed absent. Not filed pending UI-deep audit.

## Step 6 — Defining the first mission

**State today**:
- `MissionsPanel.tsx` exists.
- Backend route `POST /api/missions` accepts a JSON body with
  `waypoints` (or `nodes`).
- `App.tsx:111-117` `handleGoalClick` captures waypoints when
  `capturingWaypoints == true` (set by the "Start Capture" button in
  MissionsPanel per the props at `App.tsx:151-155`).
- Mission persistence is fine (per Phase 11.B audit).
- **No waypoint validation** (filed as `MEDIUM-11-B-05`).
- **No "preview path" before execute**.
- **No drag-to-reorder** of waypoints (TBD without deeper UI audit;
  the props show `pendingWaypoints` as a flat array, no reorder
  handler visible in App.tsx).

**Operator experience**: click "Start Capture", click on map to add
waypoints, click "Save Mission" with a name, switch to nav mode,
click "Execute" — the mission runs.

**Structural breaks**:
- Single-click on map drops a waypoint with `theta=0`
  (`App.tsx:113`). No way to set heading per waypoint from the map.
- Validation gap (`MEDIUM-11-B-05`): NaN, out-of-map, in-keepout
  coordinates all accepted.
- No preview: operator sees individual waypoint dots but no expected
  Nav2 plan between them.

**Linked findings**: `MEDIUM-11-B-05`, `MEDIUM-11-D-04`.

## Step 7 — Executing and monitoring the first mission

**State today**:
- `POST /api/missions/:id/execute` → `deps.executeMission`
  ([index.ts:461-545](../../../src/agv_ui_backend/src/index.ts)).
- Loop: for each waypoint, `ros.sendNavGoal` → poll
  `state.navState.active` until completion → next.
- Mission progress published on WS status as
  `mission_progress: { current_node, total_nodes, status }`.
- Cancel: `POST /api/missions/pause` and `/resume` flags toggle
  `state.missionPause`; cancel via `state.missionCancel`.
- Mission Pause / Resume buttons in MissionsPanel set the flags.
- **Pause flag is only consumed by backend's executor** — if
  `waypoint_manager_node` is the executor (alternative path,
  HIGH-11-B-01), pause is a no-op.
- WebSocket disconnect does NOT pause the mission
  (`MEDIUM-11-C-06`).

**Operator experience**: starts the mission, watches the
`mission_progress` indicator (presumably surfaced in MissionsPanel
or WaypointBatteryPanel), can pause / resume / cancel.

**Structural breaks**:
- If the operator's connection drops mid-mission, the mission
  continues without supervision (HAZOP `H-07`, `MEDIUM-11-C-06`).
- If Nav2 enters recovery loops, no "stuck" indicator shows the
  problem (`MEDIUM-11-D-06`).
- If a waypoint fails with `nav_state.status='aborted'`, the mission
  aborts entirely (no per-waypoint retry, no skip-to-next option per
  the executor code at [index.ts:505](../../../src/agv_ui_backend/src/index.ts)).
- No "what failed" surface: operator gets generic `mission_progress.status='failed'`.

**Linked findings**: `MEDIUM-11-C-06`, `MEDIUM-11-D-06`,
`MEDIUM-07-07` (BT timeout absence).

## Aggregate diagnosis

| Step | Wizard exists? | Terminal-free? | Validated? | Linked findings |
|---|---|---|---|---|
| 1 — Power & discovery | No | **No (must know IP)** | n/a | (Sprint D follow-up) |
| 2 — Auth | No (skipped by default) | Yes (if auth stays disabled) | **No** | `CRITICAL-11-C-01`, `HIGH-11-D-01` |
| 3 — Calibration | **No (all CLI)** | **No** | Manual per script | `MEDIUM-11-D-05` |
| 4 — Mapping | Yes | Yes | **No coverage / quality metric** | (Sprint E) |
| 5 — Annotation | Partial (AprilTag yes, zones TBD) | Yes for AprilTags | Yes for AprilTag IDs | — |
| 6 — Mission editor | Yes | Yes | **No coord validation** | `MEDIUM-11-B-05`, `MEDIUM-11-D-04` |
| 7 — Mission execution | Yes | Yes | **No deadman, no stuck signal** | `MEDIUM-11-C-06`, `MEDIUM-11-D-06` |

**The acceptance criterion "<2 h no terminal" fails primarily at
step 3 (calibration is 100 % CLI) and step 1 (must know the IP).**

If step 3 had wizards and step 1 had mDNS + QR, the flow becomes
plausible. The other gaps are non-blocking but degrade the operator
experience.

## Recommended sequencing

1. **Sprint A.5** closes the credential issue (step 2).
2. **Sprint B** closes WiFi loss → deadman + heartbeat (step 7).
3. **Sprint E (new — calibration UI)** closes step 3 — the biggest
   single blocker. Scope: 4 wizards, M each, plus 4 backend routes.
4. **Sprint D** closes step 1 (mDNS + QR), validation in step 6, and
   stuck-recovery indicator in step 7.

After Sprint E lands, a usability test with a real non-ROS technician
becomes feasible. Until then, no test would close the criterion
because the flow has structural gaps a wizard cannot paper over.
