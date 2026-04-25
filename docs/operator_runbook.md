# Operator Runbook — AGV Greenhouse

This is the field-ready procedure for using the AGV from the dashboard
HMI. It covers the four flows the operator will exercise:

1. **Mapping** — record a new map of the environment
2. **Waypoints + Missions** — define paths and execute them
3. **Rail approach** — fine alignment with an AprilTag at the rail entry
4. **Rail drive** — autonomous movement through a rail segment

The dashboard URL is `http://<jetson-ip>:8090/dashboard`. Today
`<jetson-ip>` is `192.168.55.1` (USB bridge) or whatever the WiFi/LAN
interface assigns.

## Pre-flight

Every session, before any operational step:

1. Open the dashboard. Confirm the **TopBar** shows:
   - **MODE** pill: `teleop` (default after boot)
   - **LOC** pill: `LOCALIZED` (green) or `INITIALIZING` (blue spinner)
     — never `FAILED` (red) at start of session.
   - **SAFETY** pill: green
   - **DRIVE** pill: hz around 50 Hz
2. Open `Recovery → Motor Enable` once. The pill turns green and the
   MOTORS state shows `armed=true`. If e-stop is sticky, press
   `Recovery → Clear E-Stop` first.
3. Confirm the **camera** view in the dashboard shows the floor where
   you expect it. If the AprilTag is in frame, you'll see it in the
   AprilTags panel as a `pending_detection` if it's not yet assigned.

## Flow 1 — Mapping

Use case: record a new map of the greenhouse aisle, save it, reload
later for navigation.

**Procedure**:

1. Drive the robot to the starting pose (typically near a known
   AprilTag for later relocalization).
2. In the dashboard, switch the **MODE** pill to `mapping`. The
   backend tightens cmd_vel limits to 0.4 m/s linear / 0.2 rad/s
   angular (cuVSLAM tracking-friendly).
3. Drive the robot with the joystick at ≤ 0.3 m/s through every
   corridor you want mapped. The dashboard's `MAP` panel shows the
   live grid being built (`/agv/live_map` from `scan_grid_mapper`).
4. When coverage looks complete, click **Save Map**. Type a name
   (use a stable convention: `<area>_v<N>`, e.g. `aisle_a_v1`).
5. The save flow runs:
   - `agv_map_manager/save_map` → writes `aisle_a_v1.{yaml,pgm}` to
     `${AGV_DATA_DIR}/maps/`
   - Calls `/agv/zed/save_area_memory` → cuVSLAM Area Memory
     (`aisle_a_v1.area`)
   - Calls `/visual_slam/save_map` → cuVSLAM keyframe DB
     (`aisle_a_v1_cuvslam/`)
   - Writes `aisle_a_v1_meta.json` with last-known pose
6. Click **Refresh Maps** in the dashboard. The new map should
   appear in the list with the most recent timestamp.
7. To switch back to navigation, change MODE → `nav`. The auto-init
   orchestrator runs the localization cascade automatically.

**What can go wrong**:

- The MODE switch to `nav` is rejected with HTTP 409 if Nav2's
  lifecycle is not active. Check the **NAV** pill — if amber, click
  `Recovery → Nav Restart` (or restart the service from CLI).
- Save Map fails if `cuvslam_in_hil:=true` (HIL has no real ZED).
  Confirm `AGV_MODE=real` in `systemctl show agv.service`.
- If `LOC` goes to `FAILED` after Save, drive to a known AprilTag
  via teleop and click `Recovery → Reinitialize Localization`.

## Flow 2 — Waypoints + Missions

Use case: define a sequence of waypoints (some snapped to AprilTags),
execute as a mission, optionally insert pauses or start/stop
recording at specific points.

**Procedure**:

1. With a map loaded and `LOC = LOCALIZED`, switch MODE → `nav`.
2. In the **AprilTags** panel, ensure each operationally relevant
   AprilTag is **defined** with the correct world coordinates. For
   rail-entry tags, set `type='rail_start'` so the mission executor
   can auto-trigger the precision alignment.
3. In the **Missions** panel, click **+ New Mission**. For each
   waypoint:
   - Click on the map to set the goal location, OR
   - Click an AprilTag in the map to **snap** the waypoint to its
     coordinates (preserves precision regardless of mouse jitter)
   - Optionally add an `action` after arrival: `pause N seconds`,
     `start_recording`, `stop_recording`
4. Save the mission. It is persisted to
   `${AGV_DATA_DIR}/missions.json`.
5. Click **Execute** on the mission. Watch the dashboard:
   - `mission_progress` shows `current_node / total_nodes`
   - The **NAV** pill shows the active goal
   - For waypoints snapped to a `rail_start` tag, after Nav2 reaches
     the standoff position the mission executor automatically calls
     `/agv/rail_approach/execute`. The dashboard shows
     `rail_approach.state` cycling `idle → fine_servoing → settled →
     idle`. The mission resumes after settle (or 30 s timeout).

**Pause / Resume / Cancel**:

- **Pause**: click `Pause` in the Missions panel. The current Nav2
  goal completes, but the next waypoint is held. Good for inspecting
  conditions mid-route.
- **Resume**: continues with the next waypoint.
- **Cancel**: aborts immediately, sends `cmd_vel(0,0)` through the
  safety chain.

**What can go wrong**:

- Mission rejected with `Not in nav mode` → MODE pill is not `nav`.
  Switch first.
- Mission rejected with `Motors not armed` → click `Recovery →
  Motor Enable`.
- `localization FAILED` mid-mission → recovery path: cancel mission,
  drive to AprilTag, reinitialize, re-execute.
- Rail approach times out (30 s) → either the tag was lost during
  fine_servoing or the FINE_SERVOING band wasn't reached. Check
  `/agv/rail_approach/status` topic for the abort reason.

## Flow 3 — Rail approach (precision alignment)

Use case: align the robot precisely with the entry of a rail before
driving onto it. Required for the rail-drive flow because the rails
are 51 mm tubes — alignment within ~5 cm is mandatory.

**Three trigger paths**:

**Path A (Send AGV — Nav2 + rail_approach)**: from the AprilTags panel,
click **`Send AGV`** on a tag with `type='rail_start'`. The backend:
1. Computes a standoff goal (0.5 m before the tag, facing it).
2. Sets `pendingRailApproach`.
3. Sends Nav2 goal.
4. After Nav2 succeeds, calls `/agv/rail_approach/execute` with the
   tag's hardware ID and `skip_coarse_approach=false`.

**Requires** `localization.action == 'LOCALIZED'` (a map is loaded and
an AprilTag-verified anchor is active). If localization is `DEGRADED`,
the rail_approach node will reject the call.

**Path B (Align — fine-servoing only, NEW 2026-04-25)**: click
**`Align`** instead of `Send AGV`. The backend:
1. Verifies the tag was detected in the last 2 seconds (otherwise 409).
2. Verifies motors are armed and e-stop is clear.
3. Calls `/agv/rail_approach/execute` directly with
   `skip_coarse_approach=true`. **Skips Nav2 entirely.**

This path is **independent of map state** — works with localization in
DEGRADED or even no map loaded. It's the right choice when:
- The robot is already physically near the tag (e.g., the operator
  drove there with the joystick).
- Coordinates in the map frame are unreliable.
- You only need precision alignment, not navigation.

Restriction: requires the tag to have a hardware ID assigned via the
AprilTags panel. The button is disabled if the assignment is missing.

**Path C (mission auto-trigger)**: include the tag as a waypoint in a
mission (see Flow 2). After Nav2 reaches the standoff, the mission
executor calls rail_approach with `skip_coarse_approach=false` (Path A
semantics).

**What rail_approach does** (background, no UI interaction):

- Subscribes to AprilTag detections, runs solvePnP with median(15)
  filtering for stability.
- Closed-loop fine servo with PI+FF gains tuned in iter-46
  (`Kp_linear=0.15`, `stiction_ff_vel_mps=0.035` HIL,
  `Ki_linear=0.05` HIL).
- Settles when the tag is within `tolerance_xy=0.003 m` for
  `settle_frames=5` consecutive frames.
- Validation gate (iter-46): mean error ≤ 4.5 cm at the rail
  entrance.

**Operator visibility**: the dashboard's `Rail` panel shows
`rail_approach.state` (`idle / fine_servoing / settled / aborted`).
After `settled`, the operator can proceed to rail drive (manual or
let mode_arbiter auto-trigger).

## Flow 4 — Rail drive

Use case: drive autonomously through a rail segment after successful
alignment.

**Procedure**:

1. After `rail_approach.state == settled`, the **mode_arbiter** FSM
   transitions automatically to `RAIL_DRIVE`. The arbiter relays
   `/agv/cmd_vel_rail` to `/agv/cmd_vel`.
2. **rail_driver_node** publishes a goal at +3.0 m forward (the
   expected rail length minus exit clearance) and runs P-control on
   linear velocity (Kp=1.0, max 0.5 m/s).
3. The robot drives forward through the rail. The dashboard shows
   `rail_driver.state == DRIVING` and the `RAIL` pill cycles.
4. On reaching the configured exit distance (default 3.0 m past
   entry), `rail_driver.state == REACHED`. The mode_arbiter
   transitions back to whatever was queued — typically `nav` for
   the next waypoint, or `idle` for end-of-mission.

**Reverse out of a rail**: rail_driver supports `linear.x < 0` so
you can manually back out via teleop if needed (forward-only is a
Nav2 constraint OUTSIDE rails, not inside).

**What can go wrong**:

- **Stuck**: rail_driver detects no progress for `progress_timeout`
  seconds → publishes `BLOCKED_NO_PROGRESS`. Operator must back out
  with teleop.
- **Side hits**: the rails physically constrain lateral motion, so
  collision_monitor's stop_zone (5 cm in front of footprint) is the
  only software safety. The 51 mm rail guides are not detectable by
  the LiDAR so the operator should NOT navigate into a rail unless
  rail_approach succeeded.

## End-of-day checklist

1. Cancel any running mission.
2. Drive to a parking pose (ideally near a known AprilTag for
   tomorrow's session start).
3. `Recovery → Disarm Motors` to release torque.
4. The systemd service stays running; the Jetson can be left up.

## Troubleshooting

### Backend (`:8090`) doesn't come up after a code change

If you (or an agent) modified anything in `src/agv_interfaces/`
(messages or services), the rclnodejs cache in
`node_modules/rclnodejs/generated/` is stale and the backend will
crash at startup. Symptom: `:8090` never starts listening; the
dashboard hangs at "loading…" indefinitely.

Fix:

```bash
rm -rf ~/ros2_ws/src/agv_ui_backend/node_modules/rclnodejs/generated/*
sudo systemctl restart agv.service
```

The first boot after the clear takes 2-3 min extra (cache regen).

## Quick CLI reference (bypassing the dashboard)

```bash
# State at a glance
curl -s http://localhost:8090/api/status | python3 -m json.tool

# Force-arm motors
ros2 topic pub --once /agv/motor_enable std_msgs/msg/Bool '{data: true}'

# Watch slip + dwell live (for debugging during operation)
python3 /tmp/live_slip_monitor.py     # if you saved my monitor

# Send a one-shot Nav goal
ros2 action send_goal /agv/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}}}"

# Trigger rail_approach manually (must be in nav mode + tag visible)
ros2 service call /agv/rail_approach/execute \
  agv_interfaces/srv/RailApproach \
  "{tag_id: 12, offset_x: 0.3, offset_y: 0.0}"
```
