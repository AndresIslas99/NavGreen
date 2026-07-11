# HAZOP Skeleton — AGV Greenhouse

> **Status**: Skeleton produced by the 2026-05-13 audit as a starting
> point for a proper HAZOP workshop. This file is **not** a signed
> HAZOP. It is a candidate hazard inventory with proposed severities
> and current mitigations, ready for a workshop with the operator,
> engineering lead, and customer safety representative.
>
> **Reference**: per `policies/engineering_rules.md` Rule 6, this
> repository cannot claim certified functional safety. The hazards and
> mitigations described here are operational safeguards. Certification
> (ISO 13849, IEC 61508, ISO 3691-4) requires hardware-integrated
> scope (safety scanners, dual-channel relays, safety PLCs) outside the
> current MVP.

## Methodology

Each hazard is rated on:
- **Severity (S)** — 1 = nuisance, 5 = serious injury or major damage.
- **Probability (P)** — 1 = once in robot lifetime, 5 = at least daily.
- **Risk (R)** = S × P. R ≥ 15 needs mitigation before deployment.
- **Mitigation today** — citation to code or config that addresses it.
- **Gap** — what's missing to fully address.

The values below are **initial estimates** by the auditor based on code
review and the documented operational profile (commercial greenhouse,
1.0–1.5 m corridors, mix of trained operator + untrained workers). The
workshop refines them.

## Hazard inventory

### H-01 — Frontal collision with stationary person
- **Description**: A person stands in the corridor; robot fails to detect
  and stop.
- **S**: 5 (serious injury possible at 0.25 m/s × 30 kg = 1 m/s² impact)
- **P**: 2 (would require sensor + planner failures simultaneously)
- **R**: 10
- **Mitigation today**:
  - Costmap inflation 0.55 m around static obstacles
    (`nav2_params.yaml:311`).
  - collision_monitor stop_zone 20 cm front + scan + pointcloud sources
    (`collision_monitor.yaml:52-67`).
  - vx_max 0.25 m/s → stop distance ~14 cm (`MEDIUM-07-05`).
- **Gap**: 30 cm ZED minimum depth (`agv_navigation/CLAUDE.md`); if
  person ends up within 50 cm of robot front, no sensor sees them.
  `HIGH-09-01` hardware E-stop missing.

### H-02 — Frontal collision with moving person
- **Description**: Person walks across robot's path.
- **S**: 4 (impact at relative speed ~1.25 m/s assuming person walking)
- **P**: 3 (occurs during operator-supervised work)
- **R**: 12
- **Mitigation today**: Same as H-01.
- **Gap**: collision_monitor reactivity at the closing rate. Test
  required (Phase 9 hardware item).

### H-03 — Lateral collision with plant bed / crop row
- **Description**: Robot scrapes side of cuna de plantas while traversing
  corridor.
- **S**: 2 (plant damage; cumulative crop loss)
- **P**: 4 (every mission has lateral excursions if tracking is poor)
- **R**: 8
- **Mitigation today**:
  - Footprint 0.74 m wide (but see CRITICAL-02-02 — actual wheels may
    stick out).
  - PathAlignCritic in MPPI (weight 20).
- **Gap**: CRITICAL-02-02 geometry SSOT means Nav2 footprint may not
  match the physical robot. Once that closes, this risk drops.

### H-04 — Running over a coiled hose
- **Description**: Greenhouse irrigation hose lying across the corridor;
  robot drives over and entangles.
- **S**: 3 (hose damage; risk of wheel jam; possible irrigation
  shutdown)
- **P**: 4 (hoses are routine greenhouse infrastructure)
- **R**: 12
- **Mitigation today**:
  - Voxel layer `min_obstacle_height: 0.10` (`nav2_params.yaml:274`) —
    hoses below 10 cm are **ignored**.
  - pointcloud_to_laserscan `min_height: 0.01` (`agv_full.launch.py:209`)
    — sees small objects in the scan.
  - collision_monitor `max_points: 1` (`collision_monitor.yaml:56`) —
    a single point triggers stop.
- **Gap**: Costmap doesn't plan around hoses (below voxel threshold),
  but collision_monitor reacts to them. Mitigation is reactive, not
  preventive. **Action**: lower `min_obstacle_height` in voxel layer to
  match scan threshold, OR keep current setup with explicit operator
  training "don't leave hoses on the path".

### H-05 — Fall over a drainage step / negative obstacle
- **Description**: Greenhouse drainage trench or step-down; robot can't
  detect a drop and tips.
- **S**: 4 (robot damage, possible escape from drainage, possible
  injury to repair worker)
- **P**: 2 (drainage trenches are typically marked / mapped)
- **R**: 8
- **Mitigation today**: **NONE** — `MEDIUM-06-01` (no negative-obstacle
  detection).
- **Gap**: This is the most under-engineered hazard. Mitigation
  options: (a) map drainage as keepout zones; (b) add a downward-facing
  ToF sensor at the front; (c) lower the camera and use depth gradient
  detection.

### H-06 — Localization failure mid-mission
- **Description**: AprilTag obscured by foliage; pose drifts; robot
  navigates to phantom coordinates.
- **S**: 3 (potential collision per H-01..H-04)
- **P**: 4 (foliage grows; tag occlusion is normal)
- **R**: 12
- **Mitigation today**:
  - `auto_init_orchestrator` cascade (Path A0 / A / B / C).
  - LOC pill in dashboard.
  - `sendNavGoal` gated on `localization.action != FAILED` for
    single-goal navigation.
- **Gap**: `agv_waypoint_manager` bypasses the gate (`CR-00-06`).
  Mission execution unprotected. Kidnapping not detected (`HIGH-04-03`).

### H-07 — Loss of WiFi to operator
- **Description**: Operator on tablet loses dashboard connection; robot
  continues mission without supervision.
- **S**: 3 (no immediate harm, but supervision is lost)
- **P**: 4 (greenhouse WiFi is fragile per project notes)
- **R**: 12
- **Mitigation today**:
  - `agv_odrive_node.cmd_vel_timeout_ms: 200` — motors stop if cmd_vel
    silent.
  - But Nav2 mission continues autonomously; cmd_vel keeps flowing.
- **Gap**: No "deadman" requirement that the operator remain connected
  for a mission to continue. **Recommendation**: add a periodic
  heartbeat from dashboard to robot; if missed for > 10 s, mission
  pauses (not aborts — pauses, awaiting reconnection or local intervention).

### H-08 — Motor failure on a slope
- **Description**: Greenhouse floors with slight slopes; a motor fails
  while moving up.
- **S**: 3 (roll-back into operator zone)
- **P**: 1 (rare event)
- **R**: 3
- **Mitigation today**: ODrive S1 monitors temps; ODrive has built-in
  fault detection.
- **Gap**: No software check that asks "is the robot moving the
  expected direction given commanded cmd_vel?". If the robot rolls
  back while commanded forward, no current code detects it.
- **Recommendation**: Add a "command vs reality" check in
  `fusion_monitor` — if `cmd_vel.linear.x > 0` and `odometry/local.linear.x
  < -threshold` for N seconds, raise an event.

### H-09 — Jetson Orin overheating at midday sun
- **Description**: Sun through translucent roof heats the Jetson;
  thermal throttle drops performance.
- **S**: 2 (mission slowdown, possible thermal shutdown)
- **P**: 3 (midday in Mexico is hot, even indoors)
- **R**: 6
- **Mitigation today**: Jetson L4T thermal management built-in.
- **Gap**: No software-level monitoring of CPU thermal throttling. If
  throttle kicks in, MPPI may miss its 20 Hz cycle, collision_monitor
  may degrade. **Recommendation**: subscribe to `/diagnostics` thermal
  sensors and surface to dashboard. Pause mission above threshold.

### H-10 — Battery critically low in remote zone
- **Description**: Battery drops below safe threshold while robot is
  far from charger.
- **S**: 3 (robot stranded; possible damage if it sits in puddle)
- **P**: 3 (no automatic charging in MVP)
- **R**: 9
- **Mitigation today**: **NONE** — `interfaces.yaml#/agv/battery` is
  marked `status: planned`. No battery monitoring is wired.
- **Gap**: Critical missing telemetry. **Recommendation**: read ODrive
  VBUS voltage via CAN, publish `/agv/battery`, set thresholds
  (warn at 30 %, abort-and-stop at 15 %).

### H-11 — Obstacle detection callback stalls
- **Description**: pointcloud_to_laserscan crashes or hangs; no obstacle
  topic; collision_monitor degrades.
- **S**: 4 (potential collision)
- **P**: 1 (well-tested package)
- **R**: 4
- **Mitigation today**:
  - `safety_supervisor` monitors `/agv/scan` with 250 ms deadline. If
    silent, `safety_ok = false` → `cmd_vel_gate` zeros output.
- **No gap.** This is exactly what the supervisor is for.

### H-12 — Spontaneous reboot during operation
- **Description**: Power glitch / kernel panic / OOM kill; Jetson reboots.
- **S**: 3 (mission lost; potential motion glitch during reboot)
- **P**: 2 (rare but possible in field)
- **R**: 6
- **Mitigation today**:
  - `agv_odrive` motors disabled on cmd_vel timeout.
  - `agv_start.sh` restores last map at boot.
  - `auto_init_orchestrator` Path A0/A/B/C cascade restores pose.
- **Gap**: Boot takes 30–60 s. During that window the robot is
  unsupervised. If it was mid-corridor, it stays there.
- Acceptable risk class — physical safety is preserved (no power = no
  motion).

### H-13 — AprilTag displaced or occluded by foliage growth
- **Description**: A registry tag is covered by a growing plant; pose
  estimation degrades.
- **S**: 3 (drift accumulates; downstream collision risk)
- **P**: 4 (continuous in a growing greenhouse)
- **R**: 12
- **Mitigation today**: Multi-tag voting (`marker_correction_node.cpp:417-470`);
  range-quadratic covariance; rejection threshold 3.0 Mahalanobis.
- **Gap**: No "tag last seen" telemetry. The dashboard cannot tell
  operator "tag 14 has not been seen in 24 h, may be occluded".
  **Recommendation**: AprilTagManager in backend tracks last-detection
  timestamp per tag; surface stale tags.

### H-14 — Untrained operator activates the wrong mode
- **Description**: Person not familiar with the dashboard hits "execute
  mission" without checking robot state.
- **S**: 3 (potential collision with unintended path)
- **P**: 3 (greenhouse staff turnover)
- **R**: 9
- **Mitigation today**: `agv_ui_backend` action gates (motors must be
  armed, collision_monitor fresh, localization not FAILED).
- **Gap**: No operator role / permission layer; any authenticated user
  can do anything. **Recommendation**: roles `viewer`, `operator`,
  `admin` in `auth.ts`. Mission execute requires `operator`+.

### H-15 — Robot stuck between plant and wall without recovery
- **Description**: Robot drives into a corner it can't exit; BT
  exhausts recoveries and aborts.
- **S**: 2 (no immediate injury; mission failure)
- **P**: 3 (greenhouse corners are common)
- **R**: 6
- **Mitigation today**: Custom BT with `Spin(180°)` as last recovery
  (`navigate_to_pose_forward_only.xml:80`); abort cleanly on failure.
- **Gap**: No clear signal to operator (`MEDIUM-07-07`) — operator may
  not realize the robot is stuck. Telemetry of "in recovery for X
  seconds" not surfaced.

### H-16 — Tag spoofing — malicious actor places fake tag
- **Description**: Worker prints a fake AprilTag ID matching a real one
  and places it elsewhere.
- **S**: 3 (pose jump; potential collision)
- **P**: 1 (intentional adversary not in threat model)
- **R**: 3
- **Mitigation today**: Tags not in registry are ignored (correct);
  known IDs at new locations DO update pose (vulnerable). Single tag
  outside the median by > 1 m is voted out by multi-tag (`marker_correction_node.cpp:430-440`).
- **Gap**: Single-tag observations are not voted; if the spoof is the
  only tag visible, it wins. **Recommendation**: refuse to relocalize
  from a single observation if drift > threshold; require either
  multi-tag or a confirming cuVSLAM agreement.

### H-17 — Wet floor depth aliasing
- **Description**: Puddle reflects ceiling; ZED depth returns spurious
  values below floor; voxel layer + collision_monitor get false positives
  or false negatives.
- **S**: 2 (false stops are nuisance; missed detection is hazardous)
- **P**: 4 (greenhouses are routinely wet)
- **R**: 8
- **Mitigation today**: Per `MEDIUM-06-04` notes — no explicit reflection
  filter. The voxel layer's `min_obstacle_height: 0.10` accidentally
  helps (filters returns at floor level).
- **Gap**: A real reflection filter that validates depth against the
  ground plane. **Recommendation**: ground-plane fit (RANSAC) + reject
  points that would be below the ground plane.

### H-18 — Camera mount drift after a bump
- **Description**: Worker brushes camera housing; extrinsic transform
  silently incorrect.
- **S**: 3 (continuous pose error)
- **P**: 3 (greenhouse work is hands-on around equipment)
- **R**: 9
- **Mitigation today**: **NONE** — `MEDIUM-01-03` (no online consistency
  check).
- **Gap**: HIGH-01-02 (extrinsic outside this repo) and MEDIUM-01-03
  (no monitor) together. **Recommendation**: implement the online check
  proposed in MEDIUM-01-03.

## Summary

| Risk band | Count |
|---|---|
| R ≥ 12 (mitigate before deployment) | 6: H-02, H-04, H-06, H-07, H-13 (R=12); H-01, H-03 (R=10/8) |
| 6 ≤ R < 12 | 8: H-05, H-08, H-09, H-10, H-15, H-17, H-18 |
| R < 6 | 4: H-11, H-12, H-14, H-16 |

**Items with `Gap: NONE`**: H-11 (sensor stall — handled by safety
supervisor). The remaining 17 have gaps, of which `HIGH-04-03`
(kidnapping), `HIGH-09-01` (hardware E-stop), `MEDIUM-06-01` (negative
obstacle), and `H-10` (battery monitoring) are repeated entries that
will close several hazards at once.

## Next steps

1. **Workshop** to refine S and P estimates with site-specific knowledge.
2. **Closure plan** for items R ≥ 12: assign owner + deadline.
3. **Re-rate** after Sprint A + B remediation lands.
4. **Sign-off** by customer safety representative for the field visit.
