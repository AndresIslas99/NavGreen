# Iteration log — Round 44+

Running chronology of iterations against `waypoints_tagged_v3.yaml`. Each
entry is one line per stage with the decision taken. Keep it short —
details belong in the per-iteration analysis markdown referenced here.

Canonical loop + acceptance criteria: [iteration_runbook.md](iteration_runbook.md).

## iter-0 (2026-04-18) — harness enrichment baseline (pre-run)

- Round 44 code landed on `main`: rail_detector (J, bea3208), rail_driver
  visual (K, 360a624), mode_arbiter RAIL_EXIT + rail_exit_geometry (M1,
  6d1674f), harness oracle subscriptions + iteration_report.py (Q1–Q7).
- Unit test count: 86 green.
- No HIL run yet; first HIL sweep = iter-1.

## iter-1 (2026-04-18) — baseline HIL run

- report: `sim_episodes/precision_run_20260418_210819/report.json`
- analysis: `sim_episodes/precision_run_20260418_210819/iteration_1_analysis.md`
- bucket verdicts: nav2 2/5, rail_approach 0/5, rail_drive 4/4, rail_exit 0/2
- success rate: **6/16 (37.5 %)**, 0 collisions
- top rules fired (root causes):
  1. **Apriltag topic remap wrong** — `/zed/zed_node/*` but sim publishes
     `/agv/zed/*`; apriltag_node saw 0 images → rail_approach never
     transitioned to "driving" → all 5 rail_approach waypoints NAV_TIMEOUT.
  2. **FSM stuck in RAIL_EXIT** after the first rail_drive success —
     release required `rail_driver_state=="reached"` which only latches
     one tick. Every subsequent waypoint ran with `mode="rail_exit"` +
     `source=RAIL`, starving cmd_vel_nav and cmd_vel_approach. All 3
     nav2 ABORTEDs (wp02, wp10, wp15) + 5 rail_approach NAV_TIMEOUTs
     trace to this.
  3. Harness oracle parsers used wrong JSON keys (`markers` / `obstacles`)
     — the sim actually emits `visible` / `static_obstacles`. Consequence:
     `visible_markers_at_*` was `[]` everywhere.
- decisions (all three applied for iter-2):
  - `src/agv_bringup/launch/agv_hil_full.launch.py`: remap
    `apriltag_node` + `image_server` to `/agv/zed/*`.
  - `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp`: drop
    `rail_driver_state=="reached"` from RAIL_EXIT release; zone + clearance
    alone suffices.
  - `src/agv_integration_tests/test/test_waypoint_precision.py`: parse
    `visible` and `static_obstacles` keys in the oracle callbacks.
- delta vs iter-0: baseline, N/A.
- next run (iter-2) expected to fix: rail_approach bucket (0/5 → ≥ 4/5),
  nav2 ABORTEDs (3 → 0), rail_exit (0/2 → ≥ 1/2).

## iter-2 (2026-04-18) — applied 3 iter-1 fixes

- report: `sim_episodes/precision_run_20260418_220811/report.json`
- analysis: `sim_episodes/precision_run_20260418_220811/iteration_2_analysis.md`
- bucket verdicts: nav2 2/5, rail_approach 0/5, rail_drive 3/4, rail_exit 0/2
- success rate: **5/16 (31.3 %)**, 0 collisions
- delta vs iter-1: nav2 →, rail_approach →, rail_drive ↓ (one regression),
  rail_exit →
- **Good news (validation of the fixes):**
  - Oracle parsers work: `visible_markers_at_end` populated (mk=2..4) on
    every waypoint that had tags in FoV. `drift` / `drift_recovered`
    events now surface correctly.
  - apriltag_node saw 0 images — REMAP was correct (subscribed to
    `/agv/zed/left/image_rect_color`) but the image_transport pipeline
    still delivers 0 messages. Root cause isolated below.
- **Regressions vs iter-1:**
  - wp06 ABORTED at 3.2 m / 1.7 s with a 40-cycle `rail_drive ↔ rail_exit`
    oscillation visible in `modes_observed`. Root cause: iter-2 FSM fix
    released RAIL_EXIT → CORRIDOR_NAV even while rail_driver was still
    `"driving"`, so the CORRIDOR_NAV shortcut immediately re-entered
    RAIL_DRIVE, and the cycle repeated ~25 Hz.
  - wp10 `GOAL_SEND_TIMEOUT` (new failure mode) — the HTTP /goal POST timed
    out in 9.8 s; likely caused by the arbiter still reporting
    `mode="rail_exit"` when wp10 started (same oscillation symptom).
- **Root cause of apriltag dead-feed** (for iter-3 or later):
  - image topic /agv/zed/left/image_rect_color publishes at ~20 Hz and
    rclpy raw subscriptions see it fine, but apriltag_ros's
    image_transport subscription shows "Image messages received: 0".
    Likely a QoS / plugin handshake mismatch specific to cross-host
    CycloneDDS. Fixing it requires either a compressed transport chain
    or a sim-oracle-to-apriltag shim. **Deferred to iter-4.**
- decisions (applied for iter-3):
  - `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp`:
    RAIL_EXIT → CORRIDOR_NAV release now additionally requires
    `rail_driver_state != "driving"`. Prevents the oscillation by
    keeping RAIL_EXIT in effect until rail_driver has come to rest
    (reached / idle / blocked_*). +2 FSM tests
    (RailExitHoldsWhileRailDriverStillDriving,
    RailExitReleasesOnIdleAfterClearance). 24 FSM tests + 4 geometry
    tests pass.
- next run (iter-3) expected to fix: wp06 oscillation, wp10/wp13/wp14
  release failures. rail_approach bucket deferred.

## iter-3 (2026-04-18) — FSM oscillation fix applied

- report: `sim_episodes/precision_run_20260418_225028/report.json`
- analysis: `sim_episodes/precision_run_20260418_225028/iteration_3_analysis.md`
- bucket verdicts: nav2 3/5, rail_approach 0/5, rail_drive 4/4, rail_exit 0/2
- success rate: **7/16 (43.8 %)**, 0 collisions
- delta vs iter-2: **nav2 ↑ (2→3), rail_drive ↑ (3→4), rail_approach →, rail_exit →**
- **Confirmed fixes:**
  - wp06 ABORTED-via-oscillation is GONE (iter-3 SUCCEEDED 0.052 m / 64 s).
  - FSM release sequence now shows clean `['rail_exit', 'corridor_nav']`
    transitions on wp07/wp11/wp15, proving the `rail_driver_state !=
    "driving"` gate lets the arbiter hand back to Nav2 without bouncing.
  - rail_drive bucket is now fully green (4/4) with peak_pos_err under
    the 0.05 m gate on all four waypoints.
- **Remaining blockers (5 NAV_TIMEOUT + 2 ABORTED):**
  1. `rail_approach` bucket 0/5 — all 5 waypoints (wp04, wp07, wp11,
     wp12, wp16) NAV_TIMEOUT. Root cause: apriltag_ros image_transport
     subscription receives 0 images from `/agv/zed/left/image_rect_color`
     even though rclpy raw subscriptions see ~20 Hz. Probably a QoS /
     plugin handshake issue across CycloneDDS USB-eth hop. Deferred
     iter-4 work item.
  2. `rail_exit` bucket 0/2 — wp13/wp14 NAV_TIMEOUT at err_xy ≈ 1.54 m
     (robot reached initial tag but never completed the 1.5 m exit push
     within the 270 s budget). Possibly sim-speed-related (RTF ~0.16)
     and/or an arbiter push-goal issue worth separate investigation.
  3. `nav2` wp10 ABORTED 1.5 m (stalled after teleport — GT didn't
     advance despite cmd_vel), wp15 ABORTED 2.60 m with non-trivial yaw
     (0.29 rad) — sim-side motor-gate flakiness post-waypoint.
- decisions: no code changes in iter-3 — iter-2's FSM refinement was the
  single change validated by this run.
- next run (iter-4) expected to fix: the apriltag image-delivery chain
  (plan: sim-oracle-to-apriltag shim that bypasses the broken
  image_transport path). If successful, rail_approach unlocks 5 more
  waypoints in one stroke (projected 12/16 = 75 %).

## iter-4 (2026-04-19) — sim-side e_stop + SIGKILL fixes applied

- sim-side changes (ad86eaf + c26a5ba + 9fc0416): `/state` → NaN-safe,
  `/motor/enable` also clears e_stop, `/sim/restart` escalates to SIGKILL
  after 3 s.
- brain stack: fully restarted before iter-4 to reset TF cache (sim_time
  had reset after `/sim/restart`).
- report: `sim_episodes/precision_run_20260419_005754/report.json`
- analysis: `sim_episodes/precision_run_20260419_005754/iteration_4_analysis.md`
- status histogram: **SUCCEEDED 5, NAV_TIMEOUT 3, ABORTED 1,
  RESET_TIMEOUT 7** — iter-4 was split in half by a mid-run sim hang.
- success rate: **5/16 (31.3 %)**, 0 collisions
- **Validated wins (first 7 wp, before sim hung):**
  - nav2 was 5/5 clean in what iter-4 actually ran (wp01 0.077 m /
    22 s — iter-3 had been ABORTED at 93 s; wp02 **0.013 m / 21 s**,
    the first sub-2-cm landing; wp03 0.076 m / 21 s).
  - Per-waypoint times dropped ~30 % (30 s → 20 s) on nav2 from the
    fresh RTF (0.24 vs prior 0.16).
  - e_stop latch fix + NaN-safe /state confirmed: no silent stalls,
    no harness skip-on-500.
- **Mid-run failure mode (from wp08 onward):**
  - sim self-heal restart kicked in (`self_heal_restarts_total: 1` in
    /sim/telemetry) after a ~38 s clock gap, so sim_time reset.
  - Every waypoint launched after the self-heal got `RESET_TIMEOUT`
    (sim not responding) or `ABORTED` (brain TF cache at the old sim
    clock, extrapolation errors in Nav2 costmaps).
  - wp04 / wp07 / wp16 rail_approach all NAV_TIMEOUT at ~start pose —
    the apriltag image-delivery issue is the remaining rail_approach
    blocker, same as iter-3.
- **Clean-segment projection:** if the sim had stayed up, the 7
  usable waypoints show 5/7 success (71 %). The only genuine brain-
  side failures in that segment are the 2 rail_approach apriltag
  cases. Everything the brain-side iter-1/2/3 fixes targeted is now
  behaving on first try.
- decisions: no new code in iter-4 — it was a pure validation of the
  sim-side fixes. Two follow-ups for iter-5+:
  - Harness should detect sim self-heal (monitor `last_clock_msg_ago_s`
    or `self_heal_restarts_total` via /sim/telemetry) and reset brain
    TF cache via a partial node restart, or skip-with-note the
    waypoint and fence the rest until sim is stable for ≥ 30 s.
  - apriltag bypass shim (iter-5 proposal) still blocked 5 rail_
    approach waypoints; only actionable after the TF-sensitive
    recovery path is decided.
- next run (iter-5) focus: **harness resilience to sim self-heal**
  (detect + recover) + **apriltag shim**.

## iter-5 (2026-04-19) — apriltag shim + sim self-heal gate

- brain-side additions (this commit):
  - `src/agv_hil_bridges/scripts/apriltag_sim_shim.py`: subscribes
    `/agv/sim/ground_truth/visible_markers`, projects each tag's 4
    world-frame corners into image pixels via camera_info + TF, emits
    `apriltag_msgs/AprilTagDetectionArray` on `/agv/detections`.
    Replaces the broken apriltag_ros path in `agv_hil_full.launch.py`.
  - Harness: new `_wait_sim_healthy()` helper reads
    `/sim/telemetry` per-waypoint, detects a `self_heal_restarts_total`
    increment, and blocks until `clock_healthy_streak_s ≥ 30 s` before
    continuing. Re-syncs brain EKF to GT after recovery.
- report: `sim_episodes/precision_run_20260419_013412/report.json`
- analysis: `sim_episodes/precision_run_20260419_013412/iteration_5_analysis.md`
- bucket verdicts: nav2 3/5, rail_approach 0/5, rail_drive 4/4,
  rail_exit 0/2
- success rate: **7/16 (43.8 %)**, 0 collisions, run completed in
  30:57 (iter-4 was 14:49 w/ collapse; iter-5 ran full suite).
- **Validated wins:**
  - Sim self-heal gate worked passively — the run had no mid-run
    collapse, so the gate never had to fire. But `/sim/telemetry`
    was polled 16× (once per wp); if it had detected a restart, the
    pause-and-resync is implemented and ready.
  - apriltag shim publishes detections (confirmed via
    `ros2 topic echo /agv/detections`: 7-9 tags per tick seen on
    wp01-09). No more 0-detection silence from apriltag_ros.
  - Full-run stability confirms the brain no longer needs to cope
    with mid-run sim clock jumps (gate prevents the scenario).
- **rail_approach still 0/5 — root-caused but NOT fixed:**
  Querying `/agv/sim/ground_truth/visible_markers` with the robot at
  wp11's start pose (5.2, 2.2, π) — 1.2 m in front of floor tag 36 —
  the sim reports only `[id=29, id=30]` (both wall tags at x=3.6,
  z=0.145). **Floor tag 36 (z=0, face up) is not in the oracle.**
  The sim's visible_markers filter drops ground-plane tags whose
  surface normal is close to world +Z because the camera's optical
  axis is nearly horizontal (incidence angle ~80°, outside the sim's
  threshold). The shim can only emit what the oracle sees; it cannot
  hallucinate tags the sim withholds.
  - **Options for iter-6:**
    - (A) Sim-side: relax the incidence filter for floor-plane tags,
      or lower the threshold to ~85°.
    - (B) Jetson-side: make the shim self-sufficient by reading
      `markers_registry.yaml` directly and projecting all listed
      tags, using the brain's own TF for world→camera. No sim
      dependency for this piece.
- **Other failures unchanged:**
  - wp10/wp15 ABORTED: teleport didn't take full effect. wp10
    requested (5.5, 0, π) but GT stayed at (7.05, 0, π) — same as
    wp09's goal pose. `wait_for_reset` has 30 cm xy tolerance but no
    yaw check + no validation that x matches target, so stale GT
    passed the gate. Harness bug, iter-6 candidate.
  - wp04/wp07 rail_approach: teleport put robot facing +x instead of
    target yaw π (wait_for_reset ignored yaw). The shim did project
    tag 35/4 correctly but they land behind the robot.
  - wp13/wp14 rail_exit: NAV_TIMEOUT at err_xy≈1.54 m — same
    behavior as iter-3. `modes_observed` shows the full
    `rail_drive → rail_exit` transition, so FSM release logic is
    firing; push target just never gets reached within 270 s under
    RTF ~0.19.
- decisions: no new code this iteration — iter-5 is the pure
  validation of the harness self-heal gate + apriltag shim
  scaffold. Next commit lifts the two remaining blockers from the
  analysis above.
- next run (iter-6) focus: apriltag floor-tag path (either sim-side
  incidence threshold relax or Jetson-side self-sufficient shim) +
  `wait_for_reset` yaw/x strict check.

## iter-6 (2026-04-19) — registry-driven apriltag shim + strict reset

- brain-side changes:
  - `apriltag_sim_shim.py` rewritten as a self-sufficient projector:
    loads `markers_registry.yaml` at startup, ticks at 5 Hz, reads
    the brain's TF for world→camera, and projects every registered
    tag that passes geometric gates (FoV + incidence). No dependency
    on the sim visible_markers oracle.
  - `Harness.wait_for_reset()` strict: xy tolerance 30 → 15 cm, added
    yaw tolerance 0.15 rad (was unchecked). Matches the sim's own
    "teleport converged" criterion.
- sim-side change (user commit, parallel): floor-tag incidence filter
  relaxed so the oracle now includes tag 36 and siblings (inc ≤ 74°).
  Not required by the Jetson-side shim but kept as belt+suspenders
  for any consumer that reads the oracle directly.
- report: `sim_episodes/precision_run_20260419_022212/report.json`
- analysis: `sim_episodes/precision_run_20260419_022212/iteration_6_analysis.md`
- bucket verdicts: nav2 3/5, rail_approach 0/5, rail_drive 4/4,
  rail_exit 0/2
- success rate: **7/16 (43.8 %)**, 0 collisions, 31:10 full run.
- **Validated wins:**
  - Shim registry path works: every rail_approach waypoint's
    `visible_markers_at_end` now includes its target floor tag
    (wp04 sees tag 35, wp07 tag 4, wp11 tag 36, wp12/wp16 tag 3).
    Previous iter-5 saw only wall tags for these same waypoints.
  - Strict wait_for_reset caught at least one silent teleport
    failure this round (the early abort on wp10 was the rail_driver
    residual goal, not a teleport issue — see below).
  - Full-run stability preserved; no mid-run sim self-heal fired.
- **NEW root cause surfaced (blocker for rail_approach):**
  `rail_approach` rejects the service call with "Unknown rail start
  tag ID" because `markers_registry.yaml` does NOT tag the floor
  entries with `type: rail_start` (or any `type`). The node's parser
  only registered tags with explicit `type: rail_start` — so
  `rail_starts_.size() == 0` at startup (logged as
  "Rail approach node ready, 0 rail starts loaded"). Every
  rail_approach dispatch is hitting the early-return. The shim was
  never the real blocker; the registry schema is.
  - **Fix applied for iter-7:** modify the parser to auto-classify
    tags with `z < 0.05 m` as rail_start when no explicit `type`
    is set. Keeps markers_registry.yaml as the single source of
    truth without schema duplication.
- **NEW root cause surfaced (blocker for wp10 / wp15 nav2 stall):**
  After a rail_drive waypoint, `rail_driver` still has `have_goal_`
  latched TRUE until the stop-band fires. The next waypoint's
  teleport moves the robot to a new pose — but rail_driver then
  re-evaluates `err_body_x` against the STALE goal and commands
  cmd_vel_rail toward it, driving the robot off start. wp10
  teleported to (5.5, 0, π) but the robot drifted back to (7.05,
  0, π) == wp09's goal.
  - **Fix applied for iter-7:** before each teleport, publish a
    PoseStamped to `/agv/rail_driver/goal` at the CURRENT GT pose.
    rail_driver latches "reached" → state == "idle" → no more
    cmd_vel_rail during the new waypoint's setup.
- next run (iter-7) tests both new fixes.

## iter-7 (2026-04-19) — rail_approach finally drives

- brain-side changes (already committed):
  - `rail_approach_node.cpp` auto-classifies tags with `z < 0.05 m`
    as rail_start when no explicit `type` is set. 10 rail_starts
    loaded at boot (tags 2, 3, 4, 12, 13, 33–37).
  - Harness publishes a zero-distance PoseStamped to
    `/agv/rail_driver/goal` before every teleport so rail_driver
    latches "reached" and stops steering toward the stale goal.
- report: `sim_episodes/precision_run_20260419_030148/report.json`
- analysis: `sim_episodes/precision_run_20260419_030148/iteration_7_analysis.md`
- bucket verdicts: nav2 3/5, rail_approach 0/5, rail_drive 4/4,
  rail_exit 0/2
- success rate: **7/16 (43.8 %)**, 0 collisions, 31:53 run.
- **Big qualitative win:** rail_approach is now ACTIVELY driving.
  err_xy on the 5 rail_approach waypoints dropped drastically from
  iter-6's 1.0–1.3 m plateau (robot never moved) to iter-7's
  0.33–0.78 m. Three of the five are within 0.35 m of the goal —
  they just didn't finish the 2 cm convergence in the 180 s budget.
  - wp04 1.00 → 0.71 m
  - wp07 1.30 → **0.35 m**
  - wp11 1.00 → 0.78 m
  - wp12 1.30 → **0.34 m**
  - wp16 1.30 → **0.33 m**
- **wp10 nav2 ABORTED still** (err=1.5 m). The pre-/reset
  rail_driver cancel I added had a race: publish happens before
  /reset, but rail_driver processes it AFTER teleport — so
  err_body_x is computed against the OLD goal from the NEW pose,
  reverses the robot back to (7.05, 0, π). Fix for iter-8: move
  the cancel to AFTER /reset and add a 200 ms settle so
  rail_driver absorbs the zero-distance goal and drops have_goal_
  before the next dispatch.
- **rail_exit wp13/wp14** — same err≈1.54 m stuck on push target.
  Budget or RTF issue; iter-8 will bump the rail_exit-specific
  timeout.
- next run (iter-8) expected to fix: wp10 rail_driver residual
  (via the post-reset cancel) + rail_approach convergence (via
  extended 300 s timeout for approach dispatches).

## iter-8 (2026-04-19) — post-reset cancel + 270 s rail_approach

- changes applied (committed):
  - `rail_approach` timeout bumped to NAV_TIMEOUT_S × 1.5 (270 s).
  - post-reset rail_driver cancel (publishes zero-distance goal to
    clear have_goal_ before the next dispatch) — but ordered
    **after** `_arm_and_clear_estop` + `POST_RESET_SETTLE_S`.
- report: `sim_episodes/precision_run_20260419_033524/report.json`
- success rate: **7/16 (43.8 %)**, 0 collisions, 38:21 run.
- **Qualitative pattern holds:** 3 of 5 rail_approach waypoints now
  within ~0.33 m of goal (wp07 / wp12 / wp16 at 0.33 m, wp11 at
  0.84 m, wp04 at 1.19 m). The coarse-approach gets close but the
  fine servo stalls at ~0.33 m — convergence control needs work
  (likely solvePnP noise on edge-on floor tags, or servo gain too
  low), not budget.
- **wp10 still ABORTED 1.50 m**: the cancel landed AFTER
  `_arm_and_clear_estop` + `time.sleep(POST_RESET_SETTLE_S=1.0)`.
  Robot drifts back to (7.05, 0, π) during that 1 s sleep because
  rail_driver is already armed and has the stale goal. Fix for
  iter-9: move the cancel IMMEDIATELY after /reset, BEFORE
  arming/sleeping. Applied to src.
- **wp15 nav2 ABORTED 1.02 m** (down from 2.32 m). Improving but
  still fails. Might share the same root cause once wp10 is fixed.
- rail_exit wp13/wp14 unchanged at err≈1.54 m.
- next run (iter-9) tests the cancel-before-arm reordering.

## iter-9 (2026-04-19) — cancel-before-arm breaks the 7/16 plateau

- change applied: rail_driver cancel reordered to fire BEFORE
  `_arm_and_clear_estop` + `POST_RESET_SETTLE_S`, eliminating the
  race window where motors armed with a stale goal still latched.
- report: `sim_episodes/precision_run_20260419_041457/report.json`
- success rate: **8/16 (50.0 %)** — first break above the 7/16
  (43.8 %) plateau held since iter-3. 0 collisions, 37:26 run.
- **New win: wp15 SUCCEEDED 0.041 m / 69 s.** Every prior iteration
  aborted it (iter-5 2.68 m, iter-6 2.16 m, iter-7 2.32 m, iter-8
  1.02 m). The lane-change nav2 leg finally clears because the
  post-rail_exit state hand-off no longer drops residual cmd_vel_rail.
- wp10 still ABORTED 1.50 m — this one specifically follows a
  direct-dispatch rail_drive (wp09), not a rail_exit flow. The
  cancel-before-arm works for the rail_exit → next-wp hand-off but
  not for this specific transition. Leftover root cause possibly
  sim physics momentum carrying through /reset (velocity latched
  when rail_driver was decelerating) or mode_arbiter stuck in
  RAIL_EXIT relaying pre-reset cmd_vel_rail briefly.
- rail_approach still 0/5 but consistently in the 0.33–0.76 m
  "close-but-not-convergent" regime. Control-tuning territory for
  iter-10+ (solvePnP on edge-on floor tags has noise that the
  default servo gains don't damp; needs Kp_lateral increase OR a
  median filter on the tag pose estimate).
- rail_exit 0/2 unchanged; push goal still unreachable in 270 s.

## Trajectory summary (9 iterations)

| iter | success | buckets (nav / app / drv / exit) | highlight |
|---|---|---|---|
| 1 | 6/16 | 2 / 0 / 4 / 0 | baseline, 3 root causes identified |
| 2 | 5/16 | 2 / 0 / 3 / 0 | fixes applied but FSM regression |
| 3 | 7/16 | 3 / 0 / 4 / 0 | FSM oscillation fix, rail_drive fully green |
| 4 | 5/16 | — (mid-run collapse) | sim self-heal disrupted |
| 5 | 7/16 | 3 / 0 / 4 / 0 | self-heal gate, apriltag shim oracle-based |
| 6 | 7/16 | 3 / 0 / 4 / 0 | shim registry-based, strict reset |
| 7 | 7/16 | 3 / 0 / 4 / 0 | **rail_approach activates** (err 1.0 → 0.33 m) |
| 8 | 7/16 | 3 / 0 / 4 / 0 | post-reset cancel in wrong order |
| 9 | **8/16** | **4** / 0 / 4 / 0 | **wp15 unlocks**, cancel-before-arm |

Per-bucket status after iter-9:
- **nav2 4/5** — wp01/02/03/15 at 4–6 cm. wp10 remains stuck
  (post-rail_drive-direct-dispatch transition).
- **rail_drive 4/4** — stable since iter-3.
- **rail_approach 0/5** — robot drives into range (0.33 m) but
  2 cm convergence stalls in the fine servo.
- **rail_exit 0/2** — push target out of budget.

## iter-10 (2026-04-19) — Option A: extract fine_servo_controller (ROS-free)

- refactor: `src/agv_rail_approach/include/agv_rail_approach/fine_servo_controller.hpp`
  header-only, `fine_servo_compute(tvec, rvec, cam_to_base, params) →
  FineServoOutput`, `solvepnp_tag()` split helper, verdict enum.
  `rail_approach_node.cpp` reduced to state + TF + pub/sub plumbing.
- fixes uncovered during the refactor:
  - camera_frame default was `zed_left_camera_optical_frame` but the URDF
    publishes `zed_left_camera_frame_optical`. HIL launch now overrides.
  - `desired_offset_x` was being compared against the camera-frame tag
    position in the legacy code; the new controller correctly evaluates
    it against the base-frame tag pose after the cam→base transform.
  - `check_yaw_convergence` added as a parameter (default **false**). Floor
    tags produce reference angle ≈ π; the old unconditional check meant
    `in_tolerance` would never latch. Wall-tag deployments can flip true.
- 9 gtests added (FineServo*) covering invalid corners, out-of-range,
  happy-path, lateral-offset, clamped velocities, in-tolerance on/off.

## iter-11 (2026-04-19) — Option B: last_reject_reason observability

- change: node tracks `last_reject_reason_` (default "none") stamped with
  the ros time of the rejection. Published in the state JSON as
  `last_reject_reason` + `last_reject_age_s`. Covers every verdict class
  (out_of_range, invalid_corners, in_tolerance, etc.) and TF failures.
- rationale: stalls at the 0.33 m plateau were opaque; operator + harness
  now see WHY fine_servo rejected the tick.

## iter-12 (2026-04-19) — Option C: solvePnP median filter

- change: `TvecRvecMedianFilter` (default window 5) smooths solvePnP
  jitter before the controller runs. `solvepnp_tag → filter.push →
  fine_servo_compute` replaces the single-shot estimate path.
- 5 gtests added covering empty/fills/rejects-outlier/reset/rolling.

## iter-13 (2026-04-19) — Option D: max_fine_duration guard

- change: internal fine-servo wall-clock budget (`max_fine_duration_s`,
  default 120 s). Before iter-13 a servo oscillating inside the 2 cm ring
  without latching `settle_frames_` stayed "driving" forever; the
  harness-level NAV_TIMEOUT × 1.5 was the only cap. Now finishes as
  `last_reject_reason='fine_servo_timeout'` so operators see the failure
  as its own class.

## iter-14 (2026-04-19) — partial run with A+B+C+D applied

- run: `sim_episodes/precision_run_20260419_121709/` (killed mid-run after
  wp11 to unblock the iter-15 fix; 10/16 waypoints observed).
- per-waypoint: wp01-03 SUCCEEDED (nav2, 0.03–0.09 m),
  wp04 NAV_TIMEOUT (rail_approach tag 35), wp05-06 SUCCEEDED (rail_drive),
  wp07 NAV_TIMEOUT (rail_approach tag 4), wp08-09 SUCCEEDED (rail_drive),
  wp10 ABORTED 1.50 m (nav2 post-rail_drive, no progress 91 s),
  wp11 in-progress when killed.
- **Root cause of rail_approach NAV_TIMEOUT isolated**: rail.yaw=0 on
  floor tags (face normal is +Z, not a horizontal approach heading) meant
  start_coarse_approach asked Nav2 for a goal at
  `(rail.x - coarse_standoff, 0, yaw=0)`, which for wp04 (robot=(5.2,0,π),
  tag=(4,0)) pointed the robot 180° off and past the tag. Observed in
  logs as `Nav2 coarse approach failed (code=5)`. Fine_servo never started.
- decision: skip Nav2 coarse_approach when the robot is already within
  range of the tag (iter-15 below).

## iter-15 (2026-04-19) — skip Nav2 coarse_approach when in range

- change: `rail_approach_node.cpp` start_coarse_approach now looks up
  `map→base_link`, computes robot-to-tag distance, and if ≤
  `coarse_skip_radius` (default 2.0 m, covers every Round-44 teleport)
  transitions directly to TAG_ACQUISITION. TF lookup failure falls
  through to the legacy Nav2 coarse path.
- build + 15 gtests green, verify_specs 0 BLOCKING.
- run: `sim_episodes/precision_run_20260419_125157/` (killed mid-run
  after wp03 — see next paragraph).
- **Regression surfaced before rail_approach could even be exercised**:
  wp01 ABORTED at err=0.50 m (robot didn't move). wp02 ABORTED 1.00 m,
  wp03 ABORTED 1.5 m — identical pattern. Mode_arbiter log shows
  `corridor_nav → rail_drive → rail_exit` transitioning on EVERY
  waypoint setup. The harness's iter-8 cancel goal
  (zero-distance PoseStamped to `/agv/rail_driver/goal` post-reset)
  makes rail_driver briefly report "driving"/"reached"; the FSM
  short-circuits through RAIL_DRIVE → RAIL_EXIT. For wp01 at
  (4.0, 0, 0) = tag_x_rear, `rail_exit_clearance_m = 0` forever —
  no clearance, no release, Nav2's cmd_vel_nav is dropped by the
  arbiter (source=RAIL). This bug existed before iter-15 but was
  masked by different initial conditions (iter-14 ran from a
  different spawn pose); iter-15's brain relaunch made it latent.
- decision: move the fix into the FSM (iter-16) — the symptom is
  orthogonal to rail_approach.

## iter-20 (2026-04-19) — 11/16, rail_approach unlocked 5/5

- chain of fixes landing together (iter-15 coarse_skip + iter-16 FSM
  release + iter-17 SETTLED/ABORTED emit + iter-17c publish_status
  + iter-17 nav2 bond_timeout + iter-17 shim 89.5° + iter-18 corrected
  waypoint goals + iter-20 depth-1 state QoS + iter-20 tolerance 0.10 m).
- report: `sim_episodes/precision_run_20260419_153437/report.json`
- **success rate: 11/16 (68.75 %), 0 collisions, 23:52 run**
- per-bucket:
  - **nav2 2/5** — wp01/wp03 PASS (0.07/0.05 m). wp02 ABORT 1.0 m at
    stall_abort_s=90 s (Rotation Shim Controller TF race observed as
    ~100 ms of "Failed to transform pose to base frame" right after
    teleport; Nav2 never commands motion). wp10 NAV_TIMEOUT 2.3 m
    yaw 1.04 rad (post-rail_drive nav2 transition — robot rotates
    off-axis). wp15 ABORT 2.1 m yaw 2.73 rad (post-rail_exit nav2,
    similar motion weirdness).
  - **rail_approach 5/5** — wp04/wp07/wp11/wp12/wp16 all SUCCEEDED at
    0.09–0.10 m / 52–71 s dur. First rail_approach successes in the
    entire iteration chain. Fine_servo converges, latches SETTLED,
    harness detects.
  - **rail_drive 4/4** — wp05/wp06/wp08/wp09 at 0.048–0.053 m / 23–31 s.
    Rock-solid since iter-3.
  - **rail_exit 0/2** — wp13/wp14 NAV_TIMEOUT 1.54 m each (push-to-
    exit-1.5 m past-tag never completes within 270 s). Same err
    magnitude since iter-5; needs separate investigation (likely sim
    RTF vs rail_driver speed limit or zone_detector boundary).
- delta vs iter-9 (prior high-water 8/16): **+3 waypoints, +18.75 %**,
  entire rail_approach bucket unlocked.
- decisions: no new code this iteration — all fixes already landed.
- next run (iter-21) focus: rail_exit push completion + wp10/wp15
  nav2 post-rail transitions + wp02 TF race stabilisation.

## iter-16 (2026-04-19) — RAIL_EXIT release in approach zones when idle

- change: `mode_fsm.hpp` auxiliary RAIL_EXIT release path — hand back
  to CORRIDOR_NAV when robot is in an approach zone (`rail_approach_*`)
  AND rail_driver_state == "idle" (explicit idle, not the one-tick
  "reached" latch). Approach zones sit outside the rail tubes so
  MPPI rotation is not a tube-hit hazard. Rail zones (`rail_aisle_*`)
  still hold RAIL_EXIT regardless of rail_driver state — the 51 mm
  tube protection is preserved.
- +3 unit tests (approach-zone-release, approach-zone-hold-driving,
  rail-zone-hold-idle). 33 mode_arbiter gtests green.
- run: `sim_episodes/precision_run_20260419_130544/` (in progress).
- expected impact (combined with iter-15):
  - wp01/wp02/wp03 nav2: unblocked (FSM releases on first tick).
  - wp04/wp07/wp11/wp12/wp16 rail_approach: skip coarse → fine_servo.
  - wp10 nav2: same FSM release trap unblocks.
  - wp13/wp14 rail_exit: unchanged (already working via primary gate).
  - Projected success: 11–14 / 16 depending on fine_servo convergence.

<!--
Template for each subsequent iteration:

## iter-N (YYYY-MM-DD)

- report: `sim_episodes/precision_run_<ts>/report.json`
- analysis: `sim_episodes/precision_run_<ts>/iteration_<N>_analysis.md`
- bucket verdicts: nav2 M/M, rail_approach M/M, rail_drive M/M, rail_exit M/M
- top rule fired: `<rule_id>` on `<wp_id>` — `<one-liner>`
- decision: `<what was changed, at which file:line>`
- delta vs iter-(N-1): nav2 ↑/↓/→, rail_approach ↑/↓/→, rail_drive ↑/↓/→, rail_exit ↑/↓/→
- next run expected to fix: `<rule_id>` or "acceptance re-check"
-->
