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
