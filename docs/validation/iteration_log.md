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
