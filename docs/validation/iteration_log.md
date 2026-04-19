# Iteration log — Round 44+

Running chronology of iterations against `waypoints_tagged_v3.yaml`. Each
entry is one line per stage with the decision taken. Keep it short —
details belong in the per-iteration analysis markdown referenced here.

Canonical loop + acceptance criteria: [iteration_runbook.md](iteration_runbook.md).

## iter-0 (2026-04-18) — harness enrichment baseline (pre-run)

- Round 44 code landed on `main`: rail_detector (J, bea3208), rail_driver
  visual (K, 360a624), mode_arbiter RAIL_EXIT + rail_exit_geometry (M1,
  6d1674f), harness oracle subscriptions + iteration_report.py (Q1–Q7).
- Unit test count: 86 green (22 + 4 + 19 + 9 + 17 + 11 + 4 across the
  packages listed in the runbook).
- No HIL run yet; first HIL sweep = iter-1.

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
