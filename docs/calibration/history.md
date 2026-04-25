# Diff-drive Odometry Calibration — History

This file is the rolling log of every calibration session committed to
`main`. Each row links to the CSV under `tools/calib_runs/` and the
commit SHA that introduced the corresponding code change. The intent
is to make regressions visible: if a row's `Δodom/Δtag` is worse than
the previous row, the change introduced a regression.

The protocol for filling this out lives in
`docs/calibration/baseline_protocol.md`. Use the script
`tools/calib_diff_drive_baseline.py` for baseline sessions and
`tools/calib_umbmark.py` for UMBmark sessions.

## Baseline sessions

| Date       | Commit  | Surface          | Payload | N legs | Median Δodom/Δtag | Std  | Phase | CSV |
|------------|---------|------------------|---------|--------|--------------------|------|-------|-----|
| 2026-04-25 | 72ccce1 | polished ceramic | 0 kg    | 30 (14 large) | **+1.205 (+20.5%)** | 0.13 | pre-Phase 0 | (stdout-only — no CSV; reconstructed in plan note) |

Each row should reference the commit SHA that's on `main` at the time
of measurement. After Phases 1–5 land, expect new rows with progressively
lower bias.

## UMBmark sessions

| Date | Commit | Side L | N runs | α (deg) | β (deg) | Ed | Eb | Residual | Action |
|------|--------|--------|--------|---------|---------|----|----|----------|--------|
| _(no UMBmark sessions yet)_ |

## Slip detector tunings

| Date | Commit | yaw_thr | vx_thr | min_active | settle | Δodom/Δtag pre | post |
|------|--------|---------|--------|------------|--------|----------------|------|
| _(no tunings yet — populate after first session with the slip detector enabled)_ |

## Decision log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-04-25 | Replace `caster_covariance_multiplier=10` in `odrive_can_node` with explicit slip detector node | `robot_localization` documentation explicitly discourages covariance inflation as ignore-mechanism. Brossard 2019 and De Giorgi 2024 demonstrate detector-based approaches with order-of-magnitude better drift. |
| 2026-04-25 | Skip Phase 3 (caster instrumentation) for now | Out of scope; Phases 1+2+4-advisory close ~80% of the gap without hardware. Revisit post-Chada if residual bias remains. |
| 2026-04-25 | Phase 4 implemented as advisor (passive observer), not actuator | Mutating `/agv/cmd_vel` from a sniffer node would conflict with `mode_arbiter`'s ownership semantics. Closing the loop is post-Chada work. |

## How to add a new row

1. Run the protocol against the current `main` branch (or your branch
   if comparing).
2. Capture the CSV under `tools/calib_runs/`. Commit it.
3. Append a row to the appropriate table here. The "Commit" column is
   the commit that landed the corresponding code change (or `main`
   HEAD if measurement-only).
4. If a row regresses, file a "Phase NN regression" issue and link it
   from this row.
