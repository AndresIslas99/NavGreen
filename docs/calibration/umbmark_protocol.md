# UMBmark Protocol — Borenstein & Feng (1996)

This is the bidirectional-square calibration test used to separate two
classes of systematic odometry error in differential-drive robots:

- **Type A** (α): unequal wheel diameters → causes a curved trajectory
  on a commanded straight line.
- **Type B** (β): mismeasured wheelbase → causes over- or under-rotation
  on a commanded turn.

Both errors compound when the robot moves around a closed path. Critical
insight from the paper: **a single CW square hides Type-A and Type-B
errors because they cancel.** The same is true for a single CCW square.
Only by measuring the *difference* between CW and CCW closures do they
separate cleanly.

Source: J. Borenstein & L. Feng, "Measurement and Correction of
Systematic Odometry Errors in Mobile Robots," IEEE T-RA, Dec 1996.
[PDF mirror](https://johnloomis.org/ece445/topics/odometry/borenstein/paper58.pdf).

## When to run UMBmark

- Before trusting `wheel_radius` and `track_width` from a previous
  calibration session (e.g. before claiming a code change "fixed" the
  odom bias).
- After any change to wheel hardware: tire wear, payload that
  affects deflection, gear-ratio modifications, motor swap.
- Before applying any correction derived from the baseline harness
  (`docs/calibration/baseline_protocol.md`). The baseline tells you
  *that* a bias exists; UMBmark tells you *whether geometry is the
  cause*.

## When NOT to run

- If `tools/calib_apriltag_probe.py` reports < 30 hits/3 s — the
  closure measurement requires a stable AprilTag.
- If the floor is sloped or the surface produces visibly different
  slip in different directions (e.g. wet patches). UMBmark assumes
  the surface is uniform.

## Required space

A free corridor of **`side + 1 m`** in both X and Y. Example: for the
default `--side 4.0`, you need a clear 5 m × 5 m area. If the
greenhouse aisle width forbids 4 m, run with `--side 2.0` and accept
that the angular resolution of α and β degrades by 4× (α scales with
1/L). Borenstein recommends ≥ 4 m for production-grade calibration.

## Hardware setup

- AGV in `mode=teleop`, motors armed, dashboard open in another tab
  for the e-stop.
- AprilTag id=12 visible from BOTH the start pose AND each of the
  4 corners. (For a 4-m square, the tag's effective range needs to
  cover the diagonal ≈ 5.7 m. If the tag isn't visible from the far
  corners, that's fine — `/agv/odometry/global` is the closure
  reference, and the EKF fuses cuVSLAM during the corners. The tag
  is the absolute lock at the start/end of each square.)
- A way to mark the starting pose physically (chalk, masking tape) so
  the operator can reposition the robot consistently.

## Running a session

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/agv_cyclonedds_runtime.xml

python3 tools/calib_umbmark.py \
    --side 4.0 --runs 5 --mode operator
```

The script prompts the operator at each corner. The operator drives
the robot via the dashboard joystick:

1. Mark the start pose. ENTER.
2. Drive forward 4 m (one side). ENTER.
3. Turn 90° in place (left for CCW, right for CW). ENTER.
4. Repeat for the remaining 3 sides.
5. The script logs `closure = end_pose - start_pose` (in start frame).
6. Reposition robot to start, repeat for the other direction.
7. Repeat the (CW, CCW) pair `--runs` times.

## Reading the output

The script prints, after all runs:

```
α (Type-A wheel-Ø asymmetry)  = +X.YYYY°
β (Type-B wheelbase error)    = +X.YYYY°
Ed = ...   →  r_left = ..., r_right = ...
Eb = ...   →  track_width = ...
Residual estimate after correction: X.YY% (wheel/wheelbase)
```

**Decision rule**:

- **Residual ≤ 1%**: corrections are robust. Apply to
  `src/agv_odrive/config/odrive_params.yaml`. Re-run UMBmark and
  verify residual ≤ 0.5%.

- **Residual 1–5%**: corrections improve geometry but a non-geometric
  contribution exists (slip, payload, surface). Apply geometry first,
  then proceed to Phase 2 (slip detector).

- **Residual > 5%**: do **NOT** apply corrections. Geometry is not
  the dominant cause. Likely sources, in order of probability:
  1. Caster slip on the test surface (compare against
     `docs/calibration/baseline_protocol.md` results — same surface?)
  2. Payload distribution shifted (e.g., camera mount loose).
  3. Wheel surface contamination (oil, dust on the tires).
  4. ODrive parameter drift (re-run motor calibration on ODrive itself
     via `odrivetool`).
  In all 1–4 cases, applying the UMBmark corrections will overcompensate
  in one regime and undercompensate in another. Skip directly to
  Phase 2 (slip detector) and revisit UMBmark afterwards.

## Frozen UMBmark results

| Date | Surface | Side L | N runs | α (deg) | β (deg) | Ed | Eb | Residual | Action taken |
|------|---------|--------|--------|---------|---------|----|----|----------|--------------|
| _(no UMBmark sessions yet — this row is the placeholder; populate after first run)_ |

Add a row after each UMBmark session committed to `main`. Reference
the commit SHA in the "Action taken" column.

## Math reference (Borenstein eqs. 13-16)

Given the centroid of the closure-error cluster on the X axis of the
robot frame for CW (x_cg_cw) and CCW (x_cg_ccw):

```
α = (x_cg_cw + x_cg_ccw) / (-4 * L)        [rad]
β = (x_cg_cw - x_cg_ccw) / (-4 * L)        [rad]

Ed = (90° + α) / (90° - α)                 [right/left wheel ∅ ratio]
Eb = 90°       / (90° - β)                 [effective wheelbase scale]
```

Corrections (one valid factoring):

```
r_left  = r_nominal * 2     / (Ed + 1)
r_right = r_nominal * 2 Ed  / (Ed + 1)
b       = b_nominal * Eb
```

After applying, re-run UMBmark; residual α and β should be near zero.
If they're not, the dominant error is not in the geometry parameters
this calibration covers.
