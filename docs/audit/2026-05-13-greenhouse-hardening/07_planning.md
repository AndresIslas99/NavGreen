# Phase 7 — Planning and Control

> Phase 7 goal (per prompt): "El robot llega a su destino de forma
> eficiente, suave y segura, sin oscilaciones ni stalls."

Static analysis of Nav2 stack: costmaps, MPPI controller (full critic
scale audit), behavior trees, smoother + HAL consistency, and
collision_monitor. The team's investment in Nav2 is visible —
configurations are well-commented and engineered, but six concrete
issues emerge from a careful read.

---

## A. Costmap inflation arithmetic — the "saturation" check

The first-pass concern was: in 1.0–1.5 m greenhouse corridors, does
`inflation_radius` saturate the center of the corridor and freeze the
planner?

[`nav2_params.yaml:300-311`](../../../src/agv_navigation/config/nav2_params.yaml):

```yaml
inflation_layer:
  cost_scaling_factor: 3.0   # was 2.0 → 3.0 in audit
  inflation_radius: 0.55     # was 1.0 → 0.55 in audit
```

Footprint: `[[0.50, 0.37], [0.50, -0.37], [-0.30, -0.37], [-0.30, 0.37]]`,
i.e., inscribed radius `r = 0.37 m`.

For a corridor of width `W` with walls at `y = ±W/2`:
- Both walls inflate inward by `inflation_radius = 0.55 m`.
- The inflated obstacle reaches `|y| = W/2 - 0.55 m`.
- Free space (cost < 253) survives if `W/2 - 0.55 > 0` → `W > 1.1 m`.
- For Nav2 to plan, robot center must fit: footprint half-width 0.37 m
  must be smaller than free-space width → `W > 1.1 + 0.37 = 1.47 m` for
  *cost-free* centering, OR `W > 0.74 + 0` for *any* center inside the
  costmap (accepting non-zero inflation cost).

In a 2.0 m corridor (the team's stated target per the YAML comment
"sized for 2m corridors" at line 311):
- Free-space band: `2.0 - 2 × 0.55 = 0.90 m wide`.
- Footprint width 0.74 m fits with 0.08 m margin each side.
- **The robot can plan a centered path** with the entire footprint
  inside the inflated penumbra but not in lethal cells.

In a **1.5 m corridor** (the prompt's lower bound):
- Free-space band: `1.5 - 1.1 = 0.40 m`.
- Footprint width 0.74 m **does not fit** in the free band.
- The robot must plan a path where its footprint partially overlaps
  the inflation, with cost = `253 × exp(-3.0 × distance)`. At
  `distance = 0` (touching the inflated edge), cost = 253 — effectively
  lethal. The planner refuses.
- **In a 1.5 m corridor, the current configuration prevents planning.**

In a **1.2 m corridor** (some greenhouse rows are this tight):
- Free-space band: `0.10 m`.
- Footprint cannot fit. Total saturation.

This is `MEDIUM-07-08` below (NOT critical because the team designed
for 2.0 m corridors per the comment; the question is just whether
greenhouse rows are actually 2 m wide — that is a Phase 8 question).

### Findings

---

#### MEDIUM-07-08 — Costmap inflation parameters are sized for 2 m corridors; do not survive 1.5 m or tighter
**File(s)**: `src/agv_navigation/config/nav2_params.yaml:300-311,362-367`.
**Category**: architecture.
**Symptom**: With `inflation_radius = 0.55 m` and footprint inscribed
radius 0.37 m, the minimum corridor width for cost-free planning is
~1.47 m. Below that, planning is forced to traverse inflated cells.
At 1.1 m or below, the planner refuses outright.
**Analysis**: The YAML comment at line 304 acknowledges this — "value
1.0 produced 2.74m total footprint+inflation in a 2.0m corridor". So
the team explicitly designed for 2 m corridors. The greenhouse spec
in the prompt mentions 1.0–1.5 m. If the actual greenhouse has 1.5 m
corridors, this config will not plan.
**Greenhouse impact**: First field test in a real 1.5 m row → robot
sits at the start, planner returns "no plan found" repeatedly, dashboard
shows red — no clear diagnostic that it's a costmap issue.
**Recommendation**:
1. **Measure the actual corridor widths** in the target greenhouse
   before deployment. Document in `docs/deployment/corridor_widths.md`.
2. If 1.5 m corridors exist: reduce `inflation_radius` to 0.40 m and
   raise `cost_scaling_factor` to 5.0 to keep the gradient steep.
   Check that the resulting free band (1.5 − 0.8 = 0.7 m) accommodates
   the footprint (0.74 m wide) — barely. Field-validate.
3. If 1.0–1.2 m corridors exist: this configuration cannot navigate
   them. Either physically widen, or change to a corridor-following
   controller (the rail_driver / rail_approach stack the team built
   for the rail aisle case) for those zones.
4. Add a `verify_corridor_clearance.py` script that, given the loaded
   map + footprint + inflation, reports the narrowest plannable corridor.
**Acceptance criterion**: A field test in the actual target greenhouse
shows the planner produces feasible paths through every corridor the
robot is expected to traverse.
**Effort**: S (config tune) → M (script + verify).
**Prerequisites**: Phase 5 (mapping) shows real corridor widths.

---

## B. MPPI critic scale audit

[`nav2_params.yaml:128-188`](../../../src/agv_navigation/config/nav2_params.yaml).

The prompt §7.B requires a full critic scale audit. The 8 critics
configured today, with their weights and natural scales:

| Critic | Weight | Returns (Nav2 MPPI source) | Notes |
|---|---|---|---|
| `ConstraintCritic` | 4.0 | Sum of trajectory points violating kinematic constraints (`vx_max`, `wz_max`); each violation contributes ~`|vel_excess|`, ~`[0, vx_max] = [0, 0.25]` magnitude | Tiny per-point contribution |
| `GoalCritic` | 5.0 | Distance from trajectory end to goal pose, `[0, goal_distance]` ~`[0, prune_distance = 1.0]` m | O(1 m) magnitude |
| `GoalAngleCritic` | 3.0 | Angle error to goal at trajectory end, `[0, π]` rad | O(π) magnitude |
| `PreferForwardCritic` | 12.0 | Sum of penalty terms for negative samples in trajectory; non-zero only if `vx < 0` somewhere | Should always be ~0 with `vx_min=0`; kept as guard |
| `ObstaclesCritic` | (composite) | Two terms: `repulsion_weight × dist_to_obstacle` + `critical_weight × in_collision`. `collision_cost = 10000` | **Dominates** when in collision: 10000 × 20 = 200000 magnitude |
| ` ↳ repulsion_weight` | 1.5 | Penalty for proximity, scaled by costmap cost `[0, 252]` ~ O(252) | |
| ` ↳ critical_weight` | 20.0 | Penalty for predicted collision, multiplied by `collision_cost = 10000` | |
| `PathAlignCritic` | 20.0 | Lateral distance from trajectory to reference path, `[0, max_path_occupancy_ratio × ...]` ~ O(0.1 m) | Per-point sum, O(0.1) × `trajectory_point_step=4` × 32 samples |
| `PathFollowCritic` | 7.0 | Look-ahead alignment cost, O(0.1) | |
| `PathAngleCritic` | 2.0 | Angle between trajectory tangent and reference path, O(0.1 rad) | |

The **effective cost contributions** per trajectory (rough order of magnitude):

| Critic | Weight × scale | Effective magnitude |
|---|---|---|
| ConstraintCritic | 4 × 0 (typically) | ~0 |
| GoalCritic | 5 × 1 m | 5 |
| GoalAngleCritic | 3 × 1 rad | 3 |
| PreferForwardCritic | 12 × 0 | ~0 |
| ObstaclesCritic (free space) | 20 × 0 + 1.5 × 50 | 75 |
| ObstaclesCritic (in collision) | 20 × 10000 + 1.5 × 252 | **200378** |
| PathAlignCritic | 20 × 32 × 4 × 0.05 | 128 |
| PathFollowCritic | 7 × 0.1 | 0.7 |
| PathAngleCritic | 2 × 0.1 | 0.2 |

**Observations**:
1. `ObstaclesCritic` in collision **dwarfs everything else** by 1000×.
   That's correct behavior — collisions must be infeasible.
2. **In free space**, `PathAlignCritic` (128) dominates `GoalCritic`
   (5) by 25×. This is consistent with the comment at line 167:
   "now the primary smoothness driver, beats PreferForward". Good
   intent.
3. **GoalCritic at 5 and GoalAngleCritic at 3** are very weak. The
   `near_goal_distance: 0.5` and `threshold_to_consider: 1.4` mean
   they only kick in near the goal — but their magnitude (~5) is
   dominated by `PathAlignCritic` (128) until very close to the goal.
   Result: the robot tracks the path until the last few centimeters,
   then has very weak pull toward the goal. The 0.15 m
   `xy_goal_tolerance` may be hard to hit if `PathAlignCritic` is
   pulling sideways.
4. **PathFollowCritic at 0.7 effective magnitude** is essentially
   noise — its 7.0 weight is too small for its sub-meter scale.
5. `PreferForwardCritic` weight 12.0 with effective magnitude ~0
   (because `vx_min=0` already prevents negative samples) is **dead
   weight**. The comment at line 65 says "out-weighs PathAlignCritic
   (14.0) so MPPI never sacrifices forward-ness for path alignment".
   But (a) the comment is stale (PathAlign is now 20, not 14); and
   (b) PreferForward is only triggered by negative samples that don't
   exist. The constraint is enforced via `vx_min`, not via the critic.

### Findings

---

#### HIGH-07-01 — MPPI critic weights are not on a normalised scale; PathAlignCritic effectively pinches goal convergence
**File(s)**: `src/agv_navigation/config/nav2_params.yaml:128-188`.
**Category**: bug / performance.
**Symptom**: Critic effective magnitudes range over 4 orders (0.2 to
128 in free space). `PathAlignCritic` at 128 dominates `GoalCritic`
at 5 even very close to the goal, so the robot prefers to stay on the
path over reaching the goal pose.
**Analysis**: The tuning was done iteratively in 2026-04-12 (per
comment line 167) with the explicit intent of beating
`PreferForwardCritic`. The arithmetic works for "follow the path", but
the goal-convergence behavior is fragile. The 0.15 m `xy_goal_tolerance`
+ `RotationShimController.rotate_to_goal_heading: true` partially
compensate (the robot rotates in place to the final heading once
inside tolerance), but the lateral approach to the goal still suffers.
**Greenhouse impact**: At the last waypoint of a row mission, the
robot may oscillate close to the goal, hunting between the path's
final point and the goal's exact x/y. The `xy_goal_tolerance: 0.15`
absorbs this for casual missions, but for precision docking (rail
approach) this would fail without the dedicated rail_approach
controller.
**Benchmark**: Nav2 MPPI tutorials by Steve Macenski suggest scaling
critics so each one's free-space contribution is within an order of
magnitude of the others, with `ObstaclesCritic.critical_weight`
explicitly multi-orders larger to enforce collision avoidance. The
recommended tuning practice is to normalise critic outputs to `[0, 1]`
internally and apply weights, not the current "raw cost magnitudes
× weight".
**Recommendation**:
1. Add an MPPI tuning ADR (`docs/adr/0004-mppi-critic-tuning.md`) that
   records the effective magnitudes computed above and justifies each
   weight in terms of "what behavior this critic shapes".
2. **Specifically**: cut `PathAlignCritic.cost_weight` from 20 → 8 and
   raise `GoalCritic.cost_weight` from 5 → 12 to balance them within an
   order of magnitude near the goal. Re-validate path-tracking error.
3. Consider removing `PreferForwardCritic` entirely since `vx_min: 0`
   already enforces forward motion. The critic is dead weight.
4. Increase `PathFollowCritic.cost_weight` from 7 → 25 so its
   look-ahead contribution is meaningful, not noise.
**Acceptance criterion**: HIL `test_waypoint_precision.py` p95 error
≤ 0.10 m holds after the tune, AND the "search at goal" behavior the
operator reported (per nav2_params.yaml:217 comment) disappears.
**Effort**: M (tune + HIL regression + ADR).
**Prerequisites**: HIL harness available (it is — `agv_integration_tests`).

---

## C. Velocity smoother vs HAL — silent rate-limiting

[`velocity_smoother.yaml:13-16`](../../../src/agv_navigation/config/velocity_smoother.yaml):
```
max_velocity: [0.5, 0.0, 0.3]
min_velocity: [-0.1, 0.0, -0.3]
max_accel:    [0.5, 0.0, 0.8]
max_decel:    [-1.0, 0.0, -1.0]
```

[`odrive_params.yaml:25-26`](../../../src/agv_odrive/config/odrive_params.yaml):
```
max_wheel_accel: 0.5   # turns/s²
max_wheel_decel: 1.5   # turns/s² (3x faster than accel)
```

Conversion (using physical `wheel_radius = 0.0625 m` per the
caliper measurement, see `CRITICAL-02-02`):
- 1 wheel turn = `2π × 0.0625 = 0.393 m` of linear travel.
- `max_wheel_accel = 0.5 turns/s² = 0.196 m/s²` linear.
- `max_wheel_decel = 1.5 turns/s² = 0.589 m/s²` linear.

**Comparison**:
| | Smoother (asked) | HAL (allowed) | Verdict |
|---|---|---|---|
| `max_accel` | 0.5 m/s² | **0.196 m/s²** | Smoother asks for 2.6× more than HAL allows |
| `max_decel` | 1.0 m/s² | **0.589 m/s²** | Smoother asks for 1.7× more than HAL allows |
| `max_velocity.x` | 0.5 m/s | (MPPI capped at 0.25 m/s) | OK; MPPI is the actual cap |
| `min_velocity.x` | -0.1 m/s | (controller forward-only) | OK; should be 0 for consistency though |
| `max_velocity.theta` | 0.3 rad/s | (MPPI capped at wz_max=1.5) | Smoother is the tight cap on angular |

### Findings

---

#### HIGH-07-02 — Smoother accel/decel exceeds HAL limits — HAL silently rate-limits the smoothed command
**File(s)**:
- `src/agv_navigation/config/velocity_smoother.yaml:15-16`
- `src/agv_odrive/config/odrive_params.yaml:25-26`
- `src/agv_odrive/CLAUDE.md` "Asymmetric accel limiter" notes.

**Category**: bug / performance.
**Symptom**: Smoother runs `OPEN_LOOP` at 50 Hz with accel 0.5 m/s² and
decel 1.0 m/s². HAL caps accel at 0.196 m/s² and decel at 0.589 m/s²
(with physical wheel_radius 0.0625 m). The HAL silently rate-limits
beyond what the smoother sent.
**Analysis**: The smoother in `OPEN_LOOP` mode does **not** receive
feedback from the HAL about how fast it actually accelerated. The HAL
applies its own rate limiter on top, so the actual wheel velocity ramps
slower than the smoother predicts. Two effects:
- **MPPI tracks the wrong dynamics model**. MPPI's internal kinematic
  model assumes commands are executed instantly (subject to `vx_max`,
  `wz_max`). The smoother bridges that gap by ramping. If the HAL adds
  another ramp on top, MPPI's predicted vs actual trajectories diverge,
  the `model_dt = 0.05` horizon becomes wrong, and the controller's
  tracking error grows.
- **Stop-distance math is wrong** (see `MEDIUM-07-05`). The
  collision_monitor relies on a stopping distance computed at 1.0 m/s²
  decel, but actual decel is 0.589 m/s².
**Greenhouse impact**: Tracking errors on tight maneuvers; the robot
overshoots commanded path by approximately `(0.196 − applied) × t²/2`
during the early acceleration phase. Over a 1.6 s MPPI horizon, that's
~25 cm of unmodeled lag.
**Benchmark**: `ros2_control`'s `diff_drive_controller` reads the
controller's accel limits from the same YAML as the smoother and the
URDF. Single source of truth.
**Recommendation**:
1. Align values. Either:
   - **Loosen the HAL** (raise `max_wheel_accel` to a value that
     matches smoother + MPPI), and validate the wheels don't slip; OR
   - **Tighten the smoother** to 0.2 m/s² accel and 0.6 m/s² decel,
     accept slower response, recompute MPPI tuning.
2. The smoother's `feedback: OPEN_LOOP` is fragile here — switch to
   `CLOSED_LOOP` and feed `odometry/local` so the smoother knows the
   real velocity. The smoother will then output more aggressive
   commands if it sees the robot lagging, exposing the HAL as the
   bottleneck cleanly.
3. Centralise the limits in `robot_params.yaml#robot.dynamics` so the
   smoother + HAL + MPPI all read the same numbers.
**Acceptance criterion**: With `feedback: CLOSED_LOOP`, the
commanded-vs-actual velocity gap reported by the smoother diagnostics
remains under 10 % during normal operation.
**Effort**: M.
**Prerequisites**: CRITICAL-02-02 step 0 (NVRAM dump) — must know the
correct wheel kinematics before tuning these limits.

---

#### MEDIUM-07-05 — Stop-distance math in `collision_monitor.yaml:41` assumes 1.0 m/s² decel, real HAL gives 0.589 m/s²
**File(s)**: `src/agv_navigation/config/collision_monitor.yaml:38-46`.
**Category**: bug / safety.
**Symptom**: The comment at line 41 computes stop distance as
`v²/(2a) = 0.16/2.0 = 0.08 m (8 cm)` using `max_decel = 1.0 m/s²`. The
actual HAL decel limit is 0.589 m/s². Recomputed: stop distance =
`0.16 / 1.18 = 0.136 m` — **14 cm, not 8 cm**.
**Analysis**: The 20 cm stop_zone forward extent was designed with the
8 cm number plus 4 cm reaction plus 8 cm safety. With the real 14 cm,
the margin shrinks to 2 cm. Still positive — not safety-critical — but
the math in the comment is wrong and the engineering safety margin is
much thinner than advertised.
**Greenhouse impact**: A pedestrian entering the stop_zone at the
boundary will trigger a stop at 0.20 m from the robot's front. The
robot will stop 6 cm before the pedestrian instead of the documented
12 cm. Still safe, but closer.
**Recommendation**:
1. **Recompute the stop distance with the real HAL decel limits**
   (after `HIGH-07-02` aligns the smoother with the HAL).
2. **Update the YAML comment** to reflect actual numbers.
3. If the safety margin is judged too thin, extend the stop_zone front
   from 20 cm to 25 cm. The current 20 cm was sized for vx_max 0.4 m/s
   per the comment, but vx_max is now 0.25 m/s (see `MEDIUM-07-03`),
   so there's headroom.
**Acceptance criterion**: The YAML comment matches the live decel
limits. A field measurement (when hardware is available) of the actual
stop distance at vx_max = 0.25 m/s confirms < 0.20 m.
**Effort**: S.
**Prerequisites**: HIGH-07-02 (clarify the actual decel limit).

---

## D. RotationShimController — present and correct

The prompt §7.C lists RotationShim as an expected gap. **It is present
and correctly configured** at `nav2_params.yaml:85-97`:

```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "nav2_mppi_controller::MPPIController"
  angular_dist_threshold: 0.785       # 45°
  forward_sampling_distance: 0.5
  rotate_to_heading_angular_vel: 1.0
  max_angular_accel: 1.5
  rotate_to_goal_heading: true        # critical for forward-only
```

`rotate_to_goal_heading: true` is essential for forward-only operation
(without it, the robot cannot adjust final heading because MPPI's
`vx_min = 0` prevents backing into the correct heading). The team
clearly understood this — comment at line 81–84 is thorough.

**No finding.**

---

## E. Behavior Trees

### E.1 The production BT

[`src/agv_navigation/behavior_trees/navigate_to_pose_forward_only.xml`](../../../src/agv_navigation/behavior_trees/navigate_to_pose_forward_only.xml).

- Removes `BackUp` from the recovery RoundRobin (consistent with
  forward-only).
- Recovery sequence: `ClearingActions → Spin(90°) → Wait(5s) →
  Spin(180°)`. RoundRobin × 6 retries.
- `RateController hz=2.0` (replans 2× per second at 0.25 m/s = 12.5 cm
  staleness per plan).
- `ComputePathToPose` has its own RecoveryNode that clears the global
  costmap on failure.

**Strengths**: Forward-only contract is enforced in three places (MPPI
`vx_min`, `PreferForwardCritic`, BT). Defense in depth.

**Gap**: There is no mission-level timeout. The 6 retries per goal can
run for tens of seconds before the action client returns FAILURE. For
a mission of N waypoints, total worst-case time is unbounded if every
waypoint exercises full recoveries. The dashboard would show "in
progress" while the robot is effectively stuck.

### E.2 The behaviors/ package — `enable_behaviors=false`

`src/agv_behaviors/trees/` has 3 XMLs (`navigate_with_recovery.xml`,
`single_waypoint.xml`, `waypoint_patrol.xml`) but
`behavior_executor` is `enable_behaviors=false` by default
(`agv_full.launch.py:106`), so these BTs **never run in production**.
This was already noted in Phase 0 as dead branch.

Multi-waypoint missions go through `agv_waypoint_manager` instead,
which has its own action-client logic and the localization-gate bypass
(CR-00-06).

### Findings

---

#### MEDIUM-07-07 — No global mission timeout; stuck navigation only times out on per-goal RoundRobin
**File(s)**: `src/agv_navigation/behavior_trees/navigate_to_pose_forward_only.xml:58-86`.
**Category**: bug / failure mode.
**Symptom**: The BT has `number_of_retries="6"` on the recovery node,
but no overall mission timeout. A goal can consume `6 × (clearing +
Spin + Wait + Spin) ≈ 6 × 10s = 60s` of recoveries before reporting
failure. A 12-waypoint mission worst case = 12 minutes of stuck
recoveries before any human-visible signal.
**Analysis**: The dashboard shows the robot as "navigating" throughout
the recovery cycle. Operator has no clue something's wrong unless
they're watching the robot physically.
**Greenhouse impact**: Operator walks away from the dashboard to
attend to other tasks; robot gets stuck behind a maintenance cart in
row 3; operator returns 15 minutes later to find the mission has been
"navigating" the whole time.
**Recommendation**:
1. Add a `Timeout` decorator wrapping the main `PipelineSequence`. A
   reasonable timeout = (path length / vx_max) × 2 (50 % buffer).
2. Emit a clear event to the dashboard on timeout: "Goal X did not
   complete in T seconds — robot may be stuck".
3. For mission-level supervision in `agv_waypoint_manager`, add a
   per-waypoint deadline + a mission deadline. Cancel + alert.
**Acceptance criterion**: A simulated unreachable goal (e.g., the
mission's next waypoint is in a closed-off region of the map) is
cancelled within `(path_length / vx_max) × 2` seconds with a clear
dashboard message, instead of hanging indefinitely in recoveries.
**Effort**: S (BT timeout decorator) → M (mission-level supervision in
agv_waypoint_manager).
**Prerequisites**: none.

---

#### MEDIUM-07-06 — `backup` plugin loaded in behavior_server but no BT references it
**File(s)**: `src/agv_navigation/config/nav2_params.yaml:386`.
**Category**: NIT.
**Symptom**: `behavior_plugins: ["spin", "backup", "wait"]` loads
the BackUp recovery, but the custom BT removes it and no other BT in
the workspace calls it.
**Analysis**: The team kept it loaded so the default Nav2 BTs
(`navigate_through_poses`) can resolve their action references — see
the comment at line 380–385. Functional but wasteful.
**Recommendation**: Either (a) keep as-is (documented decision, no
operational cost), or (b) verify that no BT in the workspace ever
loads `navigate_through_poses` and drop the plugin. Low priority.
**Effort**: NIT.

---

## F. collision_monitor in detail

[`src/agv_navigation/config/collision_monitor.yaml`](../../../src/agv_navigation/config/collision_monitor.yaml).

### F.1 Strengths

- **Two-source defense in depth**: `scan_source` (LaserScan) AND
  `pointcloud_source` (raw ZED depth). The pointcloud catches obstacles
  above/below the laser slice.
- **`source_timeout: 0.5`** — fixed from a previous catastrophic 2.0.
- **`max_points: 1` for stop_zone** — single point triggers stop.
  Aggressive false-positive tolerance, correct for safety.
- **Polygons match physics**: stop_zone front = footprint + 20 cm =
  0.70 m extends. Stop distance at 0.25 m/s with 0.589 m/s² decel +
  reaction = ~17 cm. 20 cm has 3 cm margin (thin — see `MEDIUM-07-05`).

### F.2 Gaps

The stop_zone polygon is **not symmetric around the robot**, but **it
DOES extend to the rear**: `points: [0.70, 0.42, 0.70, -0.42, -0.35, -0.42, -0.35, 0.42]`
gives rear `x = -0.35 m` (footprint rear is -0.30 m, so +5 cm rear).

The robot is **forward-only** (per MPPI `vx_min: 0` and the BT). Rear
obstacle perception is **explicitly absent** (per
`agv_navigation/CLAUDE.md` "30 cm hardware blind zone"). So a rear
stop_zone:
- Cannot detect a rear obstacle (no rear sensor).
- Will still false-trigger if `scan_source` or `pointcloud_source`
  produces noise points in the rear region (the ZED FoV doesn't go
  there, but pointcloud noise from reflections might).

### Findings

---

#### MEDIUM-09-05 — Stop zone extends 5 cm to the rear despite forward-only architecture
**File(s)**: `src/agv_navigation/config/collision_monitor.yaml:54`.
**Category**: bug / docs.
**Symptom**: stop_zone polygon includes `x = -0.35` m (5 cm beyond
footprint rear). The robot has no rear sensor. The rear lobe of the
stop zone can only ever produce false positives from noise.
**Analysis**: Since the robot is forward-only, a "rear obstacle" in
the stop zone is either:
- Real and unavoidable (the robot can't reverse to clear it).
- Noise from depth pipeline reflections.
Either way, the rear lobe should be exactly at the footprint (0 cm
margin) so the zone matches the robot's reachable space.
**Greenhouse impact**: Low. The rear lobe is small and the robot is
unlikely to encounter real rear obstacles in normal forward navigation.
But false stops from sensor noise are operator-visible glitches that
erode trust.
**Recommendation**: Change stop_zone rear to match footprint:
`points: [0.70, 0.42, 0.70, -0.42, -0.30, -0.42, -0.30, 0.42]`. Add a
comment explaining why the rear is footprint-aligned (forward-only
architecture).
**Effort**: S.
**Prerequisites**: none.

---

## G. Documentation drift

### Findings

---

#### MEDIUM-07-03 — Documentation drift: `vx_max=0.4` in `agv_navigation/CLAUDE.md` vs `0.25` in `nav2_params.yaml`
**File(s)**:
- `src/agv_navigation/CLAUDE.md` table at the end ("Key Configuration"): `vx_max (MPPI) | 0.4 m/s`.
- `src/agv_navigation/config/nav2_params.yaml:108`: `vx_max: 0.25` (with detailed comment explaining the 0.25 cap).

**Category**: docs.
**Symptom**: The CLAUDE.md says 0.4 m/s; the YAML says 0.25 m/s. Both
have detailed engineering comments justifying their value.
**Analysis**: The YAML was updated in 2026-04-13 to cap at 0.25
(comment explains: "MPPI cannot demand a velocity that the safety chain
... cannot stop within the 20 cm stop_zone safety margin"). The
CLAUDE.md table was not updated.
**Recommendation**: One-line edit in `agv_navigation/CLAUDE.md`:
`vx_max (MPPI) | 0.25 m/s`.
**Effort**: NIT.

---

#### MEDIUM-07-04 — Documentation drift: stop_zone "footprint + 5cm" in CLAUDE.md, +20cm in YAML
**File(s)**:
- `src/agv_navigation/CLAUDE.md` "Key Configuration" table: `Stop zone | footprint + 5cm`.
- Same file earlier section ("L3"): `stop_zone polygon = footprint + 20cm front`.
- `src/agv_navigation/config/collision_monitor.yaml:36,53-54`: 20 cm front.

**Category**: docs.
**Symptom**: The CLAUDE.md contradicts itself: the prose says 20 cm,
the table at the bottom says 5 cm. The 5 cm is the historical pre-audit
value.
**Recommendation**: Update the table to match the prose (and the YAML).
**Effort**: NIT.

---

#### MEDIUM-02-06 — `nav2_params.yaml` header says "RegulatedPurePursuit", code is MPPI
**File(s)**: `src/agv_navigation/config/nav2_params.yaml:11-12`.
**Category**: docs.
**Symptom**: First-pass finding, restated. Header comment lies about
the controller.
**Recommendation**: One-line update.
**Effort**: NIT.

---

## H. Hardware-dependent items (not in this audit)

| Acceptance criterion (per prompt §7) | Harness |
|---|---|
| Path planning < 200 ms p99 | `ros2 topic hz -w 100 /agv/path` over 100 plan requests in HIL |
| Tracking error lateral RMS < 5 cm @ 0.5 m/s | Already lower-bounded by `test_waypoint_precision.py` at 0.10 m p95 |
| Jerk linear RMS < 0.5 m/s³, angular < 1 rad/s³ | Custom subscriber on `/agv/cmd_vel_safe`, second-difference of velocity, integrated over typical mission |
| Mission success rate ≥ 98 % over 100 runs | HIL with `agv_integration_tests` repeated 100×, scripted |
| Recovery rate ≥ 95 % | HIL with synthetic obstacles inserted mid-path |

These are all achievable through the existing HIL harness once Sprint A
findings close. Each becomes a row in `tests/planning/checklist.md`.

---

## I. Status

| Item | Status |
|---|---|
| Costmap inflation arithmetic vs corridor widths | ✅ filed `MEDIUM-07-08` |
| MPPI critic scale audit | ✅ filed `HIGH-07-01` |
| Smoother vs HAL accel limits | ✅ filed `HIGH-07-02` |
| Stop-distance math correction | ✅ filed `MEDIUM-07-05` |
| RotationShimController presence | ✅ verified — no finding |
| BT global timeout | ✅ filed `MEDIUM-07-07` |
| `backup` plugin loaded unused | ✅ filed `MEDIUM-07-06` (NIT) |
| Stop zone rear extension | ✅ filed `MEDIUM-09-05` (categorised under safety) |
| Documentation drift | ✅ filed `MEDIUM-07-03`, `MEDIUM-07-04`, `MEDIUM-02-06` (restated) |
| Hardware-dependent items | ⏸ deferred (§H) |

End of Phase 7. 10 findings: 2 HIGH, 7 MEDIUM, 0 LOW + 1 NIT.
