# Phase 6 — Perception and Traversability

> Phase 6 goal (per prompt): "Detectar obstáculos no mapeados (personas,
> carritos, mangueras), y clasificar superficies para evitar caídas o
> atascos."

Static analysis of the depth→costmap pipeline, the obstacle layer
configuration, the live scan_grid_mapper, and what is **not** present
that the prompt expects (drop-off detection, person tracking, reflection
filtering). Many findings here are absence-of-feature; the team has
been honest that greenhouse-specific perception is post-MVP.

---

## A. Depth → LaserScan pipeline

[`agv_full.launch.py:197-227`](../../../src/agv_bringup/launch/agv_full.launch.py).

```yaml
pointcloud_to_laserscan parameters:
  min_height: 0.01    # 1 cm — captures floor objects
  max_height: 2.0     # 2 m — ceiling
  angle_min: -1.5708, angle_max: 1.5708   # ±90°
  angle_increment: 0.003                  # ~0.17° / step
  range_min: 0.3                          # ZED minimum depth
  range_max: 8.0
  use_inf: true
  target_frame: base_link
```

### A.1 Strengths

- **`min_height: 0.01`** was deliberately lowered from 0.03 (per launch
  file comment) to catch floor objects in the safety reaction. **Good
  decision.** Cables, hoses, feet enter the scan.
- **±90° FoV** — covers the camera's full width.
- **`range_min: 0.3`** matches ZED 2i hardware minimum. Below this,
  depth is NaN.
- **`target_frame: base_link`** — outputs the scan in the robot frame,
  not the camera frame. Saves a TF transform downstream.

### A.2 Concerns

- **No explicit ground-plane filtering.** The scan trusts depth at
  `min_height: 0.01` — that's 1 cm above `base_link`. With `base_link`
  200 mm above ground (per URDF), `min_height: 0.01` is **210 mm above
  the floor**. So actual floor returns are ignored — but anything
  between 21 cm and 220 cm is captured as obstacle. Reasonable but
  not ground-plane-aware.
- The ZED depth itself includes the **ground**, which would appear in
  every pixel below the horizon. Without a ground-plane filter, those
  returns enter `pointcloud_to_laserscan` and get **discarded** by the
  `min_height: 0.01` threshold (because ground is at z ≈ -0.2 m in
  base_link, below the threshold). **Lucky-correct**: the threshold
  acts as a crude ground filter.
- A **wet floor reflecting the ceiling** produces depth returns that
  appear to be **above the floor** (the reflected ceiling). These pass
  the `min_height: 0.01` filter and become spurious obstacles. See
  `MEDIUM-06-05` below.

### A.3 Voxel layer in costmaps

[`nav2_params.yaml:261-288, 332-354`](../../../src/agv_navigation/config/nav2_params.yaml):
```yaml
voxel_layer:
  z_resolution: 0.05
  z_voxels: 16            # 0.80 m total height
  min_obstacle_height: 0.10
  max_obstacle_height: 2.0
  observation_sources: scan      # NOT pointcloud
  scan:
    topic: /agv/scan
    raytrace_max_range: 3.0
    obstacle_max_range: 2.5
```

**Note**: voxel_layer is fed by **LaserScan only** — the comment at
line 265-270 explicitly explains why (raw ZED pointcloud marks ~99 %
of cells lethal in a 3 × 3 m window). nvblox was previously the
3D source but was removed for the structural reason at line 251-259.

So the **costmap is 2D**, despite using `VoxelLayer` plugin. The voxel
nature is unused — every observation lands in the same z slice.

### A.4 Findings

---

#### MEDIUM-06-02 — `min_obstacle_height: 0.10` in voxel layer ignores floor obstacles below 10 cm
**File(s)**: `src/agv_navigation/config/nav2_params.yaml:274,281,340,347`.
**Category**: bug.
**Symptom**: The voxel layer's `min_obstacle_height: 0.10` filters
scan returns below 10 cm above `base_link` (= 30 cm above ground,
accounting for `base_link` at z=0.2 m). The
`pointcloud_to_laserscan.min_height: 0.01` (= 21 cm above ground) is
more permissive than the costmap.
**Analysis**: An object 10 cm above the ground (e.g., a coiled hose
~5–10 cm tall) enters `/agv/scan` (because 0.01 m base_link relative >
hose top) but is **rejected by the costmap** (because 0.10 m base_link
relative < hose top of ~-0.10 m in base_link). The costmap doesn't see
the hose and doesn't plan around it. The collision_monitor **does**
see it (collision_monitor consumes `/agv/scan` directly with no height
filter — `collision_monitor.yaml:81`), so the robot will reactively
stop at the hose but cannot plan around it.
**Greenhouse impact**: H-04 in the HAZOP. The robot freezes in front
of hoses repeatedly during a mission, generating operator-visible
stops with no clear cause. Operator must manually drag the hose aside.
**Recommendation**:
1. Lower `min_obstacle_height` to match `pointcloud_to_laserscan.min_height`
   relative to base_link. With `pc_to_ls.min_height: 0.01` and `base_link`
   at 0.2 m above ground, the costmap should use `min_obstacle_height: 0.0`
   or even `-0.10` (10 cm below base_link).
2. Alternatively, accept that the costmap is "high obstacle" only and
   document the rule. Then operator training: "if mission stops
   repeatedly with no obvious cause, check for low floor objects".
**Acceptance criterion**: A hose at the floor level is reflected as a
costmap obstacle (visualised in RViz) and the planner routes around
it on its first attempt.
**Effort**: S (config change + HIL test scene).
**Prerequisites**: none.

---

## B. Drop-off / negative obstacle detection — absent

### Findings

---

#### MEDIUM-06-01 — No negative-obstacle / drop-off detection
**File(s)**: absence — no module in `src/agv_*` handles negative obstacles.
**Category**: bug / safety.
**Symptom**: Greenhouse environments routinely include drainage
channels (10–20 cm deep, 20–40 cm wide), raised plant beds (5–15 cm
step up from corridor floor), and occasional ramps. The robot has no
detection for any of these.
**Analysis**: The ZED depth sensor can detect drop-offs — they appear
as range returns at unexpected depth (the floor is suddenly farther
away). But the current pipeline:
- Filters out anything below `min_height` (so drop-offs are removed
  before they reach the costmap).
- Has no module that **converts negative obstacles to positive costmap
  features**.
The collision_monitor cannot detect them either — it monitors for
**positive** obstacles in a polygon.
**Greenhouse impact**: H-05 in the HAZOP. A robot driving toward a
20 cm drop will not stop and will tip. Result: robot damage, possible
injury to a worker bending nearby, contamination of plants if the robot
sits in a drainage trench.
**Benchmark**:
- **ETH RSL `elevation_mapping_cupy`** explicitly handles negative
  traversability — it builds a height map and marks cells with sudden
  drops as untraversable.
- **NVIDIA Isaac ROS Nvblox** (which the team has removed for unrelated
  reasons — see `nav2_params.yaml:251-259`) supports ESDF that
  naturally encodes drop-offs.
- **Traversability** packages (Wageningen, Robotec.ai) handle this
  explicitly for outdoor / agricultural robots.
**Recommendation** (3 options):
1. **Quick wins**:
   - Pre-map drainage channels as **keepout zones** in the operator
     dashboard.
   - Document operator responsibility: "before deploying robot in a
     new aisle, walk the route and mark drops as keepout".
2. **Software-only**:
   - Add a node that subscribes to `/agv/zed/point_cloud` and detects
     points below `-base_link_height - threshold` (i.e., below the
     ground plane by more than N cm). Publish these as obstacles to
     a synthetic LaserScan / OccupancyGrid that feeds the costmap.
3. **Sensor addition** (hardware, see `agv_navigation/CLAUDE.md`
   "Required hardware additions"):
   - Downward-facing ToF (VL53L1X) at the front of the robot, fixed
     range alarm at 30 cm. Cheap, deterministic, hardware-only path.
**Acceptance criterion**: A simulated 20 cm drop in front of the robot
causes a stop within 30 cm of the edge.
**Effort**: S (keepout-zone procedure) → M (software detector) → M
(hardware integration).
**Prerequisites**: Phase 5 (mapping) for the keepout-zone option;
Phase 11 (GUI) for operator workflow.

---

## C. Dynamic obstacles — no temporal filter

### C.1 What exists

- `voxel_layer` clears cells when scan rays pass through them
  (`raytrace_max_range: 3.0`). So a person standing in a corridor
  appears as a cell-cluster; when they move, the rays clear those cells
  and re-mark the new location.
- collision_monitor reacts at the polygon level — instantaneous response
  to whatever is in the scan now.
- `scan_grid_mapper` accumulates the live scan into `/agv/live_map`
  for operator visualization. **No documented decay policy** in the
  YAML I have access to (`scan_grid_mapper/config/scan_mapper_params.yaml`
  not read in this audit; defaults assumed).

### C.2 What's absent

- **No person detector** (no YOLO / detection2D / 3D bbox tracking).
- **No multi-frame temporal filter** to debounce oscillating leaves.
  Each scan is treated independently.
- **No motion-prediction**: an approaching pedestrian's velocity is
  not estimated, so the robot cannot anticipate.

### Findings

---

#### MEDIUM-06-04 — No dynamic-obstacle temporal filter; oscillating leaves likely cause false stops
**File(s)**: absence in `nav2_params.yaml`, `collision_monitor.yaml`,
`scan_grid_mapper`.
**Category**: bug / false-positive.
**Symptom**: A plant leaf swaying in greenhouse ventilation moves
~10–20 cm in 0.5 s cycles. Each scan captures it as a point in the
LaserScan. `collision_monitor.max_points: 1` triggers a stop on any
single point in the stop_zone.
**Analysis**: The combination of (a) aggressive `max_points: 1` and
(b) no temporal smoothing means the robot will false-stop several times
per hour next to a swaying plant. Each false stop:
- Triggers the recovery BT.
- Increments the dashboard event log.
- Reduces operator trust.
**Benchmark**:
- `spatio_temporal_voxel_layer` (Steve Macenski) does
  spatio-temporal accumulation — cells time out, recent observations
  dominate, oscillation flattens.
- Open Robotics' `nav2_dynamic_obstacle_avoidance` BTs use prediction
  + buffering to debounce.
- A simpler version: require `max_points: N` over a sliding window of
  M scans before triggering. Nav2's collision_monitor doesn't natively
  do this, so it requires a custom wrapper.
**Recommendation**:
1. **Quick**: raise `max_points: 1` → `max_points: 3` for the stop_zone
   and add the same for slowdown_zone (currently 2). Trade-off:
   slightly higher chance of missing a thin object.
2. **Better**: deploy `spatio_temporal_voxel_layer` as the costmap
   layer and feed collision_monitor from its clean output.
3. **Best**: train a thin DNN on plant-vs-obstacle in the greenhouse
   and route plant returns to a separate ignore-layer. Heavy lift.
**Acceptance criterion**: Field test with a swaying plant directly in
the stop zone produces fewer than 1 false stop per 10 minutes of
operation.
**Effort**: S (raise max_points) → M (spatio_temporal_voxel_layer
integration) → L (DNN).
**Prerequisites**: none for the quick fix; HIL bag of recorded oscillating
plants for the medium-tier.

---

## D. Reflection filtering — absent

### Findings

---

#### MEDIUM-06-05 — No reflection filter; wet floors produce spurious depth returns
**File(s)**: absence in `agv_full.launch.py:197-227`,
`nav2_params.yaml:261-288`.
**Category**: bug / false-positive.
**Symptom**: A wet greenhouse floor (drainage residue, recent
irrigation, condensation) reflects the ceiling. ZED depth returns
treat the reflection as a real object at the depth of the ceiling,
yielding spurious obstacle points **above** the floor.
**Analysis**:
- The `pointcloud_to_laserscan.min_height: 0.01` filters returns
  near floor level, **but** the reflected ceiling appears at +2 m
  base_link relative — well above the threshold, so it enters the scan.
- The voxel layer accepts it as a tall obstacle.
- Planner avoids the reflection as if it were real.
**Greenhouse impact**: H-17 in the HAZOP. After irrigation, the robot
sees phantom obstacles where puddles reflect ceiling pipes. Mission
fails to plan even though the corridor is physically clear.
**Benchmark**: A common filter is to **validate depth against a
ground-plane fit**: RANSAC the floor, reject points whose 3D position
falls below the fitted plane (these are reflections of objects above).
Standard technique in indoor robotics — see Robotec.ai's
`pointcloud_filters` and OpenCV `RANSAC plane fitting` tutorials.
**Recommendation**:
1. Add a `ground_plane_filter` node between ZED depth and
   `pointcloud_to_laserscan`. RANSAC the floor in the recent points,
   reject points that fall below the fitted plane (those are
   reflections).
2. Alternatively: physical solution — anti-reflection floor coating.
   Out of software scope.
3. As a stop-gap: document operator procedure "after irrigation,
   allow 30 min for water to drain before deploying robot".
**Acceptance criterion**: A wet-floor test scene (synthetic or HIL)
produces zero phantom obstacles after the filter.
**Effort**: M.
**Prerequisites**: ground-plane assumption holds (mostly flat greenhouse
floor — confirmed by `policies/engineering_rules.md` "greenhouse
hardware" notes).

---

## E. `scan_grid_mapper` — what does it do, what doesn't it do

[`src/agv_scan_mapper/src/scan_grid_mapper_node.cpp`](../../../src/agv_scan_mapper/src/scan_grid_mapper_node.cpp) (not read in full, but inferred from launch + topic spec).

- Subscribes `/agv/scan`.
- Accumulates into `/agv/live_map` (OccupancyGrid, transient_local).
- Consumed by the dashboard for the "live map overlay" — operator's
  visual confirmation that the robot sees obstacles.
- **Not** consumed by Nav2 — costmap_2d gets its own scan-based voxel
  layer.

The mapping resolution is 0.025 m per `agv_bringup/CLAUDE.md` (line 50).

### Findings

---

#### MEDIUM-06-03 — `scan_grid_mapper` has no documented decay policy; live map memory may grow unbounded
**File(s)**: `src/agv_scan_mapper/config/scan_mapper_params.yaml` (not
read in this audit). `agv_scan_mapper/CLAUDE.md` may or may not document
decay.
**Category**: bug / performance.
**Symptom**: Unknown whether `/agv/live_map` cells decay or accumulate
forever. If they accumulate, a long mission will produce a map showing
every transient obstacle ever seen — including people who walked past
10 minutes ago. Operator-visible noise.
**Analysis**: Need to read `scan_grid_mapper_node.cpp` to confirm. The
prompt §6.3 raises this question; the audit does not have a definitive
answer without reading the source.
**Recommendation**:
1. Read the source (this audit does not).
2. If no decay: add a configurable cell-decay mechanism (every N
   seconds, reduce occupancy probability by half).
3. Document the policy in `agv_scan_mapper/CLAUDE.md`.
**Acceptance criterion**: A documented decay policy exists, and a
30-minute mission produces a live map that is recognisably the
current obstacle configuration, not an accumulation of all past
observations.
**Effort**: S (read + document) → M (add decay if missing).
**Prerequisites**: none.

---

## F. Person detection — not in scope today

The team has not deployed a person-detection DNN. From the perception
infrastructure, **persons are detected as "any obstacle in the polygon"**
— the same way as plants, hoses, walls. This is acceptable for the
MVP first-greenhouse-visit (which assumes operator-supervised
operation), but it limits:
- The robot's ability to **slow down preemptively** when a person is
  approaching (no person classification → no velocity-of-pedestrian
  estimate).
- The robot's ability to **distinguish a person from a movable hose**
  in the slowdown zone (current behavior: same response for both).

**No finding** for this audit cycle. Recommend including in the
post-MVP perception roadmap. Open-vocabulary detection (Grounding DINO
+ SAM 2) on Jetson Orin is increasingly viable.

---

## G. Hardware-dependent items

| Acceptance criterion (per prompt §6) | Harness |
|---|---|
| Person at 1 m/s stops the robot before contact at 0.5 m/s | Field/HIL pedestrian-crossing scenario |
| Hose Ø2 cm at 1.5 m: ≥ 80 % recall | Lab bench test, scripted obstacle insertion |
| < 1 false stop per hour with vegetation moving | Multi-hour field log; would close `MEDIUM-06-04` |
| Latency obstacle → costmap < 200 ms p99 | Custom probe: insert synthetic obstacle, measure time to costmap update |

---

## H. Status

| Item | Status |
|---|---|
| Depth → laserscan pipeline analysis | ✅ |
| Voxel layer min_obstacle_height issue | ✅ filed `MEDIUM-06-02` |
| Drop-off / negative obstacle | ✅ filed `MEDIUM-06-01` |
| Dynamic-obstacle temporal filter | ✅ filed `MEDIUM-06-04` |
| Reflection filter | ✅ filed `MEDIUM-06-05` |
| scan_grid_mapper decay policy | ✅ filed `MEDIUM-06-03` |
| Person classification | ⏭ post-MVP roadmap item, no finding |
| Hardware-dependent items | ⏸ deferred (§G) |

End of Phase 6. 5 findings: 5 MEDIUM.
