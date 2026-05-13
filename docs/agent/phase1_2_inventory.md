# Sub-fase 1.2 — inventory of the existing AprilTag stack

**Date:** 2026-05-13
**Author:** agent (Sub-fase 1.2 §1.2.0 deliverable)
**Purpose:** document what's already in the workspace BEFORE writing
any new Tag Layout Loader code. Per the prompt anti-pattern rules,
extend the existing infrastructure rather than replace it.

This document synthesises three parallel investigations (ROS side,
UI/backend side, consumers + spec).

---

## 1. The runtime registry is the integration boundary

### 1.1 File path and schema

The single load-bearing artifact every consumer agrees on:

| | |
|---|---|
| Path | `${AGV_DATA_DIR}/runtime_markers_registry.yaml` |
| Top-level key | `markers:` (YAML sequence) |
| Per-entry fields | `id` (int, required, ≥ 0, unique) <br> `x`, `y`, `z`, `yaw`, `size` (float, optional, default 0.0) <br> `type` (string, optional — only `"rail_start"` is consumed today) |
| Yaw units | **RADIANS** |
| Parser | yaml-cpp via `marker_correction_node.cpp:246-319` (HIGH-04-04 closed) |
| Writer (today) | `agv_ui_backend/src/apriltag_manager.ts::writeRegistry` |
| Readers (today) | `agv_markers/marker_correction_node`, `agv_rail_approach/rail_approach_node` |
| Spec | `specs/persistence.yaml:224-236` |

Per-entry example:

```yaml
markers:
  - id: 0
    x: -16.88
    y: 0.00
    z: 0.145
    yaw: 0.0
    size: 0.20
  - id: 100
    x: 1.00
    y: 1.50
    z: 0.10
    yaw: 1.5708    # π/2 = 90°
    type: rail_start
    size: 0.20
```

The schema is permissive — every field except `id` falls back to 0
silently. Loaders should defend at write time, not at read time.

### 1.2 Reload mechanism

Both consumers subscribe to one topic with `transient_local + reliable`
QoS:

| Topic | `/agv/markers/registry_reload` |
|---|---|
| Type | `std_msgs/msg/Empty` |
| Subscribers | `marker_correction_node` (cb `reload_all_registries()` at `marker_correction_node.cpp:199-206`) <br> `rail_approach_node` (cb `reload_all_registries()` at `rail_approach_node.cpp:149-155`) |
| Publisher today | `agv_ui_backend/src/apriltag_manager.ts` after each `writeRegistry()` |
| QoS detail | `rclcpp::QoS(1).transient_local().reliable()` |

A single publication after writing the runtime registry triggers both
consumers to re-read from disk and rebuild their in-memory caches.
**Atomic-rename of the YAML before publish is recommended** so a
mid-read consumer never sees a half-written file (current
implementation does NOT do this — note the gap below).

---

## 2. The existing UI/backend pipeline

### 2.1 `apriltag_manager.ts` is the SoT for tag definitions

Existing manager at `src/agv_ui_backend/src/apriltag_manager.ts` owns:

- Persistence to `${AGV_DATA_DIR}/apriltags.json` (the rich
  human-edited record).
- Generation of `${AGV_DATA_DIR}/runtime_markers_registry.yaml` from
  the JSON (the consumer-facing minimal record).
- The `/agv/markers/registry_reload` publication after every write.
- The pending-detection queue and assignment workflow.

Methods that the new layout loader can reuse directly:
- `addDefinedTag(label, description, x, y, yaw_rad, type, z?)`
- `updateDefinedTag(id, fields)`
- `deleteDefinedTag(id)`
- `getDefinedTags()`
- `getRegistryYamlPath()`

The rich record:

```ts
interface DefinedTag {
  id: number
  label: string
  description: string
  type: 'wall' | 'rail_start'
  x: number
  y: number
  z: number
  yaw: number        // RADIANS, even though the UI typed it in degrees
  created_at: number
}
```

### 2.2 Existing routes (`routes/apriltags.ts`)

CRUD already wired:

| Method | URL | Purpose |
|---|---|---|
| GET | `/api/apriltags` | List defined + hardware assignments + pending |
| POST | `/api/apriltags/defined` | Create one tag from form |
| PUT | `/api/apriltags/defined/:id` | Update one tag |
| DELETE | `/api/apriltags/defined/:id` | Delete one tag |
| POST | `/api/apriltags/assign` | Assign hw_id to a defined tag |
| DELETE | `/api/apriltags/assignment/:hw_id` | Unassign |
| POST | `/api/apriltags/dismiss/:hw_id` | Dismiss pending |
| POST | `/api/apriltags/:id/navigate` | Send nav goal at tag pose |
| POST | `/api/apriltags/:hw_id/align` | Fine-servo align skip-Nav2 |

What's **missing** (the Sub-fase 1.2 scope):
- Bulk YAML import
- Bulk YAML export
- In-situ robot probe
- Layout history / atomic apply

### 2.3 Existing frontend

`web/agv_dashboard/src/components/panels/AprilTagsPanel.tsx`:
- Form-based one-tag-at-a-time entry.
- Yaw in degrees in the UI, converted to radians before sending.
- Polls `GET /api/apriltags` every 2 s.

`web/agv_dashboard/src/components/AprilTagAssignmentModal.tsx`:
- Auto-pops when a pending hardware detection arrives via WS.
- Operator picks which defined tag this hardware ID corresponds to.

The existing panel structure naturally accommodates the new
features as additional sections rather than a new panel.

---

## 3. Consumers of the registry

| Node | What it cares about | Breaks if … |
|---|---|---|
| `marker_correction_node` | All registry entries — uses `(x, y, z, yaw)` for solvePnP-based map-frame correction. | Top-level key isn't `markers:`. Field types not numeric. `id` missing. yaml-cpp parse error → previous in-memory registry preserved (silent rollback). |
| `rail_approach_node` | Only entries with `type: rail_start` (or floor tags with `z ≤ 0.05` auto-classified). Uses `(id, x, y, yaw, size)`. Service `rail_approach/execute` does `rail_starts_.find(tag_id)`; unknown ID → "Unknown rail start tag ID". | Registry missing. `type` key spelled differently. Rail tag dropped from registry between writes. |
| `auto_init_orchestrator` | **Indirect**. Reads `/agv/marker_pose` (the corrected robot pose), not the registry itself. | Only impacted if `marker_correction` stops publishing because the registry is unparseable. |

**Invariant**: any new write must preserve the schema (`markers:`
sequence with at least `id` per entry), and must publish the reload
event when done.

---

## 4. Audit findings still relevant

| Finding | Status | Code path | Implication for 1.2 |
|---|---|---|---|
| HIGH-04-02 (incidence-angle filter) | **CLOSED** in Sprint E.lite (commit `831f56d`) | `marker_correction_node.cpp:287-321` | filter exists; only affects detection-time quality. No impact on registry loader. |
| HIGH-04-04 (homebrew YAML parser) | **CLOSED** in Sprint E.lite (commit `9c90a36`) | `marker_correction_node.cpp:235-339` (yaml-cpp) | parser is robust; loader can trust it. |

No HIGH-04-* findings remain open relevant to this sub-fase.

---

## 5. Gap inventory

Things the existing stack does NOT do (= scope of Sub-fase 1.2):

1. **No bulk YAML import**. Operator must currently type each tag's
   pose manually into the form.
2. **No bulk YAML export**. No way to download the active layout.
3. **No in-situ robot probe**. Operator can't drive up to a tag and
   say "this is where this tag is — save its pose from where I am".
4. **No MapView rendering of tags**. Tags exist in the system but
   are invisible in the spatial UI.
5. **No history of layout changes**. apriltag_manager keeps the
   latest JSON; previous states are lost.
6. **No atomic rename on runtime registry write**. A consumer that
   loads exactly during a write may see a half-written file. Low
   probability in practice (writes are small) but not a guarantee.

---

## 6. Decision: extend, do not replace

### 6.1 What stays as-is

- `apriltag_manager.ts` stays as the source-of-truth manager. The
  rich JSON record + runtime YAML emitter are correct.
- `routes/apriltags.ts` existing endpoints stay. Operator workflows
  built on them continue to work.
- `AprilTagsPanel.tsx` keeps its form + list + assignment workflow.
- `marker_correction` + `rail_approach` consume the same
  `runtime_markers_registry.yaml`. No changes.
- `/agv/markers/registry_reload` semantics unchanged.

### 6.2 What's added

- New methods on `AprilTagManager`:
  - `bulkImportLayout(parsedTags, replaceMode)` — validates a
    parsed layout, atomically swaps the on-disk JSON, regenerates
    YAML + publishes reload.
  - `addProbedTag(id, role, rail_id, x, y, z, yaw_rad, size?)` —
    convenience for in-situ saves; equivalent to `addDefinedTag`
    + sets `type=rail_start` when role is `rail_entry`.
  - `historyDir()` — returns path for history snapshots.

- New routes:
  - `POST /api/tags/layout/validate` — parse + validate, don't persist
  - `POST /api/tags/layout/apply` — parse + validate + persist + reload
  - `GET /api/tags/layout/current` — export active layout as YAML
  - `GET /api/tags/layout/example` — sample YAML the operator can edit
  - `GET /api/tags/probe/status` — live detection + localization gate
  - `POST /api/tags/probe/save` — save probed tag with metadata

- New frontend components:
  - Section in `AprilTagsPanel.tsx` for Layout Import (file picker
    + preview table + apply button).
  - Modal for Robot Probe.
  - MapView tag markers with role-color + freshness coloring.

- Persistence layout (additive only):
  - `${AGV_DATA_DIR}/tags/current_layout.yaml` — the operator-facing
    layout in the new schema (yaw_deg, role, rail_id, metadata).
    apriltag_manager.ts's existing JSON stays as the internal record.
  - `${AGV_DATA_DIR}/tags/history/<timestamp>_<reason>.yaml` —
    snapshot on every successful apply or probe-save.
  - `${AGV_DATA_DIR}/tags/examples/sample_layout.yaml` — committed
    sample shipped with the package; copied here at first boot.

### 6.3 Schema conversion (the operator yaml ↔ runtime registry)

The operator's `tag_layout_v1.yaml` carries more metadata than the
runtime registry. Mapping at apply time:

| Operator-facing | Runtime registry | Notes |
|---|---|---|
| `pose.yaw_deg` | `yaw` | × π/180 |
| `pose.x`, `pose.y`, `pose.z` | `x`, `y`, `z` | direct |
| `role: rail_entry` | `type: rail_start` | rename (legacy registry term) |
| `role: charging` / `central_aisle_beacon` / `handoff` / `other` | (no `type` field) | runtime registry ignores non-rail roles |
| `rail_id` | (stored in JSON record only, not in registry) | rail_approach matches by tag_id, not by rail_id string |
| `size` | `size` | direct |
| `family` | (stored in JSON record only) | runtime registry has one global family per the marker_correction param |
| `metadata.*` | (JSON record only) | for operator audit, not consumed by ROS |

### 6.4 What's deliberately NOT in scope (→ future_work.md)

- Atomic rename of `runtime_markers_registry.yaml` (low-prob race;
  apriltag_manager today writes-then-publishes, which is fine in
  practice).
- Per-tag-family in the registry (single-family today is sufficient).
- `handoff` role wiring into multi-map flows.
- The dual-store (JSON + YAML) is documented as a known pattern. A
  future cleanup could collapse to one store; not blocking 1.2.

---

## 7. Implementation order (matches §2-§5 of the prompt)

1. Schema doc + JSON-schema-style validators + history dir setup.
2. New `apriltag_manager` methods + sample YAML asset.
3. `/api/tags/layout/*` endpoints + `validate` and `apply` flows.
4. Frontend layout section + tests fixtures.
5. `/api/tags/probe/*` endpoints + WS or polling for live detection.
6. Probe modal frontend.
7. MapView tag markers.
8. Operator-run tests (Modo 1, Modo 3, MapView).
9. Final report.

The agent does NOT mark anything `CLOSED-VERIFIED-HW` until Andrés
confirms empirically per the lessons learned in
`docs/agent/lessons_learned.md`.
