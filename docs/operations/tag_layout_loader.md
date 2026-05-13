# Tag Layout Loader — operator guide

The Tag Layout Loader lets the operator define the physical AprilTag
inventory of a greenhouse in one of two complementary ways:

1. **Modo 1 (YAML Import)**: write the layout in YAML on a laptop,
   upload it through the dashboard. Best for cold-start and for
   bulk edits.
2. **Modo 3 (Robot Probe)**: drive the robot in front of each
   physical tag and capture its in-situ pose. Best for refining a
   layout where the printed positions might be slightly off, and
   for adding tags one-by-one after a partial install.

Both modes write to the same backend store. The runtime registry
consumed by `marker_correction` and `rail_approach` is regenerated
automatically and the `/agv/markers/registry_reload` topic is
published, so neither node needs a restart.

## File layout on the Jetson

| Path | Purpose |
|---|---|
| `${AGV_DATA_DIR}/tags/current_layout.yaml` | operator-facing layout (rich schema with `role`, `rail_id`, `yaw_deg`, metadata). The single human-editable record. |
| `${AGV_DATA_DIR}/runtime_markers_registry.yaml` | auto-generated, consumed by ROS. Minimal schema (`id`, `x`, `y`, `z`, `yaw` radians, `type`). DO NOT edit by hand. |
| `${AGV_DATA_DIR}/apriltags.json` | internal manager state (defined_tags + hardware_assignments). Auto-managed. |
| `${AGV_DATA_DIR}/tags/history/<ts>_<reason>.yaml` | snapshot of the previous layout file before each apply. 7-day retention is NOT enforced today; manual cleanup if needed. |
| `${AGV_DATA_DIR}/tags/examples/sample_layout.yaml` | canonical sample copied at boot. Operator may overwrite. |

## Modo 1 — YAML Import

### Schema

```yaml
metadata:
  greenhouse_name: "ExampleFarm-Site-3"
  block_id: "block_A"
  schema_version: 1
  notes: |
    Free-form operator notes about this layout.

defaults:
  family: tag36h11
  size: 0.20

tags:
  - id: 99                       # hardware AprilTag id (integer printed on the tag)
    role: charging               # charging | rail_entry | central_aisle_beacon | handoff | other
    label: "charging_dock"       # optional, free-form
    pose:
      x: 0.0                     # meters, map frame
      y: -2.5
      z: 0.10
      yaw_deg: 0.0               # degrees, in [-180, 180]

  - id: 100
    role: rail_entry
    rail_id: "rail_1_north"      # required when role == rail_entry
    pose: { x: 1.0, y: 1.5, z: 0.10, yaw_deg: 90.0 }
    size: 0.20                   # optional per-tag override of defaults.size
```

### Validation rules

The backend rejects with HTTP 400 + a list of `{index, id, field,
message}` errors if any of the following fail:

| Rule | Message |
|---|---|
| `id` is a non-negative integer | "id must be a non-negative integer" |
| `id` unique within the file | "duplicate id N" |
| `role` in {`charging`, `rail_entry`, `central_aisle_beacon`, `handoff`, `other`} | "role must be one of …" |
| `rail_id` non-empty string when `role == rail_entry` | "rail_id is required when role=rail_entry" |
| `pose.{x, y, z, yaw_deg}` all finite numbers | "X must be a finite number, got Y" |
| `yaw_deg` in `[-180, 180]` | "yaw_deg must be in [-180, 180], got Z" |
| `size` in `(0, 1)` m if present | "size must be in (0, 1) m, got Z" |
| No two tags at the exact same `(x, y, z)` | "pose collides with tag id N at same (x,y,z)" |

### UI flow

1. Open the dashboard → **AprilTags** panel.
2. Scroll to **Layout (YAML)** section.
3. Click the file picker, choose your `.yaml`. The preview table shows
   each tag the validator parsed; if there are errors, they're listed
   in red and the Apply button stays disabled.
4. Click **Apply layout**. Confirmation modal asks to confirm replacing
   the existing layout. Operator confirms → backend persists, runtime
   registry regenerates, reload topic published.
5. **Download current** exports the active layout as YAML.
6. **Download example** downloads the sample for editing.

### Acceptance tests (operator runs these)

Use the fixtures committed at `test_fixtures/tag_layouts/`:

- **Test 1.1**: import `valid_layout_6_tags.yaml`. Preview shows 6 tags, Apply succeeds.
- **Test 1.2**: import `invalid_nan_yaw.yaml`. Error: `pose.yaw_deg must be a finite number`.
- **Test 1.3**: import `invalid_duplicate_id.yaml`. Error: `duplicate id 100`.
- **Test 1.4**: import `invalid_yaw_out_of_range.yaml`. Error: `yaw_deg must be in [-180, 180]`.
- **Test 1.5**: after a successful apply, on the Jetson:

      ls ~/agv_data/tags/current_layout.yaml
      ls ~/agv_data/runtime_markers_registry.yaml
      sudo journalctl -u agv.service | grep "Reloading marker registry"

- **Test 1.6**: open the map view. Imported tags appear as colored
  circles with orientation indicators (short line from centre).
- **Test 1.7**: `sudo systemctl restart agv-dashboard.service`. After
  the dashboard reloads, the imported tags are still listed in the
  panel and rendered on the map.

## Modo 3 — Robot Probe in-situ

### Workflow

1. Drive the robot manually (teleop) so it faces a physical AprilTag.
2. Confirm in the **System Health Panel** that
   `Localization (auto_init_orchestrator)` is **LOCALIZED** (or
   **DEGRADED**, which is accepted with a warning).
3. In the AprilTags panel → **Robot Probe (in-situ)** section, click
   **Open probe…**.
4. The modal polls `/api/tags/probe/status` every 500 ms. Wait until
   it shows the current tag's id and its map-frame pose.
5. Set **Role** from the dropdown. If `rail_entry`, fill **Rail ID**.
6. Click **Confirm & save**. The button is disabled until:
   - Localization is `LOCALIZED` or `DEGRADED`.
   - A tag has been detected in the last 2 s.
   - If role is `rail_entry`, rail_id is non-empty.

The endpoint either ADDS a new tag (hardware id was not yet in the
layout) or UPDATES the existing entry (hardware id was already
assigned). Either way, the runtime registry regenerates and the
reload topic publishes.

### Pose source

The probe captures the pose currently being published on
`/agv/marker_pose`. Because that topic carries the
marker-corrected ROBOT pose (not the tag's own pose), the captured
position is most accurate when the robot is directly in front of the
tag at ~30-50 cm distance. The "future improvement" listed in
`docs/agent/future_work.md` is to have `marker_correction` publish a
dedicated tag-pose-in-world topic for higher-fidelity capture.

### Acceptance tests (operator runs these)

- **Test 3.1**: with `LOCALIZATION_FAILED` forced (e.g., kill auto_init_orchestrator), open the probe modal. Confirm & save is disabled with a clear message.
- **Test 3.2**: with no tag visible to the camera, open the probe modal. The detection panel shows "Waiting for tag detection…".
- **Test 3.3**: drive in front of a real tag, robot localized. Open the modal → tag id + pose appear → set role + (rail_id if applicable) → Confirm & save. Tag appears on the map; backend log confirms.
- **Test 3.4**: probe a tag that was previously in the layout from Modo 1. The save UPDATES rather than ADDS. Compare the new pose to the YAML pose: deltas should be small (sub-decimeter, sub-degree).
- **Test 3.5**: probe a brand-new tag id. The save ADDS it; total tag count increases by 1.

## MapView visualization

After an import or a probe save, the map view (TopBar → main) shows
each tag as a colored circle with a short line indicating the tag's
yaw direction. Hover for `#id · label · role · yaw=N°`.

Role-color legend:

| Role | Color |
|---|---|
| `rail_entry` | blue |
| `charging` | amber |
| `central_aisle_beacon` | green |
| `handoff` | purple |
| `other` | gray |

The colors are inferred from the `label` field after import (since
the legacy `DefinedTag` only carries `wall`/`rail_start`). To get
correct colors, keep the `role` in the layout YAML and the loader
will set the label to `<role>_<hw_id>` automatically.

## Troubleshooting

- **"validate returned 401"**: the dashboard sent the request without
  the bearer token. Re-log in or refresh the page.
- **"runtime registry didn't reload"**: check `journalctl -u agv.service
  -n 50 | grep markers`. The reload topic uses `transient_local +
  reliable` QoS; subscribers should receive it on connect.
- **probe modal stuck at "Waiting for tag detection…"**: the camera
  isn't seeing a tag, or `marker_correction` is down. Check the
  System Health Panel `marker_correction` row.

## Implementation references

- Backend manager: `src/agv_ui_backend/src/apriltag_manager.ts`
- Routes: `src/agv_ui_backend/src/routes/tags.ts`
- Frontend panel: `web/agv_dashboard/src/components/panels/AprilTagsPanel.tsx`
- MapView: `web/agv_dashboard/src/components/MapView.tsx` (tag layer)
- ROS consumer: `src/agv_markers/src/marker_correction_node.cpp`
- Spec: `specs/persistence.yaml`, `specs/interfaces.yaml`
