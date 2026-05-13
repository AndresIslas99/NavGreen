# Tag Layout Loader — test fixtures

YAML files for Sub-fase 1.2 acceptance tests. Operator (Andrés) feeds
these into the dashboard's AprilTags panel → "Layout (YAML)" section to
exercise the validator.

| File | Expected result | Test |
|---|---|---|
| `valid_layout_6_tags.yaml` | preview shows 6 tags, Apply enabled, applies cleanly | 1.1 |
| `invalid_nan_yaw.yaml` | error: "yaw_deg must be a finite number" | 1.2 |
| `invalid_duplicate_id.yaml` | error: "duplicate id 100" | 1.3 |
| `invalid_yaw_out_of_range.yaml` | error: "yaw_deg must be in [-180, 180]" | 1.4 |

After Test 1.1 succeeds, the operator can verify:
- `~/agv_data/tags/current_layout.yaml` contains the imported layout (Test 1.7).
- `~/agv_data/runtime_markers_registry.yaml` was regenerated.
- `~/agv_data/tags/history/<timestamp>_replace.yaml` snapshot (if previous layout existed).
- `ros2 topic info /agv/markers/registry_reload` shows the publication.
- `marker_correction` reloaded (look for "Reloading marker registry from disk" in
  `journalctl -u agv.service`).
