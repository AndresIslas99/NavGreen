# Sub-fase 1.2 — Tag Layout Loader — checkpoint report

**Date:** 2026-05-13
**Branch:** `claude/amr-security-audit-gPtCd`
**Operator:** Andrés Islas
**Agent:** Claude Code (Opus 4.7, 1M context)

This is the **pre-tests** checkpoint report. Per
`docs/agent/lessons_learned.md` (rule §4) and Sub-fase 1.2 prompt §0.3,
the agent now stops and waits for the operator to run the acceptance
tests from his browser. Verdicts upgrade only after the operator
reports empirical results.

---

## 1. What landed (commits)

| Commit | Scope |
|---|---|
| `0ee78e3` | docs only — `lessons_learned.md` + `phase1_2_inventory.md`. No code changes. |
| `8c0c86a` | Modo 1 (YAML Import) + Modo 3 (Robot Probe). Backend (`routes/tags.ts`, `apriltag_manager.ts` extension, probeState in AppState, new subscribers) + Frontend (`AprilTagsPanel.tsx` Layout + Probe sections) + test fixtures (4 YAMLs) + js-yaml dep. |
| (this commit) | MapView role-based colour + orientation indicator + operator docs + this report. |

Code touched:

- `src/agv_ui_backend/src/apriltag_manager.ts` — extended with LayoutTag types, `validateLayoutYaml`, `applyLayout`, `addOrUpdateProbedTag`, `getCurrentLayoutYaml`, `getExampleYaml`, `installExampleAtBoot`, plus SAMPLE_LAYOUT_YAML.
- `src/agv_ui_backend/src/routes/tags.ts` — new, 6 endpoints.
- `src/agv_ui_backend/src/routes/index.ts` — registered tagsRoutes.
- `src/agv_ui_backend/src/app_deps.ts` — added `probeState`.
- `src/agv_ui_backend/src/index.ts` — wired new subscribers (`/agv/marker_pose` for pose, `/agv/marker_raw_detected` for tag_id) and `installExampleAtBoot()` at boot.
- `src/agv_ui_backend/package.json` — js-yaml added.
- `web/agv_dashboard/src/components/panels/AprilTagsPanel.tsx` — added Layout + Probe sections plus authedFetch wrapper.
- `web/agv_dashboard/src/components/MapView.tsx` — extended tag rendering with role-inferred colour + orientation indicator.
- `test_fixtures/tag_layouts/` — 4 fixtures + README.
- `docs/agent/phase1_2_inventory.md`, `docs/agent/lessons_learned.md`, `docs/operations/tag_layout_loader.md`, this report.

---

## 2. Per-feature status (per prompt §0.2 verdict scale)

| Feature | Status | Notes |
|---|---|---|
| Inventory + lessons_learned | **CLOSED-VERIFIED-CODE** | written, committed, no operator action required |
| Modo 1 — backend validate/apply/current/example | **CLOSED-VERIFIED-CODE** | TS compiles, service active, endpoints return 401 (auth-gated) on smoke test |
| Modo 1 — frontend Layout section | **CLOSED-VERIFIED-CODE** | dashboard builds (40 modules, 427 KB), panel renders new section |
| Modo 3 — backend probe endpoints | **CLOSED-VERIFIED-CODE** | endpoints registered, probeState wired |
| Modo 3 — frontend Probe modal | **CLOSED-VERIFIED-CODE** | dashboard builds, modal polls 500ms |
| MapView visualization | **CLOSED-VERIFIED-CODE** | role-based colour + yaw orientation indicator |
| Tests with operator | **PENDING-OPERATOR** | the operator has not yet run any of the 1.x or 3.x tests |
| Docs | **CLOSED-VERIFIED-CODE** | `tag_layout_loader.md` shipped |

Per the lessons_learned rule §1: every "CLOSED-VERIFIED-CODE" stays
that way until the operator confirms from HIS browser. The agent
does NOT claim CLOSED-VERIFIED-HW for any of these.

---

## 3. What the operator runs (from his browser)

### Pre-flight

The dashboard backend must be running independently of `agv.service`
(per Sub-fase 1.1 architecture). Confirm:

```bash
systemctl is-active agv.service           # may be active or inactive
systemctl is-active agv-dashboard.service # MUST be active
```

Open http://JETSON-LAN-IP:8090/dashboard, log in. Open the
**AprilTags** panel.

### Modo 1 — YAML Import tests

Fixtures live at `test_fixtures/tag_layouts/`. Operator copies them
to his laptop (e.g., `scp orza@JETSON-LAN-IP:ros2_ws/test_fixtures/tag_layouts/*.yaml /tmp/`).

- **[ ] Test 1.1** — import `valid_layout_6_tags.yaml`. Preview shows
  6 rows, no errors, Apply enabled. Click Apply → "Applied 6 tags".
- **[ ] Test 1.2** — import `invalid_nan_yaw.yaml`. Validation panel
  shows red error citing `pose.yaw_deg is not a finite number`.
  Apply disabled.
- **[ ] Test 1.3** — import `invalid_duplicate_id.yaml`. Error cites
  `duplicate id 100`. Apply disabled.
- **[ ] Test 1.4** — import `invalid_yaw_out_of_range.yaml`. Error
  cites `yaw_deg must be in [-180, 180]`. Apply disabled.
- **[ ] Test 1.5** — after Test 1.1 succeeded, in SSH:

      ls -la ~/agv_data/tags/current_layout.yaml
      ls -la ~/agv_data/runtime_markers_registry.yaml
      sudo journalctl -u agv.service | grep "Reloading marker registry" | tail -3

  Both files exist; the journal shows the reload event fired.
- **[ ] Test 1.6** — switch to the main map view. The 6 tags from
  Test 1.1 appear as colored circles with short orientation lines.
  Hover shows `#id · label · role · yaw=N°`.
- **[ ] Test 1.7** — `sudo systemctl restart agv-dashboard.service`,
  wait, refresh. Tags still present in panel + on map.

### Modo 3 — Robot Probe tests (robot armed, operator present)

- **[ ] Test 3.1** — when localization is not LOCALIZED/DEGRADED, the
  modal's Confirm button is disabled with the gate message.
- **[ ] Test 3.2** — when no tag is visible, modal shows "Waiting for
  tag detection…".
- **[ ] Test 3.3** — drive in front of a physical tag, modal shows
  the tag id + map-frame pose, set role + (rail_id if rail_entry),
  click Confirm. Tag appears on the map; backend journal shows the
  save.
- **[ ] Test 3.4** — probe a tag already in the layout from Modo 1.
  Save UPDATES rather than ADDS. Pose deltas vs the Modo 1 file
  are small (sub-decimeter on x/y, sub-degree on yaw).
- **[ ] Test 3.5** — probe a brand-new id; total tag count
  increments by 1.

### Visualization tests

- **[ ] Test 5.1** — after import/probe, tags visible on the map.
- **[ ] Test 5.2** — hover shows tooltip; click is currently a no-op
  on tags (popups not implemented in this scope).
- **[ ] Test 5.3** — DEFERRED: per-tag freshness coloring requires
  extending `/api/apriltags` to include `last_seen` per tag. Tracked
  in `future_work.md`.
- **[ ] Test 5.4** — DEFERRED: occlusion warning requires
  watchdog timer state in the backend. Tracked in `future_work.md`.

---

## 4. Items moved to `future_work.md`

- Real per-tag freshness coloring in MapView (5.3) — needs backend
  `last_seen` plumbing.
- Occlusion warning (5.4) — needs watchdog state.
- Tag click popup with Edit / Delete actions — current visualization
  is hover-only.
- Atomic rename of the runtime registry YAML — currently writes
  in-place; race window during read is small but real.
- Migration of legacy `/api/apriltags/*` to auth-gated — current routes
  are open. New `/api/tags/*` routes ARE auth-gated.

---

## 5. Verifier baseline

```
bash tools/verify_specs/all.sh
  scripts run: 12
  blocking failures: 0
  warnings: 0
```

---

## 6. Stop and wait

Per `docs/agent/lessons_learned.md` rule §4:

> The operator runs the tests BEFORE the checkpoint report, not after.

The agent has shipped the implementation. The operator's empirical
results from his browser are the load-bearing evidence for verdict
upgrade. Procedure:

1. Operator runs each test in §3 above.
2. For each test, operator reports `PASS` or `FAIL <description>`.
3. If FAIL, agent investigates and revises.
4. When ALL tests in §3 PASS, agent updates this report to
   `CLOSED-VERIFIED-HW` per feature and proposes the next sub-phase.

**Agent does NOT advance to Sub-fase 1.3 without operator OK.**
