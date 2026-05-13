# Sub-fase 1.1 — final closure report

**Date:** 2026-05-13
**Branch:** `claude/amr-security-audit-gPtCd`
**Operator:** Andrés Islas
**Agent:** Claude Code (Opus 4.7, 1M context)

This report supersedes `phase1_1_checkpoint_report.md`. The follow-up
prompt explicitly required Sub-fase 1.1 to be re-classified
`INCOMPLETE` after the prior `CLOSED-VERIFIED-HW` verdict on
1.1.b.full was empirically disproven (trauma scenario still
failed). This report documents the genuine closure.

---

## 1. Verdicts per work unit

| Work | Status | Anchor commit(s) |
|---|---|---|
| 1.1.a `verify_topic_types.py` | **CLOSED-VERIFIED-HW** | `b051147`, `b264054` |
| 1.1.b proxy + status endpoint | **CLOSED-VERIFIED-CODE** | `12f2771` |
| 1.1.b.full server-first (in-process) | **CLOSED-VERIFIED-HW** for the in-process behaviour | `b79c148` |
| **1.1.b.full systemd separation** (the real trauma fix) | **CLOSED-VERIFIED-CODE (agent-side)** — pending operator-browser verification | this commit |
| 1.1.c health backend + restart endpoint | **CLOSED-VERIFIED-HW** (verifier-from-UI test PASSED) | `fd5b04c`, `a1f37ee` |
| 1.1.c 5 health probes wired | **CLOSED-VERIFIED-CODE** | this commit |
| chrony install | **CLOSED-VERIFIED-HW** | this commit |

Verifier baseline: `bash tools/verify_specs/all.sh` → 12 scripts,
0 blocking, 0 warnings.

---

## 2. The genuine trauma fix — systemd separation

### Diagnosis (from `phase1_1_followup_log.md`)

Before: `agv.service` ran `ros2 launch agv_bringup
agv_full.launch.py`, which included `Node(package='agv_ui_backend',
executable='teleop_backend')` as a child action. `systemctl stop
agv.service` SIGTERMed `ros2 launch`, which killed all its
children including the backend. Port 8090 went dead. Dashboard
became unreachable. **The "server-first" refactor (b79c148)
protected against rclnodejs failing inside the process, but not
against the process being SIGTERMed by systemd.**

### Fix

| Change | File |
|---|---|
| New systemd unit `agv-dashboard.service` | `/etc/systemd/system/` (deployed); SoT at `src/agv_ui_backend/systemd/agv-dashboard.service` |
| Removed `Node(teleop_backend)` from launch | `src/agv_bringup/launch/agv_full.launch.py:848-877` → comment block |
| Backend exec sources ROS before node | unit's `ExecStart` is `/bin/bash -c 'source ... && exec node ...'` |

The two units (`agv.service` and `agv-dashboard.service`) are now
fully independent. Either can be stopped without affecting the
other.

### Empirical verification (agent shell)

```
Step 5: sudo systemctl stop agv.service
        agv.service: inactive
        agv-dashboard.service: active   ← survived
        curl /api/system/ros_status →
            HTTP 200, {"status":"online", "detail":"ROS bridge active"}
        curl /api/auth/status →
            HTTP 200, {"enabled":true}
```

The dashboard backend keeps responding after `agv.service` is
stopped. **The trauma scenario is closed at the systemd level.**

### Operator-side verification — REQUIRED for `CLOSED-VERIFIED-HW`

Per prompt §2.3, the operator must run Test 2 from THEIR browser
and confirm:

```
[ ] sudo systemctl stop agv.service   (from terminal or SSH)
[ ] Refresh dashboard at http://JETSON-LAN-IP:8090/dashboard
[ ] Page loads, login screen appears (or stays logged in if token cached)
[ ] System Health Panel shows agv.service in red
[ ] sudo systemctl start agv.service
[ ] Wait ~30 s
[ ] Panel shows agv.service transitioning back to green
[ ] ROS topics back to green status
```

The verdict moves from `CLOSED-VERIFIED-CODE (agent-side)` to
`CLOSED-VERIFIED-HW` once Andrés signs off on the above.

---

## 3. 5 health probes wired

Previously 4 components (ekf_local, cuvslam, marker_correction,
safety_supervisor) rendered as `?` / `no health probe wired` in
the panel because the backend didn't subscribe to their topics.
This commit adds the subscribers + state fields.

(Collision Monitor — the 5th in the operator's prompt list — was
already wired via `state.collisionMonitor.updated`.)

Files:
- `src/agv_ui_backend/src/app_deps.ts`: 4 new `lastXTime` fields.
- `src/agv_ui_backend/src/index.ts`: 4 new `node.createSubscription`
  calls inside `createAllSubscriptions()`. Each stamps the state
  field on every message (no decoding).
- `src/agv_ui_backend/src/health_monitor.ts`: switch updated to
  read the new state fields.

Backend restart confirms all subscribers register successfully:
`[ROS] All subscriptions created` in the log with no errors.

---

## 4. chrony

Installed via `sudo apt-get install -y chrony` (after a temporary
DNS workaround — `8.8.8.8` added to `/etc/resolv.conf` because the
WiFi gateway was unresponsive). Default Ubuntu pool sources.

`chronyc tracking` post-install:
- Reference ID: `AC68D1CC (172-104-209-204.ip.linodeusercontent.com)`
- Stratum: 5
- System time offset: 0.000026661 s (≈26 µs)
- Source: ntp.ubuntu.com pool, reachable

The panel's `Clock Sync (chrony)` row uses
`health_monitor.ts:checkChrony` which parses `chronyc tracking`
output; offset < 100 ms → green. Current state would render
green.

---

## 5. Tests of acceptance — final checklist

| Item | Status |
|---|---|
| Trauma operacional cerrado: `sudo systemctl stop agv-ros-stack.service` → dashboard sigue cargando, operador confirma desde browser | **AGENT-VERIFIED**, awaiting operator's browser confirmation |
| Dashboard accesible en http://JETSON-LAN-IP:8090/dashboard con ROS stack activo o inactivo | AGENT-VERIFIED (curl works both states) |
| Panel Health muestra "Overall: GREEN" cuando todo OK | OPERATOR-VERIFY (panel ships, awaits visual) |
| Los 5 probes previamente "?" ahora monitoreando activamente | OPERATOR-VERIFY |
| chrony instalado, configurado, monitoreado | AGENT-VERIFIED via `chronyc tracking` |
| Restart selectivo de ROS stack funciona sin afectar dashboard | AGENT-VERIFIED end-to-end |
| `bash tools/verify_specs/all.sh` PASS exit 0 | ✓ 12 scripts, 0 blocking, 0 warnings |
| Documentación de operations completa | ✓ `docs/operations/systemd_services.md` |

---

## 6. Lessons learned — the optimistic verdict trap

The earlier `CLOSED-VERIFIED-HW` on 1.1.b.full was wrong because:

1. **The agent's test was clean-room, not production-equivalent.**
   I ran `AGV_PORT=8091 node dist/index.js` directly and confirmed
   the in-process behaviour worked. That demonstrated the code
   change but NOT the operator's lived scenario, which is
   `systemctl stop agv.service` on the production unit.

2. **The agent conflated "code change is correct" with "operator's
   stated problem is closed".** They're not the same. The correct
   verdict at the time would have been
   `CLOSED-VERIFIED-CODE (in-process refactor) — operator scenario
   needs production-systemd-level test`.

3. **Multi-layer tests need multi-layer verdicts.** The trauma
   scenario has a layer-stack: process → systemd → network →
   browser. Each layer can fail independently. A pass at one
   layer is not a pass at the layer above.

Going forward (rule §0.1 reaffirmed):
- For any "close the trauma" scenario in future sub-phases:
  reproduce the EXACT failure mode the operator hits BEFORE
  marking closed. If the operator reports a browser-level
  failure, the verdict requires a browser-level pass.
- `CLOSED-VERIFIED-HW` requires the operator's empirical
  confirmation from their environment, not the agent's
  confirmation from a clean-room test.

---

## 7. Files touched in this follow-up

| Path | Action |
|---|---|
| `/etc/systemd/system/agv-dashboard.service` | created (deployed) |
| `src/agv_ui_backend/systemd/agv-dashboard.service` | SoT copy committed |
| `src/agv_bringup/launch/agv_full.launch.py` | removed `Node(teleop_backend)` block |
| `src/agv_ui_backend/src/app_deps.ts` | +4 last*Time fields |
| `src/agv_ui_backend/src/index.ts` | +4 subscribers in createAllSubscriptions |
| `src/agv_ui_backend/src/health_monitor.ts` | switch reads new state fields |
| `docs/agent/phase1_1_followup_log.md` | created |
| `docs/operations/systemd_services.md` | created |
| `docs/agent/phase1_1_final_report.md` | this file |

---

## 8. Stop and wait

Per prompt §4.4: the agent stops here. The operator must
execute Test 2 from their browser:

```
[ ] http://JETSON-LAN-IP:8090/dashboard loads
[ ] sudo systemctl stop agv.service
[ ] Refresh the dashboard tab — IT MUST KEEP LOADING.
```

Only the operator's confirmation upgrades the trauma-fix verdict
to `CLOSED-VERIFIED-HW`. Without it, the verdict stays
`CLOSED-VERIFIED-CODE (agent-side)`.

The agent will not advance to Sub-fase 1.2 without explicit
operator OK.
