# 2026-05-13 — Greenhouse Field-Readiness Hardening Audit

Senior Robotics Architect / Field Reliability review of the AGV greenhouse
workspace, **complementary** to the 2026-04-13 governance audit (which
established the spec SSOT). This cycle is focused on **field robustness**:
turning a stack that compiles and passes spec-verifiers into one that
survives 92 % humidity, repetitive crop rows, intermittent WiFi, and
non-ROS field technicians.

## Relation to prior audits

| Audit | Focus | SSOT |
|---|---|---|
| [2026-04-13-full-audit.md](../2026-04-13-full-audit.md) | Governance, spec drift, latent bugs, dead code | `specs/*.yaml` |
| **2026-05-13 (this)** | Field robustness, benchmark vs SOTA, calibration discipline, measurable gates | This folder |

The 2026-04-13 audit found 6 latent bugs and 9 rule violations. Five bugs are
documented as fixed (`current_state: FIXED`) in the specs; one (`hardcoded
.current.area` path in 4 files) is whitelisted pending refactor. The
**verifiers pass** at this commit (9 scripts run, 0 blocking failures,
1 warning about a vendored `zed-ros2-wrapper` not present in this clone).

This audit does **not** reopen those findings unless new evidence emerges.
It surfaces a **different class of risk**: things the existing verifiers do
not — and largely cannot — catch from a pure spec/code-text analysis.

## Scope and limits of this audit cycle

The reference prompt defines 15 phases with acceptance gates. **Many gates
require physical hardware, multi-day endurance runs, oscilloscope-level
measurements, or a real greenhouse — none of which are available to a desk
auditor.** This audit honestly partitions the work:

### Deliverable in this cycle (desk / code analysis)
- Phase 0 — Inventory & cartography (`00_inventory.md`)
- Phase 1 — Foundations (build, time, TF, deps): **static analysis only**
- Phase 2 — HAL (ODrive, ZED, URDF↔code parity): **static analysis only**
- Phase 3 — Calibration: **procedure review, NOT execution**
- Phase 4–10 — Localization, mapping, perception, planning, behaviors, safety, comms: **architectural review + gap analysis**
- Phase 11–14 — GUI, sim-to-real, CI/CD, docs: **review only**

### Deferred to field/hardware (not in this cycle)
- Build reproducibility timings (need an Orin)
- TF jitter measurements (need a running stack)
- Sensor sync latency, depth FPS sustained 30 min (need hardware)
- Calibration RMSE values (need a real ZED, AprilTag board, UMBmark course)
- Closed-loop drift in 100 m runs (need a greenhouse or Isaac Sim run)
- Oscilloscope E-stop latency (need an o-scope on the ODrive enable line)
- 8 h × 5 day endurance MTBF (Phase 15)

For every deferred metric, this audit defines:
1. **The acceptance criterion** (measurable threshold).
2. **The minimal experimental harness** to obtain it.
3. **The risk class** if the experiment never runs.

That is the most honest deliverable a desk audit can produce. Anyone running
the experiments adds the data to `tests/field/` and references the result
back to the finding ID in this folder.

## Deliverable structure

```
docs/audit/2026-05-13-greenhouse-hardening/
  README.md             ← scope, methodology (this file)
  00_inventory.md       ← Phase 0: workspace cartography + critical findings
  01_foundations.md     ← Phase 1: build / TF / time / deps
  02_hal.md             ← Phase 2: ODrive + ZED + URDF↔code parity
  03_calibration.md     ← Phase 3: procedures, manifests, online consistency
  04_localization.md    ← Phase 4: dual EKF, AprilTag fusion, loop closure
  05_mapping.md         ← Phase 5: persistence, sidecars, multi-zone
  06_perception.md      ← Phase 6: depth→costmap, traversability, dynamics
  07_planning.md        ← Phase 7: Nav2 costmaps, MPPI, BTs, smoother
  08_greenhouse.md      ← Phase 8: row following, headland, coverage
  09_safety.md          ← Phase 9: E-stop, geofencing, HAZOP/FMEA
  10_comms.md           ← Phase 10: DDS, QoS, diagnostics, observability
  11_gui.md             ← Phase 11: dashboard / commissioning workflows
  12_sim_to_real.md     ← Phase 12: parity, regression suites
  13_quality_cicd.md    ← Phase 13: CI/CD, sanitizers, coverage
  14_docs.md            ← Phase 14: docs, ADRs, onboarding
  SUMMARY.md            ← executive overview + roadmap + priority queue
```

Phases are filed lazily as work proceeds. Empty files are not created
pre-emptively — the absence of a file means that phase has not been written
yet, and the SUMMARY tracks status.

## Finding format

Each finding follows the template:

```markdown
### [SEVERITY] – [PHASE-NN-MM] – Short title
**File(s)**: path:line
**Category**: architecture | bug | performance | safety | quality | docs | calibration | sim-real-gap
**Symptom**: what is observed.
**Analysis**: why it happens, with code references.
**Greenhouse impact**: how it manifests in real operation.
**Benchmark**: how [reference framework] solves it (with citation).
**Recommendation**: concrete change. 2–3 options with trade-offs if non-trivial.
**Acceptance criterion**: how to verify the fix is correct (measurable).
**Effort**: S (< 1 d) | M (1–3 d) | L (1–2 wk) | XL (> 2 wk).
**Prerequisites**: finding IDs / phases that must close first.
```

Severities: `CRITICAL`, `HIGH`, `MEDIUM`, `LOW`, `NIT`.

Finding IDs are `PHASE-NN-MM` where `NN` is the phase number and `MM` is a
running counter within that phase, zero-padded.

## Methodology

1. **No invention**. Every claim cites `file:line` or a YAML key.
2. **Distinguish verified vs assumed**. If a measurement is not available
   without hardware, the finding says so and proposes the experiment.
3. **Resist competence theater**. If the SOTA migration is wrong for this
   robot, recommend staying put with explicit rationale.
4. **Lean on existing specs**. Where the 2026-04-13 audit already covered a
   topic, link to it and do not duplicate. New findings go in this folder.
5. **Compare to the operational reality.** Each phase considers humidity,
   lighting variability, perceptual aliasing, wet floors, vegetation
   intrusion, and intermittent WiFi as default conditions, not edge cases.

## Reference frameworks consulted

Where this audit cites benchmarks, they are drawn from:

- **Nav2 core** — Open Navigation / Steve Macenski
- **SLAM Toolbox** — Steve Macenski
- **robot_localization** — Tom Moore
- **Isaac ROS** — NVIDIA (cuVSLAM, NITROS, Nvblox)
- **FAST-LIO2 / Point-LIO** — HKU MaRS (LiDAR-inertial reference, even though this robot has no LiDAR today)
- **VINS-Fusion / OpenVINS** — HKUST / UDel (VIO references)
- **ETH RSL / ANYbotics** — elevation_mapping_cupy (traversability)
- **TIER IV Autoware** — DDS / lifecycle reference architecture
- **Fields2Cover / opennav_coverage** — UPV-EHU + Wageningen (coverage planning)
- **TUM, BUAA, ZJU FAST Lab** — trajectory optimization references
- **GTSAM / iSAM2 / Ceres** — factor graph backends (Frank Dellaert, Sameer Agarwal)
- **Tracy / LTTng / ros2_tracing** — profiling and tracing

Specific commits/docs are cited inline in each finding.

## How to read this

1. Start with `SUMMARY.md` for the executive overview and priority queue.
2. Dive into the phase document for any finding that interests you.
3. Findings cross-reference each other by ID (e.g., `PHASE-02-03`).
4. Acceptance criteria are the contract for "this finding is closed".

If you are an AI agent picking up this work, also read
[../../AGENT_INSTRUCTIONS.md](../../../AGENT_INSTRUCTIONS.md) and the SSOT
in [`specs/`](../../../specs/) before proposing any code change. The
findings in this folder describe **problems and recommendations**, not
authorized changes — a human (or a dedicated remediation PR) translates
them into commits that pass the spec verifiers.
