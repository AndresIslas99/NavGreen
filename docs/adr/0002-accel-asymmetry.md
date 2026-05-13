# ADR-0002 — Velocity-smoother and HAL accel limits are deliberately asymmetric

**Status:** Accepted — 2026-05-13
**Audit reference:** HIGH-07-02 (2026-05-13 hardening audit)
**Section-0 verification:** F5
**Owner:** drive/nav

## Context

Two layers limit how fast the robot can accelerate or decelerate:

1. **Nav2 velocity_smoother** (`src/agv_navigation/config/velocity_smoother.yaml`).
   Operates in linear m/s² and angular rad/s² between `controller_server`
   and `collision_monitor`. Current limits: `max_accel=[0.5, 0.0, 0.8]`,
   `max_decel=[-1.0, 0.0, -1.0]`.

2. **ODrive HAL** (`src/agv_odrive/config/odrive_params.yaml`).
   Operates in wheel turns/s² in `odrive_can_node`. Current limits:
   `max_wheel_accel=0.5 turns/s²`, `max_wheel_decel=1.5 turns/s²`.

With the current SSOT `wheel_radius=0.0781 m` (CRITICAL-02-02 scaffold),
one wheel revolution covers `2π·0.0781 ≈ 0.491 m`, so:

| | Smoother (m/s²) | HAL (turns/s²) | HAL expressed in m/s² |
|---|---|---|---|
| Accel | 0.5 | 0.5 | ≈ 0.245 |
| Decel | 1.0 | 1.5 | ≈ 0.736 |

**The HAL is roughly 2× tighter than the smoother on both legs.** A bench
step from 0 → 0.25 m/s saturates the HAL, never the smoother. HIGH-07-02
flagged this as a "mismatch" needing alignment.

## Decision

**Keep the asymmetry. Document it.** This is defense in depth, not drift:

- The smoother lives in the planning loop and exists to *shape* commands
  the controller emits. Its limits should reflect the *ideal* dynamics
  the planner is allowed to assume.
- The HAL lives next to the motors and exists to *enforce* what hardware
  can physically deliver without slipping the caster wheels (verified
  empirically during iter-46 caster-tuning sprint). It must be a strict
  subset of whatever the smoother lets through.

If the smoother were tighter than the HAL, a controller assuming the
smoother's envelope could legitimately request something the smoother
emits, only for the HAL to clip it. The asymmetry is corrected by
ensuring `HAL ≤ smoother` always, and that the smoother is loose enough
to never become the binding constraint at the HAL's expense.

The greenhouse floor (sealed concrete with intermittent wet patches) +
caster-wheel diff-drive geometry imposes a practical accel ceiling near
the HAL's 0.245 m/s²; pushing the smoother lower would just publish the
same envelope at two layers.

## Consequences

- **Step tests** must measure `dv/dt` on `/agv/wheel_odom` and confirm it
  respects the HAL ceiling, NOT the smoother ceiling. Section-0 F5 L3
  records this baseline.
- **Future tuning**: if caster behavior or wheel grip changes (rubber
  upgrade, different greenhouse, new motor firmware post-NVRAM-fix in
  CRITICAL-02-02 step 5), the values to revisit are the HAL ones first.
  The smoother stays unless we hit a planner pathology.
- **CI**: a future verifier could parse both yaml files, convert units
  through `wheel_radius`, and fail if `HAL > smoother` on any axis. Not
  worth writing now (manual review during config edits suffices), but
  the asymmetry direction is the invariant.
- **HIGH-07-02 is closed by this ADR**, not by aligning the numbers.
  The audit finding misread defense-in-depth as drift.

## Cross-references

- `src/agv_navigation/config/velocity_smoother.yaml:13-16` — smoother limits
- `src/agv_odrive/config/odrive_params.yaml:29-30` — HAL limits
- `src/agv_description/config/robot_geometry.yaml:38` — wheel_radius used for unit conversion
- `docs/audit/2026-05-13-greenhouse-hardening/SUMMARY.md` — finding HIGH-07-02
- `docs/audit/2026-05-13-pre-phase1-verification.md` — Section-0 F5 verification record
