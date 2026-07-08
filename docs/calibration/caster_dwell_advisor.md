# Caster Dwell Advisor — Phase 4 (advisory variant)

The `caster_dwell_advisor_node` watches `/agv/cmd_vel` for direction
reversals and publishes `/agv/caster/dwell_state` recommending that a
controller pause for `dwell_s` seconds before sending the reversed
command. The pause lets the passive caster wheels physically realign,
avoiding the bore-torque-induced lateral arc that biases wheel_odom
(measured at +20% on polished ceramic, 2026-04-25 baseline).

This is the **advisory variant** of Phase 4 in the diff-drive
calibration plan. It is a passive observer: it does NOT mutate
`/agv/cmd_vel`. Closing the loop requires either (a) a Nav2 MPPI
custom critic that consumes the advisory and adjusts the planned
trajectory, or (b) a middleware node that gates `/agv/cmd_vel` during
the dwell window before reaching `velocity_smoother`. Both are
post-deployment work.

Reference: Arrizabalaga et al., "A caster-wheel-aware MPC-based
motion planner for mobile robotics," arXiv:2110.05604, 2021. Their
formulation embeds the caster orientation as a state in MPC and
minimizes the bore torque on direction changes. This advisor is a
reduced version that does not require caster encoders (Phase 3,
which we deliberately skip).

## State machine

```
IDLE  ─[cmd_vx flips sign across deadband]→  DWELLING
DWELLING  ─[after dwell_s seconds]→  IDLE
```

`/agv/caster/dwell_state` payload:

```json
{
  "state": "IDLE" | "DWELLING",
  "last_sign": -1 | 0 | +1,
  "seconds_remaining": <float>
}
```

## Parameters

| Param | Default | Purpose |
|-------|---------|---------|
| `deadband_vx_m_s` | 0.02 | Linear velocity below which we don't count "direction." Prevents spurious flips from noise around zero. |
| `dwell_s` | 0.5 | Recommended pause duration. Inherits the value from the previous `caster_settling_tau` in `odrive_params.yaml`. |

## Verification once consumers exist

The advisor itself is observable today:

```bash
ros2 topic echo /agv/caster/dwell_state
```

Drive a forward-then-reverse maneuver from the dashboard joystick.
Expect to see `state` flip from `IDLE` to `DWELLING` for ~0.5 s after
each direction reversal.

## Future work

The pieces below are NOT implemented yet. They are the path to closing
the loop on Phase 4:

1. **velocity_smoother integration** (simplest): override Nav2's
   `velocity_smoother` configuration to subscribe to
   `/agv/caster/dwell_state` and zero its output during DWELLING. Tom
   Moore (`robot_localization` author) recommends against in-tree
   smoother modifications, so this would be a forked smoother in
   `agv_navigation`.
2. **Nav2 MPPI custom critic** (correct approach): add a `dwell_critic`
   plugin that penalizes trajectories whose first sample violates the
   dwell window. This is closer to the Arrizabalaga formulation.
3. **Caster encoders** (full Arrizabalaga): instrument the caster
   pivots with magnetic encoders and expand the slip detector +
   dwell advisor with the actual caster angle as observation.
   Decided out-of-scope for this calibration plan.
