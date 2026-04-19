# ODrive S1 — Low-Speed Tuning Procedure

This document covers how to diagnose and fix low-speed motor control issues on the AGV.
Applies to: ODrive S1, firmware 0.6.11, input_mode = VEL_RAMP (2).

---

## Principle: tune the drive first, not the upper layer

The ODrive controls velocity on the motor side. Upper-layer software (the ROS node) should
provide clean, correctly-timed velocity commands and nothing more. Do not add artificial
kicks, snapping, or feedforward to compensate for bad drive tuning — it hides the real
problem and causes secondary oscillation.

Fix order:
1. Tune ODrive gains (this document)
2. Verify clean zero-command behavior in ROS node (see `drive_debug` topic)
3. Only then adjust `max_wheel_accel` for teleop feel
4. Only as a last resort, add `min_effective_vel` / `stiction_torque_ff` — with care

---

## Parameters and their effects

### `vel_gain` (proportional gain)

- Controls how aggressively velocity error is corrected
- **Too high**: oscillation, buzzing at standstill
- **Too low**: sluggish response, motor follows commands slowly
- Typical range for M8325s: 0.10 – 0.25
- **Current target**: 0.167

### `vel_integrator_gain` (integrator gain)

- Eliminates steady-state velocity error
- **Too high** (most common cause of stick-slip): motor overshoots, integrator winds up,
  velocity oscillates around target, especially visible at low speed (5–20% joystick)
- **Too low**: motor may not reach target speed under load
- Rule of thumb: `vel_integrator_gain ≈ 0.5 × vel_gain × bandwidth`
- **Current target**: 0.167 (reduced from 0.333 — the integrator was the primary stick-slip cause)

### `input_filter_bandwidth` (Hz)

- Smooths the velocity setpoint as received by the controller
- Only active when `input_mode = INPUT_MODE_PASSTHROUGH` or when the ramp is bypassed
- With `VEL_RAMP`, the ramp already smooths the setpoint — this filter adds extra smoothing
- **Too low**: sluggish step response at normal speed
- **Too high**: no effect (passes everything)
- **Current target**: 8.0 Hz

### `vel_ramp_rate` (turns/s²)

- Rate at which the ODrive's internal velocity setpoint ramps toward the commanded value
- Only active when `input_mode = VEL_RAMP (2)`
- **Too high**: step response is still sharp (ramp too fast to matter)
- **Too low**: motor responds very slowly to any command change
- **Current target**: 0.5 turns/s²

### `vel_limit` (turns/s)

- Hard cap on commanded velocity
- Set comfortably above max operating speed
- **Current target**: 10.0 turns/s

---

## Symptom → diagnosis

| Symptom | Most likely cause |
|---|---|
| Chatter / oscillation at 10–20% joystick | `vel_integrator_gain` too high |
| Motor buzzes without moving at low command | `vel_gain` too low, or `vel_ramp_rate` too slow |
| Motor jerks forward on first command, then settles | Integrator windup from previous static friction |
| Both wheels spin but stop-start rhythmically | Integrator gain × encoder noise causing limit cycling |
| One side faster than the other | Mechanical mismatch → use `left_scale`/`right_scale` in `odrive_params.yaml` |
| Release not stopping completely | Check `zero_vel_epsilon` in `odrive_params.yaml`; check ROS node `drive_debug` topic |
| Standstill noise on encoder | Read `axis.encoder.vel_estimate` at standstill; if noisy, may need encoder config |

---

## Tuning order

1. **Start with both integrators off**: set `vel_integrator_gain = 0.0` temporarily.
   Verify motor responds proportionally and smoothly. If it doesn't track at all, raise `vel_gain`.

2. **Add integrator slowly**: increase `vel_integrator_gain` from 0.0 in steps of 0.05.
   Stop when the motor just eliminates steady-state error without overshooting.
   For these motors, 0.10 – 0.20 is the usual range.

3. **Verify low-speed behavior**: command 5% joystick. Motor should start smoothly and sustain.
   No chatter, no rhythmic oscillation.

4. **Check zero-command**: release joystick. Motor must stop within 1 full wheel rotation.
   No humming, no creep.

5. **Save to ODrive flash**:
   ```python
   odrv.save_configuration()
   ```
   Or use the config checker script with `--apply`.

---

## Using the `drive_debug` topic for live diagnosis

With the robot running:

```bash
ros2 topic echo /drive_debug
```

Output (JSON):
```json
{
  "cmd_linear": 0.15,
  "cmd_angular": 0.0,
  "left_target": 0.38,
  "right_target": -0.38,
  "left_meas": 0.35,
  "right_meas": -0.33,
  "armed": true,
  "e_stop": false,
  "cmd_valid": true,
  "zero_cmd": false
}
```

**What to look at during low-speed testing:**

- `left_target` vs `left_meas`: the gap is tracking error. Large, persistent gap at low speed
  means the integrator isn't doing enough (raise `vel_integrator_gain`).
  Oscillating gap means too much integrator.
- `zero_cmd: true`: confirms the zero bypass fired — motors should be fully stopped.
- `cmd_valid: false`: watchdog timed out — motors should be stopping automatically.

Check rate:
```bash
ros2 topic hz /drive_debug   # should read ~10 Hz
```

---

## Quick-check script

```bash
# Read current config (USB connected):
python3 src/agv_odrive/scripts/check_odrive_config.py

# Apply recommended values and save:
python3 src/agv_odrive/scripts/check_odrive_config.py --apply
```

---

## Notes

- `motor_thermistor.config.enabled = False` is required on this hardware — the NTC thermistor
  reports NaN temperature when unconnected, which triggers error 0x10 and prevents closed-loop.
- Both axes must have identical gain values. Asymmetric gains cause unequal response during
  direction changes and arcs.
- After `save_configuration()`, the ODrive reboots (~3 s). Wait for reboot before restarting
  the ROS node.
