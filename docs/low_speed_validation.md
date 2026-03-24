# Low-Speed Validation Checklist

Repeatable commissioning procedure for verifying smooth low-speed drivetrain behavior.
Run after any ODrive gain change or before a field visit.

---

## Prerequisites

- `can0` is up (`ip link show can0` → state UP)
- Both ODrive axes show `armed: true` in `drive_debug`
- `ros2 topic echo /drive_debug` is running in a separate terminal
- Tablet/phone connected to AGV over local WiFi

---

## Terminal setup before testing

```bash
# Terminal 1 — watch drive_debug (your main diagnostic view during tests)
ros2 topic echo /drive_debug

# Terminal 2 — watch raw encoder velocities
ros2 topic echo /joint_states

# Terminal 3 — (optional) record a bag for post-analysis
ros2 bag record /drive_debug /wheel_odom /cmd_vel
```

---

## Test 0 — Zero-command baseline

**Purpose**: verify release = true zero, no creep, no hum.

1. Enable motors from the UI
2. Move joystick to ~50% forward for 2 seconds
3. Release joystick completely
4. Observe:

| Check | Expected | Pass / Fail |
|---|---|---|
| `zero_cmd` in `drive_debug` | `true` immediately on release | |
| `left_meas` and `right_meas` | reach ~0.0 within 1 wheel rotation | |
| Audible motor hum after release | none | |
| Creep / drift observed visually | none | |

---

## Test 1 — Straight forward, incremental joystick

**Purpose**: verify smooth onset and sustained motion at each level.

For each joystick level: 5%, 10%, 15%, 20%, 30%:

1. Enable motors
2. Push joystick to level, hold for 3 seconds
3. Release
4. Wait 2 seconds for full stop

| Level | Smooth onset? | Sustained without chatter? | Symmetric (left≈right meas)? | Clean stop? |
|---|---|---|---|---|
| 5% | | | | |
| 10% | | | | |
| 15% | | | | |
| 20% | | | | |
| 30% | | | | |

**"Good" looks like**: motor starts within ~0.5s of command, no repeated start-stop oscillation,
`left_meas ≈ right_meas` within 10%, clean stop on release.

**Red flags**:
- Chatter at 10–15%: `vel_integrator_gain` too high → lower it
- No motion at 5–10%: mechanical stiction, investigate `vel_ramp_rate` or increase `vel_gain`
- One side persistently faster: use `left_scale`/`right_scale` in `odrive_params.yaml`

---

## Test 2 — Straight reverse, same levels

Same as Test 1 but joystick pulled back.

| Level | Smooth onset? | Sustained? | Symmetric? | Clean stop? |
|---|---|---|---|---|
| 5% | | | | |
| 10% | | | | |
| 15% | | | | |
| 20% | | | | |
| 30% | | | | |

---

## Test 3 — In-place rotation

**Purpose**: verify symmetric rotation and clean stop.

1. 20% left rotation, 3 seconds
2. Release, wait 2 seconds
3. 20% right rotation, 3 seconds
4. Release

| Check | Expected | Pass / Fail |
|---|---|---|
| Rotation appears symmetric (equal left/right) | yes | |
| No lateral drift during pure rotation | minimal | |
| Clean stop on release | yes | |
| `left_target` ≈ -`right_target` in `drive_debug` | yes (opposite signs, same magnitude) | |

---

## Test 4 — Slow arc

**Purpose**: verify blended linear + angular commands work smoothly.

1. Set joystick to ~20% forward + 15% left arc
2. Hold 4 seconds, observe circular path
3. Release

| Check | Expected | Pass / Fail |
|---|---|---|
| Robot traces an arc (not straight) | yes | |
| No oscillation during arc | yes | |
| Clean stop on release | yes | |

---

## Test 5 — E-stop during motion

**Purpose**: verify emergency stop is immediate and resets state cleanly.

1. Enable motors, drive at ~30% forward
2. Trigger E-stop from UI
3. Verify full stop
4. Release E-stop
5. Re-command motion

| Check | Expected | Pass / Fail |
|---|---|---|
| `e_stop: true` in `drive_debug` immediately | yes | |
| Motors stop within 0.2 s | yes | |
| No residual creep after E-stop | yes | |
| Normal motion resumes after E-stop release | yes | |

---

## Test 6 — Watchdog timeout (disconnect/deadman)

**Purpose**: verify motors stop if comms drop.

1. Enable motors, drive at ~20%
2. Disconnect WiFi on the tablet (or close browser tab)
3. Wait for `cmd_vel_timeout_ms` (default: 500 ms)
4. Observe

| Check | Expected | Pass / Fail |
|---|---|---|
| `cmd_valid: false` appears in `drive_debug` | yes | |
| Motors stop | yes, within ~0.5 s | |
| No continuous motion after disconnect | yes | |

---

## Pass criteria

A drivetrain passes low-speed validation when:

- All Test 0 checks pass (zero = true zero)
- Tests 1 and 2 pass at 10%, 15%, 20%, 30% (5% is advisory — stiction varies by surface)
- Test 3 passes (symmetric rotation)
- Test 4 passes (arc)
- Tests 5 and 6 pass (safety stops)

---

## What to record

For each commissioning session, record:
- Date and surface type (tile, concrete, soil)
- ODrive gain values (`check_odrive_config.py` output)
- Any failing test steps and what corrected them
- Final `odrive_params.yaml` values used

---

## Tuning reference

If tests fail, see `docs/odrive_low_speed_tuning.md` for the diagnosis-to-fix mapping.
