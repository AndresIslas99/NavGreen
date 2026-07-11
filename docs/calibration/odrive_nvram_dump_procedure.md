# ODrive S1 NVRAM Dump — Procedure

**Purpose**: identify the firmware-side configuration error that
explains the **1.25× factor** between the geometric wheel radius
(0.0625 m, measured by caliper 2026-05-13) and the runtime-effective
wheel radius (0.0781 m, currently in
`src/agv_description/config/robot_geometry.yaml`).

**Status of the bug**: see
[`docs/audit/2026-05-13-greenhouse-hardening/SUMMARY.md` §CRITICAL-02-02](../audit/2026-05-13-greenhouse-hardening/SUMMARY.md).
The 1.25 = 5/4 ratio is too clean to be UMBmark drift (typical 1–5 %).
Leading hypotheses: `encoder.config.cpr` or `motor.config.pole_pairs`.

**Time budget**: 30 minutes from start to a complete dump file. Do not
modify the field-deployed values until step 5 of the closure plan.

## Prerequisites

- Robot powered, ODrives reachable via USB or CAN.
- Python environment with `odrive` package: `pip install odrive` (or use
  the team's existing `tools/calib_runs/` virtual env if present).
- (Optional) `odrivetool` CLI on `PATH`.
- This document open in a terminal.
- A clean text file to capture the dump:
  `docs/calibration/odrive_nvram_dump_$(date +%Y-%m-%d).txt`.

## Step 0 — Confirm hardware contact

```bash
odrivetool --help
ls /dev/serial/by-id/ | grep -i odrive
```

If neither produces output, fall back to CAN:

```bash
ip -details link show can0 | grep -E 'state|bitrate'
candump -tz can0 &
CANDUMP_PID=$!
# expect heartbeat frames (arbitration ID = (node_id << 5) | 0x001)
sleep 2
kill $CANDUMP_PID
```

## Step 1 — Dump per-axis configuration

For **each axis** (`odrv0.axis0` left, `odrv0.axis1` right) capture:

```python
# odrivetool
print("axis", a, "motor.config.pole_pairs       :", odrv0.axis<a>.motor.config.pole_pairs)
print("axis", a, "motor.config.torque_constant  :", odrv0.axis<a>.motor.config.torque_constant)
print("axis", a, "motor.config.current_lim      :", odrv0.axis<a>.motor.config.current_lim)
print("axis", a, "motor.config.gear_ratio       :", odrv0.axis<a>.motor.config.gear_ratio)  # if firmware supports
print("axis", a, "encoder.config.cpr            :", odrv0.axis<a>.encoder.config.cpr)
print("axis", a, "encoder.config.mode           :", odrv0.axis<a>.encoder.config.mode)
print("axis", a, "encoder.config.use_index      :", odrv0.axis<a>.encoder.config.use_index)
print("axis", a, "controller.config.vel_gain    :", odrv0.axis<a>.controller.config.vel_gain)
print("axis", a, "controller.config.vel_integrator_gain :", odrv0.axis<a>.controller.config.vel_integrator_gain)
print("axis", a, "controller.config.input_mode  :", odrv0.axis<a>.controller.config.input_mode)
print("axis", a, "controller.config.control_mode:", odrv0.axis<a>.controller.config.control_mode)
```

Capture the **full output** of each line (do not summarise) into the
dump file. Include the date, the operator's name, and the robot
serial/identifier at the top of the file.

## Step 2 — Cross-reference against the M8325s datasheet

The M8325s motor specs the team is using state (verify against the
physical motor label):

- Pole pairs: **N** (TBD — populate from datasheet on first dump)
- Encoder CPR: **M** (TBD — typically 4× the encoder line count for
  quadrature)
- Gear ratio: **10:1** (planetary gearbox; check the gearbox stamp)

Compare each captured value against the datasheet. A 25 % mismatch in
`encoder.config.cpr` (e.g., firmware has 8000 but spec is 10000)
explains the 1.25× factor directly. A `motor.config.pole_pairs`
mismatch (5 stored vs 4 actual, or vice versa) also produces a clean
integer ratio.

## Step 3 — Confirm the hypothesis (math check)

For each candidate parameter `P`, compute the expected runtime
effective radius:

```
effective_radius = geometric_radius × (firmware_value / spec_value)
```

If `effective_radius ≈ 0.0781` for one of the candidates, that is the
root cause.

Example (encoder.cpr hypothesis):
```
geometric_radius = 0.0625
firmware_cpr     = 10000
spec_cpr         = 8192
effective_radius = 0.0625 × (10000 / 8192) = 0.0763  # 22 % over, close but not exact
```

Example (pole_pairs hypothesis):
```
firmware_pp      = 5
spec_pp          = 4
effective_radius = 0.0625 × (5 / 4)         = 0.0781  # exact match
```

The hypothesis that **exactly** produces 0.0781 wins. If none exactly
match, the explanation may be a combination — escalate to a second
diagnostic session.

## Step 4 — Correct the root cause in firmware

Once confirmed:

```python
# odrivetool
odrv0.axis<a>.motor.config.pole_pairs = <correct_value>   # e.g., 4
# OR
odrv0.axis<a>.encoder.config.cpr      = <correct_value>   # e.g., 8192
odrv0.save_configuration()
odrv0.reboot()
```

**Do this on a bench**, not in the greenhouse. After the fix the robot
will respond to velocity commands ~25 % slower than before — verify
manually before redeploying.

## Step 5 — Restore the geometric values in the workspace

Edit `src/agv_description/config/robot_geometry.yaml`:

```yaml
/**:
  ros__parameters:
    wheel_radius: 0.0625        # was 0.0781; restored after NVRAM fix per CRITICAL-02-02 step 5
    track_width:  0.735          # was 0.960; restored
    gear_ratio:   1.0            # if firmware now does the gearing; was 10.0
    # ... other keys unchanged
```

Then:

```bash
bash tools/verify_specs/all.sh
# verify_geometry_ssot should drop from WARNING to OK
```

Re-run UMBmark (`tools/calib_umbmark.py` per
`docs/calibration/umbmark_protocol.md`); residual error must now be in
the 1–5 % range.

## Step 6 — File the dump

Drop the captured dump file in `docs/calibration/` as
`odrive_nvram_dump_<YYYY-MM-DD>.txt`. Append a one-line entry to
`docs/calibration/history.md` referencing the dump and the firmware
edit (date, axis, parameter, before → after).

Update `docs/audit/2026-05-13-greenhouse-hardening/SUMMARY.md`
CRITICAL-02-02 row to mark "Closed".

## If the dump finds nothing

If every parameter matches the M8325s datasheet exactly and the 1.25×
factor remains unexplained:

- The fudge in `wheel_radius` is the actual mechanism (pre-2026-04-08
  someone empirically tuned the value until commanded distance matched
  measured distance).
- **DO NOT** change `wheel_radius` in this case. The fudge is the only
  thing keeping the robot accurate.
- File the result in `docs/calibration/history.md` and **escalate**:
  the next session needs an external odometry validator (track-side
  AprilTag at a known distance + scripted commanded-vs-measured
  comparison) to identify what physical phenomenon the 1.25× factor
  encodes.

## References

- `docs/audit/2026-05-13-greenhouse-hardening/SUMMARY.md` §CRITICAL-02-02
- `docs/calibration/umbmark_protocol.md`
- ODrive S1 manual, firmware 0.5.x reference for `motor.config` and
  `encoder.config` schema
- `src/agv_odrive/CLAUDE.md` "Improvement Opportunities" → "Validate
  gear_ratio against ODrive firmware config to prevent silent
  misconfiguration"
