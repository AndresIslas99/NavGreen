# Hardware E-Stop Wiring — Option 1 (Minimum Viable)

> **Status**: HIGH-09-01 option 1 from
> `docs/audit/2026-05-13-greenhouse-hardening/09_safety.md`. This is
> the **non-certified operational safeguard** path: a single
> normally-closed pushbutton wired in series with the ODrive S1's
> `ENABLE` input. Pressing the button breaks the loop; the ODrive
> enters IDLE at the hardware level regardless of what the Jetson is
> doing. Costs ≈ USD 5 in parts.
>
> This document does **not** describe certified functional safety. Per
> `policies/engineering_rules.md` Rule 6, certified safety requires
> dual-channel monitored relays, safety PLCs, and hardware compliance
> work outside the current MVP scope. Option 1 is the field-deployment
> minimum that prevents "robot keeps moving when operator pushes the
> red button because the Jetson froze".

## Why hardware E-stop matters even with software safety

Phase 9 of the 2026-05-13 audit traced 5 layers of software safety
(costmap inflation → collision_monitor → cmd_vel_gate → vx_max cap →
backend watchdog). Each layer assumes the layer below is functioning.
All five collapse if **the Jetson itself hangs, freezes, or crashes**:

- The dashboard E-stop button publishes `/agv/e_stop` over WebSocket
  to the backend, then to `agv_odrive_node`. Every hop depends on a
  functioning ROS graph.
- A kernel panic, OOM kill, or thermal throttle freeze leaves the
  motors running on the last commanded velocity until ODrive's own
  `cmd_vel_timeout_ms` (200 ms) fires. At `vx_max = 0.25 m/s` that's
  5 cm of unintended travel.
- A 5 cm overshoot toward a plant cuna is recoverable. A 5 cm
  overshoot toward a worker's leg is not.

A hardware loop that does not depend on the Jetson closes that gap.

## Bill of materials

| Item | Qty | Notes |
|---|---|---|
| Normally-closed (NC) mushroom-head pushbutton, latching, twist-to-release | 1 | Red, 22 mm panel mount. Latching is critical — momentary buttons defeat the "stays stopped until acknowledged" property. Example: Schneider XALK178 family or generic equivalent. |
| Wire, 22 AWG, 2 conductors, ≈ 1 m | 1 | Long enough to reach from the chassis mount point to the ODrive S1's enable terminal. Avoid running parallel to motor power leads. |
| Heat-shrink tubing | as needed | |
| Cable gland or strain relief | 1 | At the chassis penetration. |

Total cost: USD 5–15 depending on button quality.

## ODrive S1 connector reference

The ODrive S1 exposes an `ENABLE` input on its terminal block. From the
official ODrive S1 datasheet (verify against the unit in hand before
wiring):

- The `ENABLE` pin is logic-level (3.3 V tolerant on most revs).
- When the pin is **pulled high** (closed loop to 3.3 V via internal
  pull-up), the ODrive accepts commands.
- When the pin is **floating or pulled low**, the ODrive enters
  `IDLE` state and stops driving the motors.

The NC button wires **between the ENABLE pin and 3.3 V (or whatever
the ODrive's internal pull-up tap is)** such that pressing the button
opens the circuit, the pin floats, and the ODrive disarms.

**Verify the pinout against the specific ODrive S1 revision deployed
on the robot.** Older revs have a slightly different enable scheme;
some firmware versions expose `ENABLE` only via the GPIO header. Test
on a bench before installing on the robot.

## Wiring diagram

```
   ┌──────────────┐      ┌──────────────────────────────────┐
   │   ODrive S1  │      │       NC Pushbutton (E-stop)     │
   │              │      │  ┌────────────────────────────┐  │
   │   ENABLE  ◄──┼──────┤  │   normally closed contacts │  │
   │              │      │  │ press → opens the circuit  │  │
   │   3.3 V   ───┼──────┤  └────────────────────────────┘  │
   │              │      │      twist-release latch         │
   │   GND     ─┐ │      │      RED MUSHROOM HEAD           │
   └────────────┴─┘      └──────────────────────────────────┘
                                       │
                                       ▼ press: motors disarm in <10 ms
                                       ▼ release: re-engages 3.3 V → motors REMAIN DISARMED
                                                   (must call /agv/motor_enable false→true
                                                    from dashboard to re-arm — software
                                                    sees the ENABLE go high and re-engages
                                                    closed-loop control)
```

Notes:

- Wire the button **in series** with the 3.3 V rail to the ENABLE pin.
- Twist-release latch ensures the operator must consciously release the
  button before re-arming — prevents accidental clearance from a
  brushing hand.
- Re-arm is a **software action** after the button is released. The
  hardware path only disarms; the operator must click "Arm Motors" on
  the dashboard to re-enter `CLOSED_LOOP_CONTROL`. This is the correct
  industrial pattern (separate engage and reset).

## Mounting

The mushroom-head button should be:

- **Visible** from any angle around the robot.
- **Reachable** at adult shoulder height — operators must be able to
  punch it without bending or fumbling.
- **Protected** from accidental press during transit / lifting (a 22 mm
  guard ring is sufficient; full safety-cage cover is over-engineered
  for option 1).
- **Outside the camera's FoV** so a press does not occlude perception.

Typical mounting: top of the chassis, centered laterally, between the
ZED camera and the rear edge.

## Testing procedure

After installation, before any field operation:

1. With the robot powered but motors **NOT** armed: press and release
   the button. The ODrive `ENABLE` pin should toggle (verify with a
   multimeter across the ENABLE terminal — should read ~3.3 V released,
   ~0 V pressed).
2. Arm motors via dashboard. Robot should respond to a small forward
   `cmd_vel`.
3. **While the robot is moving**, press the E-stop button.
4. Confirm:
   - The robot **stops within 100 ms** (measurable with a phone slow-mo
     camera if no oscilloscope is available; the stop should appear
     "instantaneous" to the eye).
   - `agv_odrive_node` reports `motor_state.left_state` and
     `right_state` transitioning to `IDLE` (state code 1).
   - Dashboard shows motors disarmed.
5. Release the latch. Motors should **remain disarmed**. Verify by
   sending `cmd_vel` from the dashboard — robot does not move.
6. Click "Arm Motors" on the dashboard. Motors re-engage. Verify
   `cmd_vel` again moves the robot.

Document the test in `docs/calibration/history.md` with the date,
operator, and pass/fail result. A failed test means the wiring is
incorrect — do **not** deploy the robot until step 4 produces a clean
stop.

## What this does NOT replace

Option 1 stops the **motors**. It does not:

- Cut power to the Jetson. If the Jetson is the failure source (kernel
  panic), it stays powered. This is fine because the motors are
  disarmed.
- Cut power to the LiPo / battery management. Battery contactors are
  out of scope for option 1.
- Provide dual-channel monitoring. A single wire failure (e.g., crimp
  comes loose) would silently disable the E-stop. Option 2 (a
  microcontroller-bridged variant) and Option 3 (dual-channel relay
  with monitoring) address this — both are deferred to post-MVP per
  the audit's `HIGH-09-01` finding.

## Status

| Item | Status |
|---|---|
| Audit finding | HIGH-09-01 option 1 |
| Implementation | Procedure documented; field installation pending |
| Test plan | §"Testing procedure" above |
| Acceptance | Hardware test passes step 4 (<100 ms stop) |
| Closes | HAZOP `H-01` (frontal collision), `H-02` (moving person collision) physical-failure modes |

This document is the deliverable for HIGH-09-01 option 1. The actual
button installation is a 30–60 minute hardware task for the next
field-prep session.
