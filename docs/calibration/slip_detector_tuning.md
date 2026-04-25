# Wheel Slip Detector — Tuning Guide

The `wheel_slip_detector_node` (in `agv_sensor_fusion`) decides whether
the wheel-encoder twist on `/agv/wheel_odom` can be trusted. When it
detects slip, it republishes the message on `/agv/wheel_odom_validated`
with the linear-x and angular-z covariances inflated to a sentinel
value (1e6 by default), which `robot_localization` treats as "ignore
this update." This is the modern alternative to inflating covariance
multiplicatively from `odrive_can_node`, which the doc of
`robot_localization` explicitly discourages.

References:

- M. Brossard et al., "RINS-W," IROS 2019. arXiv:1903.02210.
- F. De Giorgi et al., "Online Odometry Calibration for Differential
  Drive in Low Traction Conditions," MDPI Robotics 13(1):7, 2024.
- T. Moore, robot_localization documentation: "Inflating covariance is
  unnecessary and detrimental … Set the configuration for the variable
  you'd like to ignore to false."

## State machine

```
INACTIVE  ─[slip signal]→  ACTIVE_HOLD
ACTIVE_HOLD ─[clean ≥ min_active_s since first entry]→  SETTLING
SETTLING  ─[clean for settle_s]→  INACTIVE
SETTLING  ─[slip signal]→  ACTIVE_HOLD     (re-enter, reset timer)
```

While in `ACTIVE_HOLD` or `SETTLING`, the validated message has
`twist.covariance[0]` (vx) and `twist.covariance[35]` (wz) set to
`inflated_xx`. While in `INACTIVE`, the upstream covariance is forwarded
unchanged (with `forward_upstream_baseline=true`).

## Parameters

All overrideable via ROS params; defaults live in
`include/agv_sensor_fusion/wheel_slip_detector.hpp`.

| Param | Default | Purpose |
|-------|---------|---------|
| `yaw_rate_threshold_rad_s` | 0.15 | Slip is asserted when \|gyro_wz - wheel_wz\| exceeds this. Choose above the noise floor of the gyro (Butterworth-filtered ZED IMU is well below this). |
| `linear_velocity_threshold_m_s` | 0.05 | Slip is asserted when \|visual_vx - wheel_vx\| exceeds this. Higher than the cuVSLAM RMS noise but lower than typical caster-slip signatures. |
| `min_active_s` | 0.30 | Minimum hold time after first entering slip. Avoids chatter on threshold crossings. |
| `settle_s` | 0.50 | Decay window after slip clears. The caster wheels need time to physically realign. |
| `imu_max_age_s` | 0.10 | Reject IMU older than this. Default = 5× IMU period at 200 Hz. |
| `visual_max_age_s` | 0.20 | Reject visual older than this. Default = 2× cuVSLAM period at 10 Hz. |
| `require_visual` | false | If true, both yaw AND vx must violate to assert slip. Default false because cuVSLAM may be lost while caster slip is happening; the IMU yaw test alone is sufficient evidence. |
| `inflated_xx` | 1.0e6 | Sentinel covariance written during slip. |
| `baseline_xx` | 0.05 | Used only when `forward_upstream_baseline=false`. |
| `forward_upstream_baseline` | true | When INACTIVE, pass the upstream covariance through; otherwise overwrite with `baseline_xx`. |

## Tuning protocol

1. Run `tools/calib_diff_drive_baseline.py --record-bag --label "pre-tune"`
   for a 5-min session driving forward, reverse, and direction reversals.
   Note the median Δodom/Δtag ratio.

2. Bring the stack up with the slip detector node active. While it's
   running, monitor the state output:
   ```
   ros2 topic echo /agv/wheel_slip/state
   ```
   Drive a smooth straight-line forward (0.10 m/s, no reversals).
   Verify the state stays `INACTIVE` for the entire pass. If you see
   `ACTIVE_HOLD` during smooth motion, your `yaw_rate_threshold_rad_s`
   is too tight — raise it.

3. Drive a forward → immediate-reverse maneuver. Verify the state
   transitions through `ACTIVE_HOLD` → `SETTLING` → `INACTIVE` during
   the inversion. If it stays `INACTIVE` (didn't trigger), the
   threshold is too loose — lower it.

4. Repeat the baseline session with `--label "post-tune"`. Compare:
   - Median Δodom/Δtag should approach 1.000 (or at least improve by
     ≥30% relative to pre-tune).
   - Asymmetry forward/reverse should shrink.
   - Standard deviation across legs should drop.

5. If the post-tune baseline is no better than pre-tune, the
   detector is not catching the slip. Reasons:
   - Threshold mis-set (re-tune).
   - The slip is "smooth" — wheels and gyro both drift together
     because the caster pre-loads slowly. In that case, the detector
     cannot distinguish slip from real motion; the only fix is the
     visual check (set `require_visual=true` and re-tune
     `linear_velocity_threshold_m_s`).
   - The bias is in geometry (wheel_radius / track_width), not slip.
     Run UMBmark first (Phase 1).

## Frozen tunings

| Date | Surface | Payload | Thresholds (yaw, vx) | min_active / settle | Δodom/Δtag pre | post |
|------|---------|---------|----------------------|---------------------|----------------|------|
| _(no tunings yet — populate after first session)_ |

Add a row after every tuning session committed to `main`. Reference
the commit SHA.
