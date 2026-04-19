# Hardware Setup — Jetson AGX Orin + ODrive S1 + Waveshare CAN

This document captures the real setup steps discovered during development.
The `can0 available` prerequisite in TASK.yaml files is NOT trivial — this explains why.

## Platform

- **Board**: NVIDIA Jetson AGX Orin 64GB Developer Kit
- **L4T**: R36.5 (JetPack 6.2)
- **CAN controller**: mttcan (c310000.mttcan for can0, c320000.mttcan for can1)
- **CAN transceiver**: Waveshare SN65HVD230 CAN Board
- **Motor controller**: ODrive S1, firmware 0.6.11

## CAN Pinmux — The Core Problem

### What does NOT work on L4T 36.5

Jetson-IO (`/opt/nvidia/jetson-io/config-by-function.py can0 -o dt`) generates an overlay and adds it to `extlinux.conf`, but **the overlay does not apply on boot**. Pins 29 and 31 remain as `unused` (GPIO mode) after reboot.

This is a known issue on L4T 36.5. Do not rely on Jetson-IO for CAN pinmux.

### What DOES work: busybox devmem

The CAN0 pinmux must be set at runtime using direct register writes:

```bash
# CAN0_DOUT (pin 31, TX) — set to CAN function
sudo busybox devmem 0x0c303010 w 0xc400

# CAN0_DIN (pin 29, RX) — set to CAN function
sudo busybox devmem 0x0c303018 w 0xc458
```

**Verification**: after writing, read back:
- `busybox devmem 0x0c303010` should return `0x0000C400`
- `busybox devmem 0x0c303018` should return `0x0000C458`

If you see `0xC059`/`0xC055` or `0xC058`/`0xC054`, the pins are in GPIO mode and CAN will not work.

### CAN1 pinmux (if needed)

```bash
sudo busybox devmem 0x0c303000 w 0xc400   # CAN1_DOUT (pin 33)
sudo busybox devmem 0x0c303008 w 0xc458   # CAN1_DIN  (pin 37)
```

## Wiring — Jetson 40-pin Header to Waveshare SN65HVD230

| Jetson pin | Signal    | Waveshare pin |
|------------|-----------|---------------|
| Pin 29     | CAN0_RX   | CAN_RX (R)    |
| Pin 31     | CAN0_TX   | CAN_TX (D/T)  |
| Pin 17     | 3.3V      | VCC (3.3V)    |
| Pin 39     | GND       | GND           |

**Critical**: GND must be connected between Jetson header and Waveshare, even if they share power ground through other paths.

### Waveshare to ODrive S1

| Waveshare | ODrive S1 |
|-----------|-----------|
| CAN_H     | CAN_H     |
| CAN_L     | CAN_L     |

## CAN Bus Termination

CAN bus requires 120 ohm termination at each end.

- **ODrive S1**: has internal 120 ohm termination, must be enabled via `odrivetool` or USB config
- **Waveshare end**: if no other termination, add a 120 ohm resistor between CAN_H and CAN_L

With both terminations active, measuring between CAN_H and CAN_L with bus idle should show ~60 ohm.

### Bus idle voltage levels

In a healthy idle bus:
- CAN_H ≈ 2.5V, CAN_L ≈ 2.5V, differential ≈ 0V

If you see ~2V differential at idle, the bus is stuck in dominant state. Most common causes:
1. TX pin driving dominant (pinmux not set to CAN)
2. CAN_L shorted to GND
3. H and L swapped at one end

## Kernel Modules

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

These are usually loaded automatically but verify with `lsmod | grep can`.

## Bringing Up CAN0

```bash
sudo ip link set can0 up type can bitrate 250000 restart-ms 100
```

- **bitrate 250000**: ODrive S1 default, must match `odrv0.can.config.baud_rate`
- **restart-ms 100**: auto-restarts CAN controller after bus-off errors. Without this, the interface goes DOWN permanently after a bus-off event and requires manual re-up.

## Persistent Boot Setup — systemd Service

File: `/etc/systemd/system/can-setup.service`

```ini
[Unit]
Description=Setup CAN0 pinmux and interface for ODrive
After=network-pre.target
Before=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/bin/busybox devmem 0x0c303010 w 0xc400
ExecStartPre=/bin/busybox devmem 0x0c303018 w 0xc458
ExecStart=/sbin/ip link set can0 up type can bitrate 250000 restart-ms 100

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable can-setup.service
```

## ODrive S1 Configuration

Verified via USB (`odrivetool` / Python `odrive` library):

| Parameter | Value |
|-----------|-------|
| `can.config.baud_rate` | 250000 |
| `axis0.config.can.node_id` | 0 |
| `axis0.config.can.heartbeat_msg_rate_ms` | 100 |
| Firmware | 0.6.11 |

### Velocity controller gains (per axis, low-speed tuning)

Applied 2026-03-18/19. Both axes must be identical.

| Parameter | Value | Notes |
|-----------|-------|-------|
| `controller.config.vel_gain` | 0.167 | Proportional gain |
| `controller.config.vel_integrator_gain` | 0.167 | Reduced from 0.333 — high integrator was causing stick-slip/chatter at low speed |
| `controller.config.input_filter_bandwidth` | 8.0 Hz | Reduced from 20.0 Hz — smooths setpoint on controller side |
| `controller.config.vel_ramp_rate` | 0.5 turns/s² | Reduced from 1.5 — gentler acceleration onset |
| `controller.config.vel_limit` | 10.0 turns/s | Hard velocity cap |
| `controller.config.input_mode` | 2 (VEL_RAMP) | Uses internal ramp, not step |
| `motor.motor_thermistor.config.enabled` | False | NTC thermistor reads NaN → triggers error 0x10 |

To verify or reapply:
```bash
python3 src/agv_odrive/scripts/check_odrive_config.py
python3 src/agv_odrive/scripts/check_odrive_config.py --apply
```

For tuning guidance: see `docs/odrive_low_speed_tuning.md`

### Motor thermistor

The motor thermistor reports `NaN` temperature when connected but not properly reading. This triggers active errors and prevents closed-loop control. To disable:

```python
import odrive
odrv0 = odrive.find_any()
odrv0.axis0.motor.motor_thermistor.config.enabled = False
odrv0.save_configuration()  # ODrive reboots
```

### USB descriptor confusion

`lsusb` reports `ODrive v3` for the S1 — this is a misleading USB string. Confirm S1 by:
- Firmware version 0.6.x (v3 uses 0.5.x)
- API has `active_errors` / `disarm_reason` (v3 uses `axis0.error` bitmask)

## Verification

```bash
# 1. Check pinmux
busybox devmem 0x0c303010   # expect 0x0000C400
busybox devmem 0x0c303018   # expect 0x0000C458

# 2. Check interface
ip link show can0            # expect state UP

# 3. Check heartbeats
timeout 3 candump can0       # expect messages with ID 0x001 every 100ms

# 4. Check statistics
ip -statistics link show can0 # RX packets should be increasing
```

## Robot Physical Dimensions

Measured 2026-03-18 on production chassis. Canonical source: `src/agv_description/config/robot_params.yaml`

```
   TOP VIEW                          SIDE VIEW

        +X (forward)                 base_link (0,0,0)
         ^                               |
         |                         ------+------ top plate
  LW ----+---- RW                        |  55mm
   367.5mm 367.5mm                      ZED
         |                               |
    ZED (700mm fwd)                      |
                                  ------+------ axle (z=-137.5mm)
                                     [wheel]
                                      62.5mm radius
                                  ------+------ ground (z=-200mm)
```

| Component | Position (x, y, z) mm | Notes |
|-----------|----------------------|-------|
| base_link | 0, 0, 0 | 200mm above ground |
| left wheel | 0, +367.5, -137.5 | radius 62.5mm |
| right wheel | 0, -367.5, -137.5 | radius 62.5mm |
| ZED 2i | +700, 0, -55 | no tilt (rpy 0,0,0) |

| Derived | Value |
|---------|-------|
| track_width | 0.735 m |
| wheel_radius | 0.0625 m |
| wheel_diameter | 0.125 m |

## Sources

- [NVIDIA CAN Documentation (r36.2)](https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/HR/ControllerAreaNetworkCan.html)
- [Forum: Jetson Orin AGX CAN BUS problem](https://forums.developer.nvidia.com/t/jetson-orin-agx-can-bus-problem/282377)
- [Forum: Jetson Linux v36.5 CAN can't work](https://forums.developer.nvidia.com/t/jetson-linux-v36-5-can-bus-cant-work/363561)
