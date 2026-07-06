# Security Policy

## Reporting a vulnerability

Please report security issues privately — do **not** open a public issue.

- Preferred: [GitHub private vulnerability reporting](https://github.com/AndresIslas99/agv-greenhouse/security/advisories/new)
- Alternative: email andresislas2107@gmail.com with `[SECURITY]` in the subject

You should receive an acknowledgement within a week. Please include steps to
reproduce and the component affected (package name or file path).

## Deployment security model

This stack drives a physical robot and is designed to run on an **isolated
greenhouse LAN**, not on the public internet. Know the boundaries before you
deploy:

| Surface | Posture |
|---------|---------|
| Operator dashboard / REST / WebSocket (`agv_ui_backend`, port 8090) | JWT auth with `viewer` / `operator` / `engineer` roles is available but **DISABLED by default**, and no default accounts ship. While disabled, **any client that can reach the port gets full operator control** (teleop, nav goals, e-stop, motor enable); the backend prints a loud `[SECURITY]` warning at startup. Enable before any field deployment: `npm run adduser -- <user> <pass> <role>`, then set `"enabled": true` in `$AGV_DATA_DIR/users.json`. Stop-type endpoints (`/api/nav/cancel`, `/api/recovery/trigger_estop`, `/api/missions/pause`) intentionally stay unauthenticated so the robot can always be stopped. |
| Fleet manager REST / WebSocket (`fleet/agv_fleet_manager`, port 8092) | Unauthenticated by default (logs a `[SECURITY]` warning). Set `FLEET_API_TOKEN` to require `Authorization: Bearer <token>` on `/api/*` and `?token=` on the WebSocket; bind with `FLEET_BIND_ADDR`. |
| MQTT broker (`fleet/mosquitto`, ports 1883/9001) | Ships with `allow_anonymous true`; the per-robot topic ACL in `fleet/mosquitto/config/aclfile` is **inactive** until authentication is enabled. The enable procedure (`mosquitto_passwd` + swapping the commented directives) is documented in `fleet/mosquitto/config/mosquitto.conf`. |
| Camera streaming (`agv_image_server`, port 8091) | Plain HTTP MJPEG, no authentication |
| ROS 2 DDS traffic | Unencrypted by default (no SROS2 configuration in this repo) |

**Never expose any of these ports beyond the robot's isolated network.** If
your deployment requires remote access, put the network behind a VPN, enable
dashboard authentication, and enable broker authentication before doing
anything else.

## Safety boundary

Operational safeguards in this repository (collision monitor configuration,
mode arbitration, software E-stop paths) are **not certified functional
safety**. Certified human safety requires external hardware-integrated scope —
safety-rated scanners, PLC/relay logic, and a compliance process. Do not
describe or rely on the software in this repository as a certified safety
system.
