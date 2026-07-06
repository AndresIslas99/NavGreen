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
| Operator dashboard / REST / WebSocket (`agv_ui_backend`) | JWT auth with `viewer` / `operator` / `engineer` roles, users stored locally |
| MQTT broker (`fleet/mosquitto`) | Ships with `allow_anonymous true` — intended for a trusted local network only |
| Camera streaming (`agv_image_server`) | Plain HTTP MJPEG, no authentication |
| ROS 2 DDS traffic | Unencrypted by default (no SROS2 configuration in this repo) |

**Never expose any of these ports beyond the robot's isolated network.** If
your deployment requires remote access, put the network behind a VPN and
enable broker authentication before doing anything else.

## Safety boundary

Operational safeguards in this repository (collision monitor configuration,
mode arbitration, software E-stop paths) are **not certified functional
safety**. Certified human safety requires external hardware-integrated scope —
safety-rated scanners, PLC/relay logic, and a compliance process. Do not
describe or rely on the software in this repository as a certified safety
system.
