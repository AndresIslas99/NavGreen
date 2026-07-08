# fleet/ — VDA 5050 Fleet Layer

Optional multi-robot layer on top of the single-robot stack in `src/`.
It is **not** part of the default robot runtime started by
`agv_full.launch.py`; it is started explicitly via `./start_fleet.sh` or the
systemd units in [systemd/](systemd/).

```
fleet dashboard ──HTTP/WS──▶ agv_fleet_manager ──MQTT (VDA 5050)──▶ mosquitto
                                                                       ▲
                              per-robot: agv_vda5050_adapter ──────────┘
                                             │ rclnodejs
                                             ▼
                                       ROS 2 graph (/agv/*)
```

## Components

| Component | Runs on | Role |
|-----------|---------|------|
| [mosquitto/](mosquitto/) | one host (Jetson or server) | MQTT broker, VDA 5050 backbone (ports 1883, 9001) |
| [agv_fleet_manager/](agv_fleet_manager/) | one host | VDA 5050 master: fleet state aggregation, order dispatch, traffic zones, REST + WebSocket on port **8092** |
| [agv_vda5050_adapter/](agv_vda5050_adapter/) | each robot | Bridges ROS 2 topics/actions ↔ VDA 5050 MQTT messages |

Port **8091 is reserved for `agv_image_server`** (see
`specs/project.yaml` → `default_image_server_port`), which runs on the same
Jetson — the fleet manager therefore defaults to **8092** (`FLEET_PORT`).

## Security

Read [SECURITY.md](../SECURITY.md) first. Defaults assume an **isolated
greenhouse LAN**:

- **Fleet REST/WebSocket** — unauthenticated by default. Set
  `FLEET_API_TOKEN` to require `Authorization: Bearer <token>` on all
  `/api/*` routes and `?token=` on `/ws/fleet`. Bind to a specific interface
  with `FLEET_BIND_ADDR` (default `0.0.0.0`).
- **MQTT broker** — `allow_anonymous true` by default; the topic ACL in
  [mosquitto/config/aclfile](mosquitto/config/aclfile) is **inactive** until
  authentication is enabled. Follow the steps in
  [mosquitto/config/mosquitto.conf](mosquitto/config/mosquitto.conf) to turn
  on `password_file` + `acl_file`, then supply `VDA_MQTT_USERNAME` /
  `VDA_MQTT_PASSWORD` to both Node services. Bind the broker to one
  interface with `MQTT_BIND_ADDR` (docker-compose).

## VDA 5050 action semantics

Per VDA 5050, `startPause` **activates** pause mode (stop driving, keep the
order) and `stopPause` **deactivates** it. Pause never touches the robot's
latching `/agv/e_stop` topic.

Two manufacturer-specific instant actions map the fleet e-stop onto the
robot's safety topic:

| Action | Effect |
|--------|--------|
| `emergencyStop` | publishes `/agv/e_stop = true` (latches), cancels the active nav goal, keeps the order queue |
| `clearEmergencyStop` | publishes `/agv/e_stop = false`; does **not** resume motion — send `stopPause` to resume |

Unknown instant actions are reported with `actionStatus: FAILED`.

## Known gaps (tracked)

- The adapter dispatches `/agv/navigate_to_pose` goals without the operator
  backend's mode/motors-armed/localization gates (it only defers while
  paused or e-stopped). Same class of gap as `agv_waypoint_manager`
  (`specs/interfaces.yaml` → `known_gap`).
- The fleet interfaces are not yet registered in `specs/interfaces.yaml`,
  and `specs/project.yaml` still lists fleet management as out of scope.
  See the package `CLAUDE.md` files for the exact entries required.
