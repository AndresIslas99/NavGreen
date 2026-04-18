# agv_hil_bridges

HIL-only bridge nodes. `dev_only: true` in TASK.yaml — NOT loaded in production.

## Purpose

After `agv-greenhouse-sim` commit `3d44cec` (2026-04-17), the sim host
stopped publishing topics that are Jetson software work on the real robot:
`/agv/wheel_odom`, `/visual_slam/tracking/odometry`, `/agv/scan`. The sim
now publishes only hardware-emulator outputs (`/agv/joint_states`,
`/agv/imu/data`, `/agv/zed/*`) and the LLM oracle (`/agv/sim/*`).

This package bridges the gap for HIL testing:

| Brain-side topic | Bridge node | Input(s) |
|---|---|---|
| `/agv/wheel_odom` | `joint_states_to_wheel_odom.py` | `/agv/joint_states` |
| `/visual_slam/tracking/odometry` (fallback) | `vslam_fallback_relay.py` | `/agv/wheel_odom` |

`/agv/scan` is produced by `pointcloud_to_laserscan` (standard ROS node,
added to `agv_hil_full.launch.py`), not by this package.

## Responsabilidades

- Reproducir la kinemática del `agv_odrive` real en HIL, integrando
  `/agv/joint_states` con las **mismas** constantes `wheel_radius` y
  `track_width` de `src/agv_odrive/config/odrive_params.yaml`.
- Entregar `/agv/wheel_odom` a la misma tasa (50 Hz) y covarianza que
  produce `covariance_override_node` (0.005 x/y, 1e-6 yaw).
- Relay opcional `vslam_fallback_relay` cuando cuVSLAM no corre en HIL.

**No** reemplaza hardware en producción — este paquete se omite en
`agv_full.launch.py`. No publica TF (ekf_local es el owner).

## Interfaces propias

Subscribed:
- `/agv/joint_states` — `sensor_msgs/JointState` (BE QoS), del sim host
  (emulación de encoders ODrive).

Published:
- `/agv/wheel_odom` — `nav_msgs/Odometry` @ 50 Hz. `frame_id: odom`,
  `child_frame_id: base_link`. Covarianza estática.
- `/visual_slam/tracking/odometry` (opcional, gate `cuvslam_in_hil:=false`)
  — relay de `wheel_odom` con covarianza incrementada para no duplicar
  peso en el EKF global.

## Interfaces consumidas

- `/agv/joint_states` (sim encoder emulation)
- `/agv/wheel_odom` (para el relay cuVSLAM fallback)

## Invariantes

- Las constantes `wheel_radius` y `track_width` vienen del **mismo**
  YAML que el real odrive_can_node (`src/agv_odrive/config/odrive_params.yaml`).
- Si alguien cambia los valores en un solo lado, ekf_local tratará HIL
  y producción como robots distintos → acceptance gates mentirán.
- Solo se lanza bajo `hil_mode:=true`. En `agv_full.launch.py` está
  gateado con `UnlessCondition(hil_mode)`.
- No publica TF (lo hace ekf_local).

## Failure modes

- `/agv/joint_states` silente → no hay integración → ekf_local sin odom0
  → Nav2 cree que robot nunca se mueve.
- Constantes distintas a odrive_params → HIL drift sistemático vs real.
- rclpy timer atasca → `/agv/wheel_odom` se degrada bajo 30 Hz → EKF
  rechaza por stale.

## Relación con otros specs

- `specs/interfaces.yaml` entradas:
  - `/agv/wheel_odom` con anotación `hil_override` apuntando a este pkg.
  - `/agv/joint_states` declarado como sim-published topic.
- `specs/launch_sequence.yaml#hil_simulation` describe que estos bridges
  se lanzan en el Jetson, junto con pointcloud_to_laserscan y cuVSLAM.
- `docs/validation/RUNBOOK_lan_hil.md` §4.1 diagrama de topic flow.
