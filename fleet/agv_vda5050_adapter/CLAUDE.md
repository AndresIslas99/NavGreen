# agv_vda5050_adapter

TypeScript (Node.js + rclnodejs) bridge between the robot's ROS 2 graph and
VDA 5050 2.0 MQTT topics. Runs on **each robot** alongside the ROS 2 stack.
Not part of the default robot runtime — started via `fleet/start_fleet.sh`
or `fleet/systemd/agv-vda5050-adapter.service`.

## Responsabilidades

- Publica el estado del robot como mensajes VDA 5050 (`state` 1 Hz,
  `visualization` 5 Hz, `connection` 15 s + Last Will).
- Ejecuta órdenes VDA 5050 (`order`) despachando metas a
  `/agv/navigate_to_pose`, nodo por nodo.
- Ejecuta instant actions: `cancelOrder`, `startPause`/`stopPause` (pausa
  VDA 5050) y las acciones específicas del fabricante `emergencyStop`/
  `clearEmergencyStop`.
- NO hace: arbitraje de cmd_vel, gestión de misiones del dashboard,
  gestión de tráfico (eso es del fleet manager).

## Interfaces propias

MQTT (prefijo `uagv/v2/{manufacturer}/{serialNumber}/`):
- Publica: `state`, `visualization`, `connection` (retained + LWT)
- Suscribe: `order`, `instantActions`

ROS 2 (aún **no registradas** en `specs/interfaces.yaml` — gap conocido;
al registrar, estas son las entradas exactas):
- Publica `/agv/e_stop` (`std_msgs/msg/Bool`) — SOLO desde
  `emergencyStop`/`clearEmergencyStop`; el topic es latching por semántica
  ("true persists until false"), tercer publicador junto a agv_ui_backend.
- Action client `/agv/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
  — tercer caller además de agv_ui_backend y agv_waypoint_manager.

## Interfaces consumidas

- `/agv/wheel_odom` (`nav_msgs/msg/Odometry`) — velocidad + Hz
- `/agv/odometry/global` (`nav_msgs/msg/Odometry`) — pose para VDA 5050
- `/agv/motor_state` (`std_msgs/msg/String` JSON) — armado/errores
- `/agv/e_stop` (`std_msgs/msg/Bool`) — espejo del estado real de e-stop
- `/slam/quality` (`std_msgs/msg/String` JSON) — localizationScore

## Invariantes

- La pausa VDA 5050 (`startPause`/`stopPause`) NUNCA escribe `/agv/e_stop`.
  Solo `emergencyStop`/`clearEmergencyStop` tocan ese topic.
- `clearEmergencyStop` no reanuda movimiento por sí solo; se requiere
  `stopPause` explícito.
- No se despachan metas de navegación mientras `paused` o `eStopActive`.
- Órdenes se validan antes de mutar estado: `orderUpdateId` duplicado se
  ignora, obsoleto se rechaza con `orderUpdateError` (WARNING) en `state`.
- `headerId` es monotónico POR topic (state/visualization/connection).
- Acciones desconocidas se reportan `FAILED`, nunca `FINISHED`.
- Configuración solo por entorno: `VDA_MQTT_BROKER`, `VDA_MANUFACTURER`,
  `VDA_SERIAL_NUMBER`, `AGV_NAMESPACE`, `VDA_MQTT_USERNAME`/`_PASSWORD`.

## Failure modes

- Adaptador caído → LWT publica `CONNECTIONBROKEN`; el fleet manager marca
  el robot desconectado (~60 s). El robot sigue operable por el dashboard.
- Broker inaccesible → cliente MQTT reintenta; el stack ROS 2 no se ve
  afectado.
- Nav2 no disponible → la orden queda aceptada pero no ejecuta (se loguea
  warning); no hay reintento automático.

## Known gap (gating)

Las metas a `/agv/navigate_to_pose` NO pasan por los gates del backend
(modo, motores armados, frescura del collision monitor, localización ≠
FAILED); solo se difieren con pausa/e-stop locales. Mismo patrón de
`known_gap` que agv_waypoint_manager en `specs/interfaces.yaml:790-795`.

## Relación con otros specs

- `specs/interfaces.yaml` — registro pendiente de las interfaces de arriba.
- `specs/project.yaml` — fleet/VDA 5050 sigue listado como out_of_scope;
  debe moverse a una fase al registrar este paquete.
- `SECURITY.md` — postura de red del broker MQTT.
- `fleet/README.md` — semántica de acciones y modelo de seguridad.
