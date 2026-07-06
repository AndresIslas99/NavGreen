# agv_fleet_manager

TypeScript (Node.js) VDA 5050 master. Aggregates every robot's
state/visualization/connection topics from MQTT and exposes a REST +
WebSocket API for the fleet dashboard. Runs on **one host** (a Jetson or a
server on the greenhouse LAN). Not part of the default robot runtime.

## Responsabilidades

- Descubre robots por wildcard MQTT (`uagv/v2/+/+/state|visualization|connection`)
  y mantiene su último estado, posición y batería.
- Despacha órdenes VDA 5050 (`navigate`) e instant actions por robot.
- E-stop / resume de flota (acciones `emergencyStop` / `clearEmergencyStop`
  + `stopPause` — ver `fleet/README.md`).
- Gestión de tráfico (P3.4): zonas de exclusión/dirección, pausa por
  ocupación (`startPause`), detección y resolución de deadlocks.
- KPIs de flota.
- NO hace: puentear ROS 2 (eso es del adaptador), servir el dashboard de
  operación single-robot (agv_ui_backend :8090).

## Interfaces propias

- REST `http://<host>:8092/api/*` — fleet, state, navigate, action, estop,
  resume, kpis, traffic zones/occupancy/events. Puerto por `FLEET_PORT`
  (default **8092**; 8091 pertenece a `agv_image_server`, ver
  `specs/project.yaml`).
- WebSocket `ws://<host>:8092/ws/fleet` — snapshot + broadcast 2 Hz.
- MQTT: publica `uagv/v2/{man}/{serial}/order` y `.../instantActions`.

Auth: `FLEET_API_TOKEN` (secreto compartido). Si está definido, todo
`/api/*` exige `Authorization: Bearer <token>` y `/ws/fleet` exige
`?token=`. Sin definir, la API queda abierta (solo LAN aislada; se loguea
una advertencia al arrancar). Credenciales de broker via
`VDA_MQTT_USERNAME`/`VDA_MQTT_PASSWORD` (default de usuario:
`fleet-manager`, coincide con el aclfile).

## Interfaces consumidas

- MQTT `uagv/v2/+/+/state` (QoS 1), `.../visualization` (QoS 0),
  `.../connection` (QoS 1).

## Invariantes

- Semántica VDA 5050 de pausa: `startPause` pausa, `stopPause` reanuda.
- El e-stop de flota usa `emergencyStop` (acción específica del
  fabricante), nunca una acción de pausa.
- Robot sin mensajes por >60 s → `CONNECTIONBROKEN` y se libera de las
  zonas de tráfico.
- Configuración solo por entorno (`FLEET_PORT`, `FLEET_BIND_ADDR`,
  `FLEET_API_TOKEN`, `VDA_MQTT_*`); sin IPs ni puertos hardcodeados fuera
  de los defaults documentados.

## Failure modes

- Fleet manager caído → los robots siguen operando sus órdenes actuales y
  el dashboard single-robot (:8090) no se afecta; se pierde la pausa por
  tráfico y el e-stop de flota.
- Broker caído → estado de flota congelado; reconexión automática del
  cliente MQTT.
- Sin `FLEET_API_TOKEN` → superficie de control abierta en la LAN
  (advertencia al arrancar; ver SECURITY.md).

## Relación con otros specs

- `specs/project.yaml` — fleet sigue en out_of_scope; el puerto 8092 debe
  registrarse junto a `default_image_server_port` al moverlo a una fase.
- `specs/interfaces.yaml` — la sección websocket/rest solo cubre
  agv_ui_backend; esta superficie debe añadirse al registrar el paquete.
- `web/agv_dashboard` — consume `/api/traffic/*` y `/ws/fleet`
  (`VITE_FLEET_BASE`).
- `SECURITY.md`, `fleet/README.md` — modelo de seguridad.
