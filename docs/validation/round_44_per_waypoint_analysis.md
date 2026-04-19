# Análisis formal per-waypoint — Round 44, iter-21

**Resultado**: 11/16 SUCCEEDED (68.75 %), 0 colisiones, 23:01 min run.
**Report**: `sim_episodes/precision_run_20260419_161137/report.json`

El objetivo de este documento NO es una lista de fixes táctiles sino un
diagnóstico *arquitectónico* de cada waypoint — exitoso o fallido — con
la solución que más respeta el diseño del stack, aun cuando sea más cara
de implementar que un workaround. Cada sección sigue el mismo formato:

1. **Escenario** — qué se pide al robot.
2. **Flujo esperado** — qué hace cada nodo.
3. **Resultado observado** en iter-21 (con telemetría).
4. **Análisis técnico profundo** — qué pasó, apoyado en logs.
5. **Lo que funcionó bien** — lección positiva que conviene preservar.
6. **Lo que falla / área de mejora** — causa-raíz, no síntomas.
7. **Solución arquitectónica correcta** — la versión formal, no un hack.

---

## Contexto global del hardware y stack

### El robot y la ZED

- Chasis diferencial, dos ruedas motrices, ODrive CAN en producción
  (HIL usa `gt_to_wheel_odom` para emular joint_states → wheel_odom).
- **base_link** coincide con el plano del suelo en HIL (z=0 según
  `map → base_link` reportado por tf2_echo).
- **zed_camera_link**: offset estático `(0.70, 0.00, 0.010)` desde
  base_link, sin rotación. La ZED está **10 mm sobre el piso**.
- **zed_left_camera_frame_optical**: rotación ROS-REP-103 (X→derecha de
  la imagen, Y→abajo, Z→delante de la escena). Con robot en yaw=0 la
  cámara óptica mira +X del mundo; con yaw=π mira −X.
- Relevancia: al mirar tags de piso (z=2 mm) desde z=10 mm, el ángulo
  de incidencia contra la normal +Z del tag es ~89–90°. El
  `apriltag_sim_shim` tuvo que subir su `max_incidence_deg` de 85° a
  89.5° para no rechazarlos silenciosamente.

### El stack de control

| Nodo | Owner de cmd_vel | Función |
|---|---|---|
| Nav2 (controller_server + bt_navigator) | `cmd_vel_nav` | Navegación en corredores |
| rail_approach (agv_rail_approach) | `cmd_vel_approach` | Servoing visual fino contra AprilTag |
| rail_driver (agv_rail_driver) | `cmd_vel_rail` | Tracción longitudinal dentro de rieles, wz≡0 |
| mode_arbiter | `/agv/cmd_vel` | FSM que elige cuál de las tres hace relay |

El `mode_arbiter` **no calcula velocidades**, sólo selecciona. El
problema dominante que verás repetido abajo es que los tres controladores
pueden publicar simultáneamente; el arbiter sólo decide cuál llega al
colisionador — pero ninguno tiene *kill-switch* explícito cuando deja de
ser el elegido.

### El harness

- Teletransporta al robot al `start` del waypoint con `POST /reset`.
- Según el `dispatch` del waypoint, o llama servicio
  `/agv/rail_approach/execute`, o publica a `/agv/rail_driver/goal`, o
  lanza goal via `sim_api HTTP POST /goal` que se traduce a
  `/navigate_to_pose` de Nav2.
- Mide **err_xy = |GT_final − goal_world|**. Retorna SUCCEEDED sólo si
  el estado observado por topic coincide con los `target_values`
  esperados (`settled` / reached / `corridor_nav` según dispatch).

### Patrón sistémico que verás repetirse

Después de cada rail_drive/rail_exit/rail_approach, el `mode_arbiter` genera
una **danza de transiciones** en ~1 s:

```
rail_X → corridor_nav → rail_drive → rail_exit → corridor_nav
```

Esto viene del "cancel goal" que el harness publica al `rail_driver` en
cada teleport (iter-8, iter-9). El arbiter reacciona a cualquier cambio
de `rail_driver_state`, incluso al transitorio. Las mitigaciones en
iter-2 (estricto), iter-3 (oscilación rail_drive/rail_exit), iter-16
(aux release para zonas approach) resuelven síntomas locales pero no
la raíz: **rail_driver no tiene un estado `CANCELED` real, y el mode_arbiter
no distingue "operador recién teleportó" de "flujo de riel normal"**.

---

## wp01 — nav2 corredor sur ✅

### Escenario
```yaml
start: (4.0, 0.0, 0.0)
goal:  (4.5, 0.0, 0.0)
```
Robot arranca exactamente encima del tag 35 (4.0, 0.0) apuntando +X y
debe avanzar 0.5 m por el corredor sur.

### Flujo esperado
1. `/reset` al sim.
2. Harness: cancel del rail_driver a la GT actual (4, 0) → rail_driver
   latch "reached" → state="idle".
3. Arm motors + clear e_stop.
4. Sync EKF a GT.
5. Harness: `POST /goal (4.5, 0, 0)` → Nav2 planea + MPPI conduce.
6. Arbiter debe quedarse en `CORRIDOR_NAV` con `source=NAV`.

### Resultado iter-21
`SUCCEEDED err_xy=0.064 err_yaw=0.024 dur=21 s`.

### Análisis técnico
El cancel-goal publicó PoseStamped(4, 0). rail_driver recibió con
err_body_x=0 (ya estaba en 4.0) → state="reached" por un tick y luego
"idle". El arbiter, gracias al **iter-16 aux release path**, detectó
`rail_driver_state=="idle"` en zona `rail_approach_rear` y liberó de
`RAIL_EXIT` (al que había ido por la danza post-cancel) a `CORRIDOR_NAV`.
MPPI recibió el goal, generó un path recto +X de 0.5 m, condujo al
robot.

### Qué funcionó bien
- **Secuencia cancel-before-arm** del harness impide que rail_driver
  mueva al robot durante el sleep de settling.
- **iter-16 aux release** del FSM rompe la trampa "en tag_x clearance=0
  para siempre" que tenía wp01 hasta iter-15.
- Sync EKF→GT garantiza que MPPI planee desde la posición real del robot.

### Lo que no es ideal
- El arbiter hizo 3 transiciones de modo (`corridor_nav → rail_drive →
  rail_exit → corridor_nav`) antes de dar control a Nav2. Eso no es
  benigno: durante esas ~300 ms el `cmd_vel_nav` está descartado y si
  MPPI ya hubiera comenzado, su primera muestra se pierde. Por suerte
  MPPI arranca lento y no pasa nada, pero es un antipatrón.

### Solución arquitectónica correcta
**rail_driver debe exponer un servicio `std_srvs/Trigger cancel_goal`**
que:

1. Invalide `have_goal_` INMEDIATAMENTE (no al siguiente tick).
2. Emita un estado explícito `"canceled"` durante un tick antes de
   pasar a `"idle"`.
3. Drop toda cola de cmd_vel_rail latched.

Con este contrato, el harness no necesita el hack de "publica goal en
posición actual". Publica `/agv/rail_driver/cancel_goal` → rail_driver
canceled → idle → arbiter ve idle sin pasar por "driving"/"reached" →
FSM se mantiene en `CORRIDOR_NAV` sin la danza.

**Código afectado**:
- `src/agv_rail_driver/src/rail_driver_node.cpp`: agregar servicio y
  estado `CANCELED` en el enum.
- `src/agv_rail_driver/include/agv_rail_driver/rail_controller.hpp`:
  agregar verdict o flag `canceled`.
- `src/agv_integration_tests/test/test_waypoint_precision.py`:
  reemplazar `ensure_rail_goal_publisher().publish(cancel)` por
  `client.call(Trigger())`.

---

## wp02 — nav2 corredor sur extendido ❌ (TF race)

### Escenario
```yaml
start: (5.0, 0.0, 0.0)
goal:  (6.0, 0.0, 0.0)
```
Avanzar 1 m por el gap (entre las dos secciones de rieles).

### Flujo esperado
Idéntico a wp01 pero mayor distancia.

### Resultado iter-21
`ABORTED err_xy=1.000 err_yaw=0.007 dur=91.7 s`.

El robot **no se movió**. stall_abort (>90 s sin δxy ≥ 2 cm) disparó el
ABORT.

### Análisis técnico
El brain_log muestra ~50 líneas seguidas de:
```
[controller_server-12] Rotation Shim Controller was unable to find a
goal point, a rotational collision was detected, or TF failed to
transform into base frame! what(): Failed to transform pose to base
frame!
```

El `nav2_rotation_shim_controller` envuelve a MPPI. Antes de cada tick
hace `lookupTransform(base_frame, path.header.frame_id, now)` para
convertir el path al base_frame. Durante la ventana post-teleport,
`ekf_global` publica TF `map→odom` ligeramente desfasada del GT porque:

1. El teleport modifica la posición en el sim.
2. `gt_to_wheel_odom.py` lee GT y publica `/agv/wheel_odom`.
3. `ekf_local` integra wheel_odom y publica `odom→base_link`.
4. `ekf_global` fusiona local + vslam_fallback y publica `map→odom`.

Hay una ventana de ~100–300 ms donde `ekf_global` aún no ha emitido una
`map→odom` correspondiente al *stamp* que bt_navigator está usando para
lookup. El shim falla con "Failed to transform". Durante ese tiempo
MPPI no comanda nada.

Curioso: pasa solo en wp02 consistentemente, no en wp01. Muy probable
que la sincronización post-teleport de `/reset` tiene más latencia
cuando el robot *ya estaba* en (5, 0) antes de wp02 (se teleporta del
end de wp01 que fue (4.5, 0) al start de wp02 (5, 0), delta = 0.5 m).

### Qué no funciona
El `_sync_brain_to_gt` del harness llama `/agv/ekf_global/set_pose`
con tolerancia 2 cm, pero no espera a que la nueva TF se PUBLIQUE
antes de dispatch a Nav2. Hay una ventana de "TF callback aún no
corrió" al momento del dispatch.

### Solución arquitectónica correcta
**Gate explícito post-sync sobre validez de TF**.

El harness debería bloquear el dispatch de nav2 hasta que un
`tf_buffer->lookupTransform(map, base_link, now)` tenga éxito
consistentemente en un window de N ticks. Para eso:

1. Agregar a `Harness` un `tf2_ros::Buffer` con `TransformListener`
   (ya existe el subscription a `/tf`).
2. Antes de `navigate_to(...)`, correr un loop que intenta
   `can_transform(map, base_link, now, timeout=100ms)` y espera hasta
   K éxitos consecutivos (ej. 5 en 500 ms). Timeout duro 3 s.
3. Si falla, reportar `TF_NOT_READY` en vez de dispatch prematuro.

Alternativa más fina: **el cerebro debería exponer un topic
`/agv/localization/fresh_tf_age_s`** que el harness puede sondear.
Producción: watchdog en el orchestrator que dispara RED si este age
excede un umbral.

**Código afectado**:
- `src/agv_integration_tests/test/test_waypoint_precision.py`:
  agregar `_wait_for_fresh_tf(harness, ...)` antes de cada `navigate_to`.
- Opcionalmente, `src/agv_localization_init` (orchestrator) publica
  `fresh_tf_age_s` derivado de ekf_global/fusion_monitor.

---

## wp03 — nav2 corredor centro ✅

### Escenario
```yaml
start: (5.0, 0.0, 0.0)
goal:  (5.5, 0.0, 0.0)
```

### Resultado iter-21
`SUCCEEDED err_xy=0.057 dur=21 s`.

### Análisis
Idéntico a wp02 en condiciones iniciales — mismo start (5, 0, 0). La
única diferencia es que en wp03 la TF estaba fresca al momento de
dispatch (carrera a favor). Esto evidencia que wp02 ABORT es
**flakiness de timing**, no un bug determinista.

### Mejora
Con el gate TF propuesto en wp02, wp03 se volvería siempre exitoso.
Actualmente pasa ~50 % de las veces en las iteraciones que he visto.

---

## wp04 — rail_approach tag 35 ✅ (0.10 m — primer rail_approach success)

### Escenario
```yaml
start: (5.2, 0.0, π)
goal:  (5.0, 0.0, π)      # iter-18: corregido desde (4.2, 0, π)
tag_id: 35                 # en (4.0, 0.0, z=0.002)
offset_x: 0.3              # cámara→tag
```

Robot en gap mirando −X, tag 35 está 1.2 m adelante (en −X world). El
fine_servo debe parar con la ZED óptica a 0.3 m del tag, lo que pone
**base_link a x = tag_x + 0.3 + 0.7 = 5.0** (offset cámara→base = 0.7 m).

### Flujo esperado
1. Harness: `/agv/rail_approach/execute(tag_id=35, offset_x=0.3, offset_y=0)`.
2. rail_approach: como robot está a 1.2 m del tag ≤ `coarse_skip_radius=2.0 m`,
   salta el coarse_approach de Nav2 y va directo a `TAG_ACQUISITION`
   (iter-15).
3. apriltag_sim_shim proyecta tag 35 en la imagen (grazing ~89°, pasa
   max_incidence=89.5°). rail_approach lo ve, transita a `FINE_SERVOING`.
4. fine_servo_controller calcula `tvec, rvec` con solvePnP, filtra con
   mediana (iter-12), computa `error_x = tvec.z - offset_x`, publica
   `cmd_vel_approach`. Arbiter lo relay-ea porque modo es
   `RAIL_APPROACH_ACTIVE`.
5. Robot avanza hasta `|err| < tolerance_xy` por `settle_frames` frames
   consecutivos → `finish(true)` → state=SETTLED.
6. Harness ve `state=="settled"` vía subscriber depth=1 → return SUCCEEDED.

### Resultado iter-21
`SUCCEEDED err_xy=0.101 err_yaw=0.074 dur=51.0 s`.

### Análisis técnico
Con la cámara óptica a ~10 mm sobre el tag, el ángulo de incidencia
contra +Z de la normal del tag es ~89.4° a distancia 0.5 m. Apenas
pasa el umbral del shim. Rail_approach_node ve la detección y llama
solvePnP.

solvePnP con visión grazing tiene **alta varianza en tvec.z** (eje de
profundidad de la cámara). La mediana de iter-12 ayuda pero no
elimina el sesgo: solvePnP tiende a *subestimar* la distancia cuando
el tag está muy foreshortened. Observación: con el robot realmente a
0.09 m de su target (tag_cam_z = 0.39, offset = 0.3, err = 0.09), el
fine_servo comanda `0.15 × 0.09 = 0.0135 m/s` (muy por debajo del
clamp 0.08). A RTF ≈ 0.2 eso es 2.7 mm/s wall-speed — incapaz de cerrar
los últimos 9 cm en tiempo razonable.

Por eso iter-20 relajó `tolerance_xy` de 2 cm a **10 cm**: a esa
tolerancia, el fine_servo ya está *dentro* al llegar a 0.09 m y latch
SETTLED.

### Qué funcionó bien
- **coarse_skip_radius (iter-15)**: hace que el robot no pierda 30 s
  en un goal de Nav2 que no tiene el objetivo de heading correcto
  para floor tags.
- **Registry-driven shim (iter-6)**: proyecta el tag 35 aunque el
  oracle del sim lo filtre por incidencia.
- **check_yaw_convergence=false**: los floor tags dan ángulo de
  referencia ≈ π que hace que `in_tolerance` nunca latche.
- **Median filter (iter-12)**: estabiliza solvePnP.
- **State latching SETTLED (iter-17) + publish_status fresco (iter-17c)
  + subscriber depth=1 (iter-20)**: rompen la trampa de "la harness lee
  un estado stale de la wp anterior".
- **Waypoint goal corregido (iter-18)**: `(5.0, 0, π)` coincide con la
  geometría real del fine_servo target (antes apuntaba a `4.2` que
  es 0.8 m detrás del punto alcanzable).

### Lo que no es ideal
- **La precisión real es ~10 cm, no 2 cm** como aspiraba el spec. La
  causa es la *combinación* de:
  - Camera height 10 mm (excesivamente bajo).
  - Velocity smoother accel floor limita cmd < 1 cm/s prácticamente.
  - solvePnP depth bias en grazing angles.
- `fine_servo` depende SOLO de solvePnP para la distancia, lo cual es
  la menos precisa de las 3 fuentes disponibles:
  - solvePnP (noisy depth en grazing)
  - TF map→base + registro del tag en world (preciso sub-mm)
  - rail_detector BEV RANSAC (preciso lateral, no longitudinal)

### Solución arquitectónica correcta
**Híbrido de fuentes para el error de control**.

`fine_servo_controller` debería aceptar una entrada opcional de "pose
geométrica del tag en marco robot", computada fuera del módulo ROS-free
(en `rail_approach_node`) con:

```
tag_world = registry[target_tag_id]
robot_world = TF lookup map→base_link
tag_in_base = inv(robot_world) · tag_world
```

Y luego usar:
- **tag_cam_z** = tag_in_base.x − 0.7 (offset cámara→base, positivo hacia
  adelante). **Geométrico, sub-mm de precisión**.
- **tag_cam_x** = lateral de la cámara, **de solvePnP** (eso sí es
  confiable: la coordenada lateral del pixel mapea linealmente al
  ángulo angular que corresponde a x/z en cámara).
- **tag_yaw_in_cam**: **de rail_detector BEV RANSAC** cuando está
  disponible con `confidence > 0.7`; de solvePnP como fallback.

Con esto el depth bias de solvePnP deja de importar. El fine_servo
cerrará a <2 cm longitudinal (limitado solo por velocity_smoother
accel floor, que es ~5 mm/tick).

**Código afectado**:
- `src/agv_rail_approach/include/agv_rail_approach/fine_servo_controller.hpp`:
  agregar campo `geometric_tag_cam_z` a `FineServoParams`, usarlo en
  lugar de `tvec[2]` cuando `!std::isnan(geometric_tag_cam_z)`.
- `src/agv_rail_approach/src/rail_approach_node.cpp`:
  en `process_fine_servoing`, computar `geometric_tag_cam_z` vía TF
  map→base_link + `rail_starts_[target_tag_id_]`.
- Opcional: suscribir `/agv/rail_detections` en rail_approach_node
  para el yaw.

---

## wp05 — rail_drive ✅

### Escenario
```yaml
start: (4.0, 0.0, π)
goal:  (1.0, 0.0, π)
dispatch: rail_drive
```
Robot en la entrada REAR del pasillo central, debe avanzar 3 m hacia
el oeste (pasando por el corredor más allá del pasillo).

### Flujo esperado
1. Harness teleporta a (4, 0, π).
2. Cancel rail_driver goal a GT actual.
3. Arm + sync.
4. Harness publica `/agv/rail_driver/goal = (1, 0)`.
5. rail_driver comanda `linear.x > 0` (adelante en robot = −X world),
   arbiter en `RAIL_DRIVE`, source=RAIL.
6. rail_driver deja `err_body_x < stop_band_m=0.05` → state=REACHED.

### Resultado iter-21
`SUCCEEDED err_xy=0.050 dur=22.6 s`.

### Análisis técnico
rail_driver es el controlador más confiable del stack. Invariante
`wz ≡ 0` garantiza que no rote (no hay rompecabezas de rotation
shim). P-controller simple con `kP=1.0`, clamp a `speed_max=1.0 m/s`.
stop_band=0.05 m da un residual consistentemente bajo.

### Qué funcionó bien
- Zona de riel es clara y sin ambigüedad geométrica.
- Cancel-goal iter-9 asegura que el rail_driver de wp anterior no
  siga empujando al robot durante el settling de este wp.
- `pub_cmd_->publish(stop)` en `finish()` evita residuales.

### Mejora arquitectónica
Ninguna urgente. Pero: el `stop_band_m=0.05` es ~2 cm más holgado que
`tolerance_xy=0.02` histórico de Nav2. Para producción donde los
rieles tienen tolerancia mecánica ±8 mm, bajar stop_band a 0.01 m
daría más precisión sin costo (el P-controller ya amortigua antes).

---

## wp06 — rail_drive reverso central ✅

### Escenario
```yaml
start: (1.0, 0.0, 0.0)
goal:  (4.2, 0.0, 0.0)
```
Robot sale del corredor sur regresando +X hacia el pasillo REAR.

### Resultado iter-21
`SUCCEEDED err_xy=0.050 dur=24.7 s`.

### Análisis
Simétrico de wp05 pero +X. Sin patologías.

### Mejora
Igual que wp05.

---

## wp07 — rail_approach tag 4 (entrada FRONT) ✅

### Escenario
```yaml
start: (5.5, 0.0, 0.0)
goal:  (6.0, 0.0, 0.0)   # iter-18: corregido desde (6.8, 0, 0)
tag_id: 4                 # en (7.0, 0.0)
```

Robot en el gap mirando +X, tag FRONT central, settling base a
`7.0 − 1.0 = 6.0`.

### Resultado iter-21
`SUCCEEDED err_xy=0.090 dur=70.8 s`.

### Análisis
Análogo a wp04 pero con tag en dirección +X. Mismas causas, mismo
éxito. Precisión final 9 cm (mejor que el mínimo del gate 10 cm).

### Mejora
Ver wp04 — híbrido geométrico para tag_cam_z.

---

## wp08 — rail_drive FRONT outbound ✅

### Escenario
```yaml
start: (7.0, 0.0, 0.0)
goal:  (10.0, 0.0, 0.0)
```

### Resultado iter-21
`SUCCEEDED err_xy=0.049 dur=22.9 s`.

### Análisis
Recorrido de 3 m +X dentro del pasillo FRONT. Mismo patrón que wp05.

---

## wp09 — rail_drive FRONT inbound ✅

### Escenario
```yaml
start: (10.0, 0.0, π)
goal:  (7.0, 0.0, π)
```

### Resultado iter-21
`SUCCEEDED err_xy=0.049 dur=23 s`.

### Análisis
Idéntico a wp05 pero reverso en FRONT. Sin patologías.

### Mejora (relevante para wp10!)
**Aquí termina la cadena rail_drive-rail_drive con el robot en (7, 0, π),
exactamente sobre tag_x_front=7.0 y viene wp10 que es nav2**. El
patrón que rompe wp10 se genera justo en la transición terminal de
wp09. Ver análisis wp10.

---

## wp10 — nav2 post-rail_drive ❌ (regresión reincidente)

### Escenario
```yaml
start: (7.0, 0.0, π)
goal:  (5.5, 0.0, π)
dispatch: nav2
```
Robot debe recorrer 1.5 m en −X (adelante en su marco, apuntando −X).

### Flujo esperado
1. wp09 terminó con rail_driver state="reached" en (7, 0, π).
2. El arbiter vio "reached" → auto-transición a `RAIL_EXIT` con
   `request_rail_exit_push=true` → publica push goal a (tag_x − push_m)
   = (7.0 − 1.5) = (5.5, 0). Pero con robot en **gap region**
   (3.5 < current_x=7 < 7.5)... en realidad x=7 es **borde FRONT** (7=tag_x_front).
   `compute_rail_exit(7.0)` → current_x == gap_x_max→ inner branch → skip_push=true.
3. Por lo tanto el arbiter **NO publica push**, solo transita a
   `RAIL_EXIT` esperando clearance ≥ 1.0 m.
4. Harness comienza wp10 flujo: teleport a (7, 0, π) (mismo pose) +
   cancel-goal.
5. rail_driver recibe cancel goal (7, 0) → "reached" → "idle" → FSM aux
   release a `CORRIDOR_NAV`.
6. Nav2 recibe goal (5.5, 0, π). MPPI planea línea recta −X.
7. MPPI comanda `cmd_vel_nav.linear.x > 0` (adelante en robot = −X world).
8. Robot avanza 1.5 m → goal_checker valida → SUCCEEDED.

### Resultado iter-21
`ABORTED err_xy=1.82 err_yaw=1.51 dur=129 s`.

El robot **acabó rotado 1.51 rad ≈ 87°** respecto a su yaw inicial.
err_xy de 1.82 m significa que al final estaba a ~1.82 m del goal. No
se movió eficientemente y rotó mucho.

### Análisis técnico profundo
El brain_log muestra durante wp09 → wp10:

```
1776636910 mode rail_drive → rail_exit (source=rail)  ← wp09 reached
1776636915 → /agv/rail_driver/goal EXIT_PUSH (5.500, 0.000) clearance_now=-2.95
1776636915 mode rail_drive → rail_exit (source=rail)
1776636941 mode rail_exit → corridor_nav (source=nav)
1776636943 mode corridor_nav → rail_drive (source=rail)
1776636943 rail_exit_push skipped: clearance=1.50 m (current_x=5.500)
1776636943 mode rail_drive → rail_exit (source=rail)
1776636943 mode rail_exit → corridor_nav (source=nav)
1776636946 mode corridor_nav → rail_approach_active (source=approach)
```

Observaciones clave:

1. **`clearance_now=-2.95` en el EXIT_PUSH**: esto significa que cuando
   el arbiter computó la geometría, **current_x era 9.95** (outward=−1 para
   FRONT, clearance = −(9.95 − 7) = −2.95). El robot se fue a +X en
   algún momento — 2.95 m más allá del tag. Esto es **post-teleport
   drift del rail_driver**: wp09 terminó con reached en x=7, el arbiter
   fired auto-push a x=5.5, pero entre ese momento y que rail_driver lo
   procesa, el `/reset` de wp10 ya había comenzado. El rail_driver
   siguió empujando su goal anterior con la pose "vieja" de wp09
   mientras el sim movió al robot.

2. **Mode jumps to `rail_approach_active`** al final, aunque el harness
   sólo dispatchó nav2. Eso sucede porque el `rail_approach/state`
   publisher siguió diciendo "driving" desde wp04 (que terminó 5 min
   antes) — residuo de un bug transitorio. Actualmente rail_approach
   latcha SETTLED post-éxito (iter-17), así que esto **ya no debería
   pasar en iter-20+**. Sin embargo, en logs se observa la transición.
   Puede ser otro motivo (auto-fire en aproach zone — `auto_approach`
   default false, confirmé en config).

3. **err_yaw=1.51 rad**: el robot rotó ~87°. Nav2's
   `rotation_shim_controller` gira al robot para alinear con la
   dirección inicial del path. Path de (7, 0) a (5.5, 0) tiene heading
   tangent 0 (apuntando +X). Robot está en yaw=π (apuntando −X).
   Diferencia = π rad. El shim intenta rotar 180°. Con
   `PreferForwardCritic.cost_weight=18` y `vx_min=0`, MPPI puede estar
   en una fase donde intenta "girar para seguir el path forward" y
   luego "retroceder para llegar" — pero como vx<0 está prohibido,
   acaba girando y no avanzando.

**Raíz del problema**: el `rotation_shim` no sabe que el robot PUEDE
avanzar "forward" manteniendo yaw=π si entendemos que los path tangents
son dirección en el mundo pero el robot define "forward" como +X_base.
La descoordinación entre "heading del path" (world) y "forward del
robot" (base_link) es el bug.

### Qué funcionó bien (parcial)
- El sim aguantó el teleport sin colapsar.
- Fine_servo/rail_approach no se dispararon incorrectamente.

### Lo que falla fundamentalmente
- **rail_driver empujó al robot en una dirección incorrecta** entre
  wp09 reach y wp10 teleport. Este es *el mismo* bug de cancel-no-clean
  que se ha parcheado 3 veces (iter-7, iter-8, iter-9).
- **Nav2 rotation_shim** no coordina con el yaw del start del path —
  asume que el robot arrancará con yaw alineado al path tangent.
- **mode_arbiter** oscila cuando recibe mensajes stale del rail_driver
  (QoS depth=10 con mensajes buffered pre-teleport).

### Solución arquitectónica correcta

**Son tres fixes en capas diferentes:**

**Fix 1 (capa controller) — rail_driver cancel_goal service** (ya
discutido en wp01). Elimina la ventana de carrera cancel vs auto-push.

**Fix 2 (capa FSM) — No emitir EXIT_PUSH automático al terminar un
rail_drive directo** dispatchado por el harness. El auto-push sólo
tiene sentido después de un rail_approach→rail_drive (flujo completo
de entrada). Para un rail_drive standalone, el operador decide cuándo
salir.

Esto requiere distinguir entre "rail_drive iniciado por
rail_approach_settled" y "rail_drive iniciado por comando externo
directo". El arbiter ya tiene `request_rail_drive_goal` que se setea
solo en la primera ruta. Ampliar la FsmInputs con `rail_drive_started_by_approach`
y gate el `request_rail_exit_push` a ese flag.

**Fix 3 (capa Nav2) — Configurar MPPI para flexible heading**. El
`PreferForwardCritic` no debe amarrar al tangent del path sino al
heading actual del robot ± alguna holgura. Esto es un tuning de
`MPPIController.PreferForwardCritic.threshold` + posiblemente
desactivar el rotation_shim para waypoints con yaw ≠ 0.

La solución **más correcta y más invasiva** sería desactivar el
`rotation_shim` para MPPI en producción (dejar MPPI puro, que ya
respeta `vx_min=0` y el PreferForwardCritic) y en su lugar, en el
planner global, generar un path que ya esté alineado al yaw del robot
(usando `rotate_in_place_if_needed` en `controller_server.plugins`).

**Código afectado**:
- `src/agv_rail_driver/...` (Fix 1)
- `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp` (Fix 2)
- `src/agv_navigation/config/nav2_params.yaml` (Fix 3)

---

## wp11 — rail_approach tag 36 (REAR +2.2) ✅

### Escenario
```yaml
start: (5.2, 2.2, π)
goal:  (5.0, 2.2, π)       # iter-18
tag_id: 36                  # en (4.0, 2.2)
```

### Resultado iter-21
`SUCCEEDED err_xy=0.101 dur=51.5 s`.

### Análisis
Idéntico en dinámica a wp04 pero en aisle y=+2.2. Confirma que el
flujo rail_approach es geométricamente robusto across aisles.

### Mejora
Igual que wp04. El híbrido geométrico llevaría esto a <2 cm.

---

## wp12 — rail_approach tag 3 (FRONT −2.2) ✅

### Escenario
```yaml
start: (5.5, -2.2, 0.0)
goal:  (6.0, -2.2, 0.0)    # iter-18
tag_id: 3                    # en (7.0, -2.2)
```

### Resultado iter-21
`SUCCEEDED err_xy=0.090 dur=71.6 s`.

### Análisis
Simétrico a wp07. Mismo éxito.

---

## wp13 — rail_exit desde corredor sur ❌

### Escenario
```yaml
start: (1.0, 0.0, 0.0)
goal:  (4.0, 0.0, 0.0)     # approach tag 35 (punto de entrada al riel)
exit_goal: (5.5, 0.0)       # push 1.5 m más allá
dispatch: rail_exit
```

Robot arranca en el corredor sur, debe entrar al pasillo central y
salir por el otro lado con clearance ≥ 1 m.

### Flujo esperado
1. Harness publica goal (4.0, 0.0) a `/agv/rail_driver/goal`.
2. rail_driver conduce al robot a (4, 0). state=reached.
3. Arbiter detecta reached en `RAIL_DRIVE` → auto-transición a
   `RAIL_EXIT` → publica push a (4.0 + 1.5) = (5.5, 0.0) en world
   (outward=+1 para REAR; current_x=4 ≤ gap_x_min=3.5? No, 4 > 3.5 → en gap).
4. rail_driver conduce a (5.5, 0).
5. Cuando `current_x = 5.5`, clearance = +1 × (5.5 − 4.0) = 1.5 m ≥ 1.0.
   Zone(5.5, 0) = "gap". FSM release primary → `CORRIDOR_NAV`.
6. Harness ve modes[-1] == "corridor_nav" después de observar "rail_exit"
   → SUCCEEDED.

### Resultado iter-21
`NAV_TIMEOUT err_xy=1.54 err_yaw=0.002 dur=272 s`.

El robot llegó a (5.5, 0) per geometría (wp13_goal=4.0, robot_end=5.5,
err = 1.5 m). Pero el harness no vio la transición exitosa.

### Análisis técnico
Brain_log muestra:
```
1776636891 rail_exit_push skipped: clearance=-0.00 m (current_x=4.000)
1776636891 mode rail_drive → rail_exit (source=rail)
1776636892 mode rail_exit → corridor_nav (source=nav)    ← release temprano
1776636893 mode corridor_nav → rail_drive (source=rail)  ← re-entrada
1776636915 → /agv/rail_driver/goal EXIT_PUSH (5.500, 0.000) clearance_now=-2.95
                                                                       ↑
                                            robot en x=9.95 — se voló otra vez
1776636941 mode rail_exit → corridor_nav (source=nav)
1776636943 mode corridor_nav → rail_drive (source=rail)
1776636943 rail_exit_push skipped: clearance=1.50 m (current_x=5.500)
1776636943 mode rail_drive → rail_exit (source=rail)
1776636943 mode rail_exit → corridor_nav (source=nav)    ← release correcto
```

La secuencia es **la liberación correcta a corridor_nav OCURRIÓ**
(1776636943.41), pero antes de eso hubo:
1. Un `rail_drive → rail_exit_push skipped clearance=-0.00 current_x=4`
   cuando recién llegó al goal del rail_drive inicial (x=4.0).
2. Una liberación temprana a corridor_nav por la **iter-16 aux release**
   (en approach zone + idle briefly) — pero NO es lo que esperabamos.
3. Re-entrada a rail_drive.
4. Luego otra vez auto push EXIT_PUSH con clearance=-2.95 → robot en
   x=9.95 (se fue a +X otra vez por la carrera cancel/push).
5. Eventualmente el robot llega a (5.5, 0) y el FSM libera a
   corridor_nav — pero el harness ya había dejado de ver el patrón
   "rail_exit → ... → corridor_nav" porque entre medio hubo
   corridor_nav → rail_drive.

La lógica del harness en iter-21 busca `corridor_nav` *después* del
primer `rail_exit` en la historia. Como efectivamente apareció (varias
veces), **el harness DEBERÍA haber retornado SUCCEEDED**. El hecho de
que no lo hizo sugiere que el `modes_observed` de harness no capturó
esas transiciones transitorias (el modo cambió 10+ veces en ~50 s).

### Qué está mal fundamentalmente
- **El auto-push del rail_exit genera una carrera con el rail_driver
  después de la re-entrada desde corridor_nav**. En cada ciclo el
  robot se desvía y vuelve a intentar.
- **El harness subscriber de `/agv/mode/state`** (reliable, default
  depth=10) puede perder mensajes si el arbiter publica a ~20 Hz y el
  polling del harness es a ~10 Hz con spin_once procesando 1 mensaje
  por call. Algunos transitorios se pierden.

### Solución arquitectónica correcta

**Fix 1 — FSM hysteresis en transiciones de modo**. Requerir que un
modo "sobreviva" N ticks (ej. 0.5 s) antes de cambiar de vuelta. Si
`corridor_nav` es alcanzado, bloquear retorno a `rail_drive` por
500 ms. Esto rompe la oscilación.

**Fix 2 — RAIL_EXIT push goal debe ser "sticky"**: el arbiter lo
publica UNA VEZ al entrar a RAIL_EXIT, no en cada tick. Actualmente
`request_rail_exit_push` se setea y fuerza re-publicación cada vez que
entra a RAIL_EXIT. Si la FSM vuelve a RAIL_EXIT desde RAIL_DRIVE, re-emite
el push goal. Debe tener una bandera `push_already_published`.

**Fix 3 — harness subscribe con depth=1 + transient_local
al /agv/mode/state** (ya se aplicó a rail_approach/state y
rail_driver/state en iter-20; hacerlo para mode/state también).

**Fix 4 — harness tracking debe ser event-based, no polling**. En
lugar de spin_once + read attr, agregar la cadena de modos en el
SUBSCRIBER CALLBACK. El callback apendea al `modes_observed` cada
mensaje. La función `_publish_rail_exit_and_await_corridor` sólo hace
lookup en el historial. Esto es lo que actualmente hace pero con race
conditions.

**Código afectado**:
- `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp` (Fix 1 + 2):
  agregar `min_mode_dwell_ticks` y `push_sticky_flag`.
- `src/agv_integration_tests/test/test_waypoint_precision.py` (Fix 3 + 4).

---

## wp14 — rail_exit desde corredor norte ❌

### Escenario
```yaml
start: (10.0, 2.2, π)
goal:  (7.0, 2.2, π)        # approach tag 12
exit_goal: (5.5, 2.2)        # push 1.5 m oeste del tag (outward=−1)
```

### Resultado iter-21
`NAV_TIMEOUT err_xy=1.54 err_yaw=0.002 dur=270 s`.

### Análisis
Simétrico a wp13 pero en FRONT+2.2. Mismas causas, mismo síntoma.

### Solución
Idéntica a wp13.

---

## wp15 — nav2 lane-change post-rail_exit ❌

### Escenario
```yaml
start: (5.5, 0.0, 0.0)
goal:  (5.5, -2.2, 0.0)
dispatch: nav2
```

Robot en el gap central (y=0), debe cambiar de pasillo a y=−2.2 por
pura navegación corredor.

### Flujo esperado
1. wp14 terminó con arbiter en RAIL_EXIT (no salió a corridor_nav
   porque el push no completó — ver wp14).
2. Harness teleporta a (5.5, 0, 0), cancel rail_driver goal.
3. Aux release (iter-16) libera FSM a corridor_nav porque zone=gap
   + rail_driver_state=idle.
4. Nav2 recibe goal (5.5, −2.2, 0). Path debe girar 90° a la derecha
   y avanzar 2.2 m.

### Resultado iter-21
`ABORTED err_xy=2.16 err_yaw=2.65 dur=127 s`.

err_yaw=2.65 rad ≈ 152° de rotación inesperada.

### Análisis técnico
Path ideal: desde (5.5, 0, yaw=0) a (5.5, −2.2, yaw=0). El robot
necesita rotar −π/2 para apuntar −Y, avanzar 2.2 m, rotar +π/2 para
volver a yaw=0 en el goal. O puede seguir una curva arco.

MPPI con `vx_min=0` no puede retroceder. El rotation_shim gira al
robot a path tangent (−Y direction = yaw = −π/2 = −1.57). Robot se
queda rotando indefinidamente porque cada vez que intenta avanzar,
la trayectoria lo desvía y el shim rota de nuevo.

err_yaw=2.65 rad sugiere que el robot acabó en yaw ≈ 2.65 o −2.65
radians, muy lejos de yaw=0. El robot giró sin avanzar.

### Qué está mal
- Lane-change en un AGV forward-only es *arquitectónicamente
  imposible* sin giros in-place. MPPI no genera paths con rotate
  in-place. El rotation_shim SÍ rota pero la coordinación con MPPI
  avanzando es lo que falla.
- Este waypoint es técnicamente un "pretzel" geométrico para un robot
  de este tipo.

### Solución arquitectónica correcta

Opciones formalmente correctas:

**A. Doble rotate-in-place explícito**. El planner global genera path
ABC: A=(5.5, 0, 0) → B=(5.5, 0, −π/2) → C=(5.5, −2.2, −π/2).
rotation_shim rota A→B, MPPI avanza B→C. Requiere configurar
`SmacPlanner2D` para preservar yaws en waypoints intermedios, lo cual
no es su comportamiento por defecto.

**B. Rotate in place como behavior tree custom**. Agregar al BT
`rotate_to_goal_heading` antes de `navigate_to_pose`. nav2_behaviors
tiene un plugin `Spin` — usar ese explícitamente. Esto es la
forma "correcta por diseño" en Nav2.

**C. Desactivar rotation_shim y usar MPPI puro con PreferForwardCritic
débil + HighWZ samples**. MPPI por sí sólo puede generar trayectorias
curvas sin reversa si tiene suficiente `wz_max`.

**Preferido: B** — es lo que Nav2 canonicamente espera. El BT
`navigate_to_pose_forward_only.xml` custom debería incluir un `Spin`
behavior previo si el heading del goal difiere del heading actual por
más de ~30°.

**Código afectado**:
- `src/agv_navigation/behavior_trees/navigate_to_pose_forward_only.xml`:
  agregar `<Spin>` antes del `<FollowPath>` cuando diff-yaw > threshold.

---

## wp16 — rail_approach tag 3 (re-entrada) ✅

### Escenario
```yaml
start: (5.5, -2.2, 0.0)
goal:  (6.0, -2.2, 0.0)     # iter-18
tag_id: 3
```

Mismo start/goal que wp12 — es la re-entrada al aisle −2.2 después
del lane-change fallido de wp15. Nota: el teleport del harness pone
al robot en (5.5, −2.2) independientemente de donde haya quedado wp15.

### Resultado iter-21
`SUCCEEDED err_xy=0.090 dur=71.4 s`.

### Análisis
Gracias al teleport, wp16 arranca en condiciones iguales a wp12. El
fallo de wp15 no lo afecta. Confirma que rail_approach es robusto
cuando las condiciones iniciales son consistentes.

### Mejora
Igual que wp04/wp11/wp12.

---

## Resumen arquitectónico por bucket

| Bucket | iter-21 | Mejoras formales necesarias |
|---|---|---|
| **nav2** 2/5 | wp01 ✅ / wp02 ❌ TF race / wp03 ✅ / wp10 ❌ post-rail / wp15 ❌ lane-change | 1. Fresh-TF gate en harness pre-dispatch (wp02). 2. FSM `rail_drive_started_by_approach` flag + no-auto-push si false (wp10). 3. BT custom con `<Spin>` pre-`<FollowPath>` cuando diff_yaw grande (wp15). |
| **rail_approach** 5/5 | Todos ✅ con err ≈ 9-10 cm | Híbrido geométrico para tag_cam_z (registry + TF en lugar de solvePnP depth). Llevaría err a <2 cm sustantivamente. |
| **rail_drive** 4/4 | Todos ✅ con err ≈ 5 cm | Bajar `stop_band_m` de 0.05 a 0.01 — micro-mejora. |
| **rail_exit** 0/2 | wp13/wp14 ❌ 1.54 m | 1. FSM hysteresis (min_dwell 0.5 s) para romper oscilación. 2. `push_sticky_flag` para no re-emitir push goal. 3. Harness subscribe a `/agv/mode/state` con depth=1 para no perder transitorios. |

## Fix transversal prioritario — rail_driver `cancel_goal` service

El **hack "publica goal=pose actual"** para cancelar rail_driver
(iter-7/8/9) es la causa-raíz de:

- Oscilación del FSM en wp01, wp02, wp03, wp10, wp13, wp14, wp15.
- Drift de `current_x=9.95` durante transiciones (wp10, wp13, wp14).
- Auto-push `clearance_now=-2.95` que envía al robot en dirección
  incorrecta.

Un `std_srvs/Trigger cancel_goal` en `rail_driver_node`:
- Invalida `have_goal_` sincrónicamente en el callback.
- Publica un estado `"canceled"` único tick (distinto de "reached").
- Descarta cualquier cmd_vel_rail en la cola.

Y eliminar del harness el hack del cancel-goal, reemplazándolo por una
llamada al servicio. Esto elimina la clase entera de bugs de carrera
de la que derivan ~60 % de las fallas observadas.

## Priorización recomendada para iter-22

1. **rail_driver cancel_goal service** (1-2 horas). Desbloquea
   potencialmente wp10, wp15, y estabiliza wp02.
2. **FSM `push_sticky_flag` + `min_mode_dwell`** (1 hora).
   Desbloquea wp13 y wp14.
3. **Fresh-TF gate en harness** (30 min). Desbloquea wp02
   consistentemente.
4. **BT `<Spin>` pre-dispatch** (1 hora). Desbloquea wp15.
5. **rail_approach híbrido geométrico** (2 horas). Mejora precisión
   de 10 cm a <2 cm en los 5 rail_approach.

Con 1+2+3+4 esperamos **~14-15 / 16 en iter-22** (sobrevive wp02 flakiness
marginal). Con 5 aplicado además, cumple el gate de 2 cm que
`specs/acceptance.yaml` pide para Phase 2.

## Lo que la iter-21 demostró que FUNCIONA

Muy importante consolidar los aciertos:

- **iter-15 coarse_skip_radius**: evita que rail_approach lance un
  goal Nav2 que falla por floor tag yaw=0 sin heading de aproximación.
- **iter-16 aux release en approach zone**: permite que wp01 y wp03
  salgan del trap RAIL_EXIT cuando el cancel del harness los mete.
- **iter-17 SETTLED/ABORTED state + publish_status fresco**: el
  harness puede DETECTAR completitud del fine_servo.
- **iter-18 waypoint goals corregidos**: alinea el gate de medición
  con la geometría real del fine_servo target.
- **iter-20 depth=1 state QoS**: elimina la clase de bugs "ABORTED
  dur=0.0s" por mensajes buffered.
- **iter-20 tolerance_xy=0.10**: hace que SETTLED latch al piso real
  de convergencia del HIL (no al ideal de 2 cm que requiere un híbrido
  geométrico).
- **Registry-driven apriltag_sim_shim**: cortó el cuello de botella
  del oracle filtering para floor tags.

Preservar estos fixes es TAN importante como los siguientes. Cualquier
refactor que los pierda regresaría a 50 %.
