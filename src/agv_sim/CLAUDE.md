# agv_sim

Hardware-free **Gazebo Classic** simulation of the AGV differential-drive
robot for **ROS 2 Humble**. `dev_only: true` in TASK.yaml — it is a
development / simulation tool and is **never** part of the robot runtime
stack.

Its whole reason to exist: a contributor with no robot can run one command,
spawn the AGV in a greenhouse-ish world, drive it with the keyboard, and watch
odometry — without CAN, ODrive, a Jetson, or any hardware.

## Responsabilidades

- Componer la geometría **existente** de `agv_description`
  (`urdf/agv_full.urdf.xacro` → `agv_base.xacro` + `wheel.xacro`) y añadir
  únicamente lo que Gazebo necesita para simular una tracción diferencial de
  dos ruedas: un bloque `<ros2_control>` con el plugin
  `gazebo_ros2_control/GazeboSystem`, el plugin `libgazebo_ros2_control.so`, y
  etiquetas `<gazebo reference="...">` de fricción/material.
- Reutilizar **verbatim** las ganancias de `diff_drive_controller` /
  `joint_state_broadcaster` de
  `src/agv_hw_interface/config/agv_controllers.yaml` para que el sim conduzca
  cinemáticamente como el robot real.
- Publicar odometría en tópicos **sin namespace** (`/cmd_vel`, `/odom`,
  `/joint_states`, `/tf`, `/clock`) para que un recién llegado maneje con un
  solo comando.
- Proveer un mundo Gazebo Classic ligero y robusto para CI (sol + suelo +
  4 cajas "hileras de cultivo").

### Lo que este paquete NO hace

- **No** ejecuta cómputo de producción de ningún tipo: sin filtrado, sin
  SLAM, sin EKF, sin Nav2. Eso corre en el Jetson, nunca aquí. El sim solo
  aporta **inputs** (física + tracción sin sensores).
- **No** emula sensores: sin cámaras, sin lidar, sin IMU. Es un sim de
  *drivetrain* solamente. (Por eso `gzserver` headless no necesita GPU y
  arranca en CI.)
- **No** usa el namespace de producción `/agv/`. Es deliberadamente plano.
- **No** reemplaza `agv-greenhouse-sim` ni los HIL bridges (`agv_hil_bridges`).
  Este es un sim de escritorio autónomo, no la emulación del host HIL.
- **No** corre en el robot ni forma parte del stack de runtime.

## Interfaces propias

Todos los tópicos son **sin namespace** (demo autónoma para recién llegados).

Subscribed:
- `/cmd_vel` — `geometry_msgs/Twist` (unstamped, `use_stamped_vel: false`).
  Consumido por `diff_drive_controller` vía remap
  `/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel` en `sim.launch.py`.

Published:
- `/odom` — `nav_msgs/Odometry` @ 50 Hz. `frame_id: odom`,
  `child_frame_id: base_link`. Remap de `/diff_drive_controller/odom:=/odom`.
- `/joint_states` — `sensor_msgs/JointState` (posición + velocidad de
  `left_wheel_joint` y `right_wheel_joint`) por `joint_state_broadcaster`.
- `/tf`, `/tf_static` — `tf2_msgs/TFMessage`. El árbol
  `odom → base_link` lo publica `diff_drive_controller`
  (`enable_odom_tf: true`, ver abajo). `base_link → left_wheel/right_wheel/
  base_footprint` lo publica `robot_state_publisher`.
- `/clock` — `rosgraph_msgs/Clock`, por el plugin `gazebo_ros_init` de
  `gzserver`. Todos los nodos corren con `use_sim_time: true`.
- `/robot_description` — `std_msgs/String`, por `robot_state_publisher`
  (lo consume `spawn_entity.py -topic robot_description`).

Frames: `odom_frame = odom`, `base_frame = base_link` (más
`base_footprint` y las dos ruedas del URDF de `agv_description`).

## Interfaces consumidas

- `agv_description/urdf/agv_full.urdf.xacro` — geometría, joints, inercias y
  colisiones del robot (incluida, no redefinida).
- Ganancias de `src/agv_hw_interface/config/agv_controllers.yaml`
  (wheel_separation, wheel_radius, límites lineal/angular).
- `gazebo_ros` (`gzserver.launch.py`, `gzclient.launch.py`, `spawn_entity.py`),
  `gazebo_ros2_control` (`libgazebo_ros2_control.so` +
  `gazebo_ros2_control/GazeboSystem`), `controller_manager` spawner.

## Invariantes

- **El sim solo provee inputs.** Nunca corre cómputo que en producción
  correría en el Jetson (filtrado, SLAM, EKF, Nav2, percepción).
- **No es para el robot.** `dev_only: true`; no aparece en ningún launch de
  runtime ni en `agv_full.launch.py`.
- **Sin sensores.** No hay cámaras/lidar/`gpu_ray` en el mundo ni en el URDF,
  por lo que `gzserver` headless no necesita GPU ni contexto OpenGL — requisito
  para que corra en CI.
- **Sin fetch de red.** Sol y suelo están **inline** en `greenhouse.world`
  (no `<include><uri>model://…</uri>`), así que gzserver no toca la base de
  modelos de Gazebo.
- **Paridad cinemática.** `wheel_separation`, `wheel_radius` y límites vienen
  del **mismo** origen que `agv_hw_interface` (agv_geometry.yaml). Si divergen,
  el sim y el robot dejan de ser el mismo robot.
- **TF ownership (única diferencia intencional con producción):** aquí
  `enable_odom_tf: true` — el sim autónomo publica su propio
  `odom → base_link`. En **producción** es `false`, porque el stack dual-EKF
  es el dueño de `odom → base_link` y `map → odom` (ver CLAUDE.md raíz,
  "Localization architecture", y `agv_hw_interface/config/agv_controllers.yaml`).
- **Sin namespace.** Los tópicos son planos (`/cmd_vel`, `/odom`, …), no
  `/agv/…`, a propósito.

## Failure modes

- `libgazebo_ros2_control.so` / `gazebo_ros2_control` no instalado → el modelo
  aparece en Gazebo pero `controller_manager` no arranca dentro de gzserver →
  los spawners fallan → no hay `/joint_states` ni `/odom`. (gzserver en sí
  sigue vivo.) CI debe instalar `ros-humble-gazebo-ros2-control`.
- Sin fricción en las ruedas (`mu1/mu2`) → el robot patina o no avanza. Por eso
  el URDF añade `<gazebo reference="left_wheel/right_wheel">`.
- Spawnear el modelo a la altura equivocada → ruedas dentro del suelo o robot
  cayendo. Se spawnea `base_link` en `z=0.2` (base_footprint = suelo).
- `--controller-ros-args` no soportado por una versión muy vieja del spawner →
  los remaps `/cmd_vel` `/odom` no aplican y el controller queda en
  `/diff_drive_controller/*`. Requiere `controller_manager` de Humble
  reciente (backport presente).
- `teleop_twist_keyboard` sin TTY → no lee teclas. `teleop_sim.launch.py` lo
  abre en `xterm`; en headless usa una terminal propia (ver docstring).

## Relación con specs

- `specs/interfaces.yaml` — este paquete introduce, en variante **sim /
  dev_only**, `/cmd_vel` (sub), `/odom`, `/joint_states`, `/tf`, `/clock`
  (pub), **sin** el namespace `/agv/`. El parent registra estos contratos en
  el SSOT.
- `specs/state_machine.yaml` / CLAUDE.md raíz "Localization architecture" —
  documentan el dual-EKF de producción (`enable_odom_tf: false`); este paquete
  es la excepción autónoma (`true`).
- `specs/persistence.yaml` — sin artefactos persistentes nuevos (mundo y
  configs son estáticos, versionados).
- `specs/project.yaml` — herramienta `dev_only` de desarrollo/simulación, no
  target de despliegue en Jetson.
- Contraste con `src/agv_hil_bridges/` — aquél emula el host HIL sobre el
  Jetson (namespaced `/agv/…`); éste es un sim de escritorio autónomo y plano.
