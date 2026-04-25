"""
AGV Full Stack — Single-command launch for real hardware (PRODUCTION)

Launches the complete autonomy stack:
  t=0s  robot_state_publisher (URDF → static TF)
  t=0s  odrive_can_node (motor control + wheel odom at 50 Hz)
  t=0s  pointcloud_to_laserscan (ground-filtered scan from ZED)
  t=0s  image_server (camera + depth MJPEG on :8091)
  t=0s  scan_grid_mapper (live occupancy grid)
  t=0s  operator backend (dashboard on :8090)
  t=2s  agv_slam (cuVSLAM, TF DISABLED)
  t=4s  dual EKF (local: odom→base_link, global: map→odom)
  t=5s  map_manager + waypoint_manager
  t=6s  Nav2 stack
  t=7s  marker_correction + rail_approach (optional)
  t=7s  behavior_executor (optional)

TF OWNERSHIP:
  odom → base_link:  ekf_local
  map → odom:        ekf_global
  cuVSLAM:           topic-only (/visual_slam/tracking/odometry)

Usage:
  ros2 launch agv_bringup agv_full.launch.py map:=/path/to/map.yaml
  ros2 launch agv_bringup agv_full.launch.py map:=/path/to/map.yaml enable_markers:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    enable_markers = LaunchConfiguration('enable_markers')
    enable_behaviors = LaunchConfiguration('enable_behaviors')
    enable_slam_localization = LaunchConfiguration('enable_slam_localization')
    slam_map_file = LaunchConfiguration('slam_map_file')
    # hil_mode: when true, skips Jetson-side nodes that conflict with sim
    # sensors (ZED wrapper+cuVSLAM, ODrive, IMU filter, pointcloud_to_laserscan,
    # image_server). The sim PC provides their equivalents over the DDS
    # network. The full brain stack (Nav2, dual EKF, safety chain,
    # auto_init_orchestrator, map_manager) stays active so PR-validation and
    # operator dashboards behave like production. See docs/hil_validation.md
    # for the sim contract.
    hil_mode = LaunchConfiguration('hil_mode')

    bringup_dir = get_package_share_directory('agv_bringup')
    nav_dir = get_package_share_directory('agv_navigation')
    cuvslam_no_tf = os.path.join(bringup_dir, 'config', 'cuvslam_greenhouse.yaml')
    # Fase 7 F3: map_dir unified to ${AGV_DATA_DIR}/maps. Previously
    # maps_dir = os.path.join(nav_dir, 'maps') which pointed at the colcon
    # install share (read-only template dir). Save operations to map_manager
    # were writing per-map .area/cuvslam/meta sidecars into the install
    # share while agv_ui_backend reads from AGV_DATA_DIR, causing save/load
    # asymmetry. All consumers now agree on AGV_DATA_DIR/maps — canonical
    # value in specs/project.yaml#deployment.default_data_dir.
    maps_dir = os.environ.get('AGV_DATA_DIR', '/home/orza/agv_data') + '/maps'
    missions_file = os.path.join(nav_dir, 'missions', 'missions.json')
    slam_loc_config = os.path.join(nav_dir, 'config', 'slam_toolbox_localization.yaml')

    # map_yaml controls whether Nav2 + the full safety chain come up. Empty
    # string ⇒ mapping-first mode: no Nav2, no collision_monitor, no gate.
    has_map = PythonExpression(["'", map_yaml, "' != ''"])

    # Composed guards for HIL mode. The ODrive branches are gated on both
    # `has_map` AND `hil_mode` because in HIL there is no CAN hardware to talk
    # to in either mapping-first or real-map scenarios — the sim PC closes the
    # cmd_vel loop instead.
    has_map_real = PythonExpression(
        ["'", map_yaml, "' != '' and '", hil_mode, "'.lower() != 'true'"])
    no_map_real = PythonExpression(
        ["'", map_yaml, "' == '' and '", hil_mode, "'.lower() != 'true'"])

    # Derive use_sim_time directly from hil_mode. In HIL the sim publishes
    # /clock at 72 Hz via IsaacSim's ClockPublisher and every publisher
    # (wheel_odom, imu, tf, etc.) stamps messages with IsaacReadSimulationTime.
    # Running the brain on wall_clock against those stamps yields TF_OLD_DATA
    # rejections on every lookup and premature STALE reports from safety and
    # fusion_monitor. In production (hil_mode=false) everything is wall_clock.
    # Known exception: /agv/motor_state and /agv/drive_debug are published by
    # the sim's sim_motor_gate with a WALL_CLOCK timer (commit 3a4467b, sim
    # side) because the real ODrive emits at 10 Hz wall regardless of sim
    # time. Subscribers must NOT age-validate those two topics against
    # use_sim_time=true — verified that agv_ui_backend does not.
    use_sim_time = PythonExpression(
        ["'true' if '", hil_mode, "'.lower() == 'true' else 'false'"])

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML (empty=start without map, load later via GUI)'),
        DeclareLaunchArgument('enable_markers', default_value='true',
                              description='Enable AprilTag marker correction'),
        DeclareLaunchArgument('enable_behaviors', default_value='false',
                              description='Enable behavior tree executor'),
        DeclareLaunchArgument('enable_slam_localization', default_value='true',
                              description='Enable SLAM Toolbox in localization mode for loop closure'),
        DeclareLaunchArgument('slam_map_file', default_value='',
                              description='Path to serialized SLAM Toolbox map (without extension)'),
        DeclareLaunchArgument(
            'hil_mode', default_value='false',
            description=(
                'HIL mode: skip nodes that conflict with sim sensors (ZED+cuVSLAM, '
                'ODrive CAN, IMU filter, pointcloud_to_laserscan, image_server). '
                'The full brain stack (Nav2, dual EKF, safety chain, orchestrator, '
                'map_manager) stays active. Default false = production with real '
                'hardware.'),
        ),
        # Sprint 1 Fase A4 (2026-04-24): top-level foxglove_bridge for engineer
        # diagnostics (TF tree, costmaps, lifecycle status). NOT a substitute
        # for the operator dashboard — Foxglove Studio went closed-source in
        # 2024, so production HMI flows MUST stay on agv_ui_backend. Off by
        # default to avoid burning ports/CPU when no engineer is connected.
        #
        # NAMING NOTE: this arg is intentionally `enable_foxglove_bridge`,
        # NOT `enable_foxglove`. Reason: agv_slam.launch.py is included with
        # `launch_arguments={'enable_foxglove': 'false', ...}` at t=3s, and
        # IncludeLaunchDescription in ROS 2 Humble MUTATES the parent
        # context's launch_configurations — so a same-named top-level arg
        # gets silently overwritten to 'false' by the time TimerActions
        # later in the launch evaluate `LaunchConfiguration('enable_foxglove')`.
        # Using a distinct name avoids the collision entirely.
        DeclareLaunchArgument(
            'enable_foxglove_bridge', default_value='false',
            description=(
                'Start foxglove_bridge on ws://<host>:8765 for engineer '
                'diagnostic clients. Off by default; turn on per-session '
                'with enable_foxglove_bridge:=true.'),
        ),

        # ── Robot description (URDF → static TF) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_description'), 'launch', 'description.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # ── ODrive motor control (immediate, real hardware only) ──
        # With map: listens to cmd_vel_safe, which is the gated output of the
        # safety chain (teleop/Nav2 → velocity_smoother → collision_monitor →
        # cmd_vel_collision_safe → cmd_vel_gate → cmd_vel_safe → ODrive).
        # Skipped when hil_mode=true — in HIL the sim PC closes the cmd_vel
        # loop via its sim_motor_gate + sim_drive_shaping_node pipeline.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_odrive'), 'launch', 'odrive.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'cmd_vel_topic': 'cmd_vel_safe',
            }.items(),
            condition=IfCondition(has_map_real),
        ),
        # Mapping-first mode (no map): Nav2 and safety chain are not launched,
        # so the ODrive subscribes directly to /agv/cmd_vel from teleop_server.
        # This mode is commissioning-only — the operator is expected to drive
        # line-of-sight at low velocity to build the initial map.
        # Also skipped when hil_mode=true.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_odrive'), 'launch', 'odrive.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'cmd_vel_topic': 'cmd_vel',
            }.items(),
            condition=IfCondition(no_map_real),
        ),

        # ── Ground-filtered LaserScan from ZED point cloud (production pipeline) ──
        # In production, the ZED wrapper publishes /agv/zed/point_cloud locally
        # on this Jetson (ROS2 IPC, zero network), so the ~180 Mbps subscription
        # here is free. In HIL the sim PC publishes the point cloud over the
        # network and the sim PC runs its own pointcloud_to_laserscan; running a
        # second copy here saturates the radio and starves /agv/scan. So we skip
        # it in HIL and consume /agv/scan directly from the sim. See the HIL
        # validation contract in docs/hil_validation.md.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace=ns,
            parameters=[{
                # min_height was 0.03 — that filtered out cables, tools, feet,
                # and any small floor obstacle. Lowered to 0.01 (1cm) to feed
                # those into /agv/scan so the collision_monitor can react.
                # The voxel_layer of the costmap has its own min_obstacle_height
                # (0.10) which keeps the COSTMAP tolerant of floor noise — this
                # change only affects the LAST-LINE safety reaction.
                'min_height': 0.01,
                'max_height': 2.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.003,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 8.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'target_frame': 'base_link',
            }],
            remappings=[
                ('cloud_in', '/agv/zed/point_cloud/cloud_registered'),
                ('scan', 'scan'),
            ],
            output='log',
            condition=UnlessCondition(hil_mode),
        ),

        # ── C++ Image Server (camera + depth MJPEG on port 8091) ──
        # Skipped when hil_mode=true: in HIL the camera/depth topics come from
        # the sim PC over WiFi (~30 MB/s RELIABLE). The brain-side image_server
        # would re-consume them, saturating the radio and starving /clock and
        # other small topics. In production the ZED feeds are local, no WiFi
        # involved — image_server runs normally.
        Node(
            package='agv_image_server',
            executable='image_server_node',
            name='image_server',
            namespace=ns,
            parameters=[{
                'port': 8091,
                'camera_topic': '/agv/zed/left/image_rect_color',
                'depth_topic': '/agv/zed/depth/depth_registered',
                'jpeg_quality': 70,
                'max_width': 640,
            }],
            output='log',
            condition=UnlessCondition(hil_mode),
        ),

        # ── Live occupancy grid from scan data ──
        Node(
            package='agv_scan_mapper',
            executable='scan_grid_mapper_node',
            name='scan_grid_mapper',
            namespace=ns,
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('agv_scan_mapper'), 'config', 'scan_mapper_params.yaml'
                ]),
                {'use_sim_time': use_sim_time},
            ],
            respawn=True,
            respawn_delay=2.0,
            output='log',
        ),

        # ── SLAM pipeline (t=3s, TF DISABLED — EKF owns transforms) ──
        # Skipped in HIL: the ZED wrapper requires physical hardware (it
        # fails with "CAMERA NOT DETECTED" after a 30s retry window), and
        # cuVSLAM is replaced by the sim PC's sim_global_odom shim which
        # publishes /visual_slam/tracking/odometry directly.
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agv_slam'), 'launch', 'agv_slam.launch.py'
                        ])),
                    launch_arguments={
                        'enable_foxglove': 'false',
                        'enable_gui': 'false',
                        'slam_params_override': cuvslam_no_tf,
                    }.items(),
                ),
            ],
            condition=UnlessCondition(hil_mode),
        ),

        # ── IMU vibration filter (t=3.5s — must start before EKF) ──
        # Skipped in HIL: the sim PC publishes a pre-filtered IMU through
        # isaac_ros_bridge on /agv/imu/data (bias-drift-injected but already
        # noise-free). ekf_local subscribes to /agv/imu/filtered in production
        # and to /agv/imu/data in HIL via the hil collision_monitor override
        # pathway; see the sim contract in docs/hil_validation.md.
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package='agv_sensor_fusion',
                    executable='imu_filter_node',
                    name='imu_filter',
                    namespace=ns,
                    parameters=[
                        PathJoinSubstitution([
                            FindPackageShare('agv_sensor_fusion'), 'config', 'imu_filter.yaml'
                        ]),
                    ],
                    remappings=[
                        ('imu/raw', '/agv/zed/imu/data'),
                        ('imu/filtered', 'imu/filtered'),
                    ],
                    output='log',
                ),
            ],
            condition=UnlessCondition(hil_mode),
        ),

        # ── Wheel slip detector (t=3.8s — between IMU filter and EKF) ──
        # Phase 2 of the diff-drive calibration plan. Listens to wheel
        # odom + filtered IMU + cuVSLAM odometry; republishes wheel
        # odom with covariance inflated when caster slip is detected.
        # ekf_local consumes wheel_odom_validated (NOT /agv/wheel_odom).
        # See docs/calibration/slip_detector_tuning.md for thresholds.
        TimerAction(
            period=3.8,
            actions=[
                Node(
                    package='agv_sensor_fusion',
                    executable='wheel_slip_detector_node',
                    name='wheel_slip_detector',
                    namespace=ns,
                    output='log',
                ),
                # Caster-aware dwell advisor (Phase 4, advisory variant).
                # Watches /agv/cmd_vel for direction reversals and
                # publishes /agv/caster/dwell_state. Passive observer;
                # does NOT mutate cmd_vel. Used by future controller
                # extensions (Nav2 MPPI custom critic or velocity_smoother
                # mod) to insert a 0.5 s pause before reversing.
                Node(
                    package='agv_sensor_fusion',
                    executable='caster_dwell_advisor_node',
                    name='caster_dwell_advisor',
                    namespace=ns,
                    output='log',
                ),
            ],
        ),

        # ── Dual EKF sensor fusion (t=4s) ──
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agv_sensor_fusion'), 'launch', 'fusion.launch.py'
                        ])),
                    launch_arguments={
                        'namespace': ns,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),

        # ── Factor graph estimator (t=4.5s, parallel to ekf_global) ──
        # Runs alongside ekf_global with publish_tf=false. Compare on
        # /agv/factor_graph/odometry vs /agv/odometry/global to validate.
        # Set publish_tf:=true to perform cutover from ekf_global.
        TimerAction(
            period=4.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agv_factor_graph'), 'launch', 'factor_graph.launch.py'
                        ])),
                    launch_arguments={
                        'namespace': ns,
                        'publish_tf': 'false',  # Parallel mode — ekf_global owns TF
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),

        # ── SLAM Toolbox lifelong mapping with loop closure (t=5s) ──
        # Async mode = optimizer runs in background, scan ingestion stays real-time.
        # Provides loop closure that bounds cuVSLAM drift. Doesn't publish TF
        # (transform_publish_period=0.0 in YAML); ekf_global owns map→odom.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox_localization',
                    namespace=ns,
                    parameters=[
                        slam_loc_config,
                        {'use_sim_time': use_sim_time},
                    ],
                    remappings=[
                        ('scan', 'scan'),
                    ],
                    output='log',
                    condition=IfCondition(enable_slam_localization),
                ),
            ],
        ),

        # ── Map manager + Waypoint manager (t=5s) ──
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='agv_map_manager',
                    executable='map_manager_node',
                    name='map_manager',
                    namespace=ns,
                    parameters=[{
                        'map_dir': maps_dir,
                        'map_topic': '/agv/map',
                        'use_sim_time': use_sim_time,
                    }],
                    output='log',
                ),
                Node(
                    package='agv_waypoint_manager',
                    executable='waypoint_manager_node',
                    name='waypoint_manager',
                    namespace=ns,
                    parameters=[{
                        'missions_file': missions_file,
                        'default_speed': 0.3,
                        'use_sim_time': use_sim_time,
                    }],
                    output='log',
                ),
            ],
        ),

        # ── Nav2 stack (t=6s, only if map provided) ──
        # hil_mode is passed down so navigation.launch.py can append the
        # collision_monitor HIL override (drops pointcloud_source — the raw
        # ZED point cloud is not available over the HIL WiFi without saturating
        # the radio; scan_source alone is enough for HIL validation).
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agv_navigation'), 'launch', 'navigation.launch.py'
                        ])),
                    launch_arguments={
                        'namespace': ns,
                        'use_sim_time': use_sim_time,
                        'map': map_yaml,
                        'hil_mode': hil_mode,
                        # Phase-2 arbiter ownership: controller_server publishes
                        # to cmd_vel_nav, mode_arbiter relays to cmd_vel.
                        'controller_cmd_vel_topic': 'cmd_vel_nav',
                    }.items(),
                    condition=IfCondition(has_map),
                ),
            ],
        ),

        # ── Safety layer (t=6.5s — supervisor + cmd_vel_gate) ──
        # Only when a map is loaded (Nav2 is up). collision_monitor publishes
        # cmd_vel_collision_safe → cmd_vel_gate → cmd_vel_safe, consumed by
        # odrive_can_node. The gate forces zero output on hardware_estop or
        # safety_supervisor unsafe verdict.
        # In mapping-first mode the safety chain is bypassed — see the ODrive
        # mapping-first include above.
        TimerAction(
            period=6.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agv_safety'), 'launch', 'safety.launch.py'
                        ])),
                    launch_arguments={
                        'namespace': ns,
                        'use_sim_time': use_sim_time,
                    }.items(),
                    condition=IfCondition(has_map),
                ),
            ],
        ),

        # ── AprilTag detection + marker correction (t=7s, optional) ──
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='apriltag_ros',
                    executable='apriltag_node',
                    name='apriltag_node',
                    namespace=ns,
                    parameters=[{
                        'family': '36h11',
                        'size': 0.2,
                        'max_hamming': 0,
                        'detector.threads': 2,
                        # decimate=2.0 made apriltag_node process
                        # 336×188 out of the 672×376 VGA NATIVE stream → a
                        # 20 cm tag at 2 m fell to ~13 px, below the
                        # tag36h11 decode floor. Field observation at the
                        # iter-37 visit: tag visible in the image but
                        # never decoded at rail_approach trigger distance.
                        # 1.0 = no decimation; CPU cost is marginal at
                        # VGA and we recover detection out to ~3 m.
                        # NOTE 2026-04-25: parameter is `detector.decimate`,
                        # NOT `detector.quad_decimate` — the older name was
                        # silently ignored, so this override never actually
                        # applied. That left runtime decimate=2.0 (the
                        # apriltag_ros default) and explained the iter-37
                        # "tag visible but undecoded" symptom in full.
                        # Verified by reading the live param after fix:
                        # `ros2 param get /agv/apriltag_node detector.decimate`.
                        'detector.decimate': 1.0,
                        'use_sim_time': use_sim_time,
                    }],
                    remappings=[
                        ('image_rect', '/agv/zed/left/image_rect_color'),
                        ('camera_info', '/agv/zed/left/camera_info'),
                    ],
                    output='log',
                    condition=IfCondition(enable_markers),
                ),
                Node(
                    package='agv_markers',
                    executable='marker_correction_node',
                    name='marker_correction',
                    namespace=ns,
                    parameters=[{
                        'markers_registry_file': os.path.join(
                            get_package_share_directory('agv_markers'), 'config', 'markers_registry.yaml'),
                        # Runtime registry — operator-defined tags from dashboard
                        'runtime_registry_file': '/home/orza/agv_data/runtime_markers_registry.yaml',
                        'max_detection_range': 5.0,
                        'tag_size': 0.2,
                        'covariance_xy': 0.01,
                        'covariance_yaw': 0.03,
                        'use_sim_time': use_sim_time,
                    }],
                    output='log',
                    condition=IfCondition(enable_markers),
                ),
                Node(
                    package='agv_rail_approach',
                    executable='rail_approach_node',
                    name='rail_approach',
                    namespace=ns,
                    # Iter-37: load rail_approach_params.yaml — same class of
                    # iter-26d bug the HIL launch had. Without this, all the
                    # tolerance_xy=0.15 / tolerance_yaw=0.25 / settle_frames=5
                    # tuning from iter-26c..27 defaults back to the header
                    # values at runtime on real hardware and rail_approach
                    # never settles.
                    parameters=[
                        os.path.join(
                            get_package_share_directory('agv_rail_approach'),
                            'config', 'rail_approach_params.yaml'),
                        {
                            'registry_file': os.path.join(
                                get_package_share_directory('agv_markers'),
                                'config', 'markers_registry.yaml'),
                            # Runtime registry — operator-defined rail_start tags from dashboard
                            'runtime_registry_file': '/home/orza/agv_data/runtime_markers_registry.yaml',
                            'tag_size': 0.2,
                            'camera_info_topic': '/agv/zed/left/camera_info',
                            # Iter-46 transfer: tighten settle gate. The 12.7 cm plateau
                            # observed in HIL iter-43/45 traced to the tolerance gate, not
                            # PnP precision (Monte-Carlo σ_z ≈ 0.5 mm at settle geometry,
                            # tools/pnp_bias_sweep.py). Real ZED 2i at 1280×720 has
                            # equivalent or better PnP, so the same 0.05 m gate applies.
                            'tolerance_xy': 0.05,
                            # Iter-46 transfer: longer fine-servo budget. Pure-P at the
                            # tighter 0.05 m gate may take longer to settle on the real
                            # plant; 360 s leaves margin without changing failure semantics.
                            'max_fine_duration_s': 360.0,
                            # NOTE: PI+FF gains (Ki_linear, stiction_ff_vel_mps) deliberately
                            # NOT copied from agv_hil_full.launch.py. Those were tuned via
                            # tools/plant_id.py against the HIL drive chain (4-8 % efficiency,
                            # deadband ≈ 0.020 m/s — sim-specific). Real ODrive plant ID must
                            # be run before enabling them; controller defaults to pure-P
                            # when Ki_linear == 0 by design (see iter-46 commit a8e9867).
                            # default_offset_x = 0.30 (yaml default) preserved: the 0.40 m
                            # bump in HIL was driven by sim camera fy = 235 at 672×376;
                            # real ZED at 1280×720 (fy ≈ 530) sees the floor tag well below
                            # the 0.349 m visibility floor that drove the HIL change.
                            'use_sim_time': use_sim_time,
                        },
                    ],
                    output='log',
                    # Phase-2 arbiter ownership: rail_approach must publish to
                    # cmd_vel_approach, not directly to cmd_vel. Without this
                    # remap rail_approach competes with mode_arbiter + teleop_server
                    # for the final /agv/cmd_vel topic and the safety-chain smoother
                    # deadband collapses the pulsed output.
                    remappings=[('cmd_vel', 'cmd_vel_approach')],
                    condition=IfCondition(enable_markers),
                ),
                # ── Phase 2 stack ported from agv_hil_full.launch.py (iter-37) ──
                # These four nodes exist in HIL but were never added to the
                # production launch. Without them, the 3-mode cmd_vel arbitration
                # (Nav2 / rail_approach / rail_driver), the rail-aisle
                # BLOCKED_LATERAL gate, and the marker_correction RELOC gates
                # (iter-31/34/36 fixes) have no subscribers on real hardware.
                Node(
                    package='agv_zone_detector',
                    executable='zone_detector_node',
                    name='zone_detector',
                    namespace=ns,
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='log',
                    condition=IfCondition(has_map),
                ),
                Node(
                    package='agv_rail_detector',
                    executable='rail_detector_node',
                    name='rail_detector',
                    namespace=ns,
                    parameters=[
                        os.path.join(
                            get_package_share_directory('agv_rail_detector'),
                            'config', 'rail_detector_params.yaml'),
                        {'use_sim_time': use_sim_time},
                    ],
                    output='log',
                    condition=IfCondition(has_map),
                ),
                Node(
                    package='agv_mode_arbiter',
                    executable='mode_arbiter_node',
                    name='mode_arbiter',
                    namespace=ns,
                    parameters=[
                        os.path.join(
                            get_package_share_directory('agv_mode_arbiter'),
                            'config', 'mode_arbiter_params.yaml'),
                        {'use_sim_time': use_sim_time},
                    ],
                    output='screen',
                    condition=IfCondition(has_map),
                ),
                Node(
                    package='agv_rail_driver',
                    executable='rail_driver_node',
                    name='rail_driver',
                    namespace=ns,
                    parameters=[
                        os.path.join(
                            get_package_share_directory('agv_rail_driver'),
                            'config', 'rail_driver_params.yaml'),
                        {'use_sim_time': use_sim_time},
                    ],
                    output='log',
                    condition=IfCondition(has_map),
                ),
                # Auto-localization orchestrator: listens for /agv/maps/loaded,
                # loads the matching cuVSLAM keyframe DB, waits for an AprilTag
                # (up to marker_wait_timeout_s), then calls
                # /visual_slam/localize_in_map. Publishes /agv/localization/state
                # which the dashboard displays as an informational LOC pill.
                #
                # Gated on has_map (NOT enable_markers): when AprilTags are
                # disabled via enable_markers:=false, Path A times out and the
                # orchestrator falls through to Path B (last-known pose from
                # {map}_meta.json) → DEGRADED, still localized. If we gated
                # this on enable_markers, disabling markers would also kill
                # cold-start and leave the robot without a map→odom origin.
                # AprilTag-dependent nodes (detector, marker_correction,
                # rail_approach) remain gated on enable_markers above — they
                # have no purpose without tags.
                Node(
                    package='agv_localization_init',
                    executable='auto_init_orchestrator_node',
                    name='auto_init_orchestrator',
                    namespace=ns,
                    parameters=[
                        os.path.join(
                            get_package_share_directory('agv_localization_init'),
                            'config', 'auto_init_params.yaml'),
                        {'map_dir': '/home/orza/agv_data/maps'},
                        {'use_sim_time': use_sim_time},
                    ],
                    output='screen',
                    condition=IfCondition(has_map),
                ),
            ],
        ),

        # ── Behavior executor (t=7s, optional) ──
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='agv_behaviors',
                    executable='behavior_executor_node',
                    name='behavior_executor',
                    namespace=ns,
                    parameters=[{
                        'trees_dir': '',
                        'use_sim_time': use_sim_time,
                    }],
                    output='log',
                    condition=IfCondition(enable_behaviors),
                ),
            ],
        ),

        # ── foxglove_bridge (optional, Sprint 1 Fase A4) ─────────────────
        # Diagnostic-only WebSocket bridge for Foxglove Studio. Lives at port
        # 8765, separate from the operator dashboard (8090). Started after
        # most of the stack so its topic discovery is complete by first
        # client connect.
        TimerAction(
            period=7.5,
            actions=[
                Node(
                    package='foxglove_bridge',
                    executable='foxglove_bridge',
                    name='foxglove_bridge',
                    parameters=[{
                        'port': 8765,
                        'address': '0.0.0.0',
                        'tls': False,
                        # Send compressed PNG/JPEG over the wire when topics
                        # are large (default true is fine; documented for
                        # discoverability).
                        'send_buffer_limit': 10000000,
                    }],
                    output='log',
                    condition=IfCondition(LaunchConfiguration('enable_foxglove_bridge')),
                ),
            ],
        ),

        # ── Operator backend (TypeScript, t=8s — after all ROS nodes for DDS discovery) ──
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='agv_ui_backend',
                    executable='teleop_backend',
                    name='teleop_server',
                    namespace=ns,
                    additional_env={
                        'AGV_PORT': '8090',
                        'AGV_NAMESPACE': 'agv',
                        'AGV_DATA_DIR': '/home/orza/agv_data',
                        # Pass the map basename (without extension) so the backend
                        # knows which map was loaded by Nav2's map_server at boot.
                        # The backend publishes this to /agv/maps/loaded ~10s after
                        # its own start to trigger the auto_init_orchestrator.
                        # If map arg is empty, this resolves to an empty string and
                        # the backend skips the boot-time publish.
                        'AGV_BOOT_MAP_NAME': PythonExpression([
                            "__import__('os').path.splitext("
                            "__import__('os').path.basename('",
                            map_yaml,
                            "'))[0]"
                        ]),
                    },
                    output='log',
                ),
            ],
        ),
    ])
