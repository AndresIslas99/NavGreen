"""
AGV HIL Full Stack — Single-command launch for HIL simulation mode

Launches the complete Jetson autonomy brain against simulated sensors from PC:
  - robot_state_publisher (URDF → static TF)
  - agv_hil_bridges/joint_states_to_wheel_odom (sim /agv/joint_states → /agv/wheel_odom)
  - pointcloud_to_laserscan (sim /agv/zed/point_cloud → /agv/scan)
  - cuVSLAM (via agv_slam include, consumes sim /agv/zed/left|right/* + IMU)
  - covariance_override_node (wheel_odom + imu + vslam → *_cov topics)
  - ekf_local  (wheel_odom_cov + imu_cov → odom→base_link)
  - ekf_global (local + visual_slam_odom_cov → map→odom)
  - Nav2 stack (planner, controller, costmaps, lifecycle)
  - Operator backend (dashboard + teleop + REST + WebSocket on :8090)

Does NOT launch (provided by PC sim as hardware-emulator outputs):
  - agv_odrive (no CAN hardware, sim emulates encoders via joint_states)
  - ZED SDK (vendor locked to hardware; sim emulates outputs via raytracing)

External topics expected from PC (2026-04-17 contract after agv-greenhouse-sim
commit 3d44cec — sim stopped publishing anything the Jetson handles in production):
  /clock                                  rosgraph_msgs/Clock      sim time
  /agv/joint_states                       sensor_msgs/JointState   ≥30 Hz (encoder emu)
  /agv/imu/data                           sensor_msgs/Imu          100+ Hz (BMI088 emu)
  /agv/zed/left/image_rect_color          sensor_msgs/Image        30 Hz (SDK raytracing)
  /agv/zed/left/camera_info               sensor_msgs/CameraInfo   30 Hz
  /agv/zed/right/image_rect_color         sensor_msgs/Image        30 Hz
  /agv/zed/right/camera_info              sensor_msgs/CameraInfo   30 Hz
  /agv/zed/depth/depth_registered         sensor_msgs/Image        10-30 Hz
  /agv/zed/point_cloud/cloud_registered   sensor_msgs/PointCloud2  10 Hz (RELIABLE QoS)
  /agv/motor_state, /agv/drive_debug      (motor gate telemetry)
  /agv/sim/*                              (validation oracle — see specs/interfaces.yaml)

PC must subscribe to /agv/cmd_vel to close the control loop.

Usage:
  ros2 launch agv_bringup agv_hil_full.launch.py \\
    map:=/path/to/map.yaml

  # Skip cuVSLAM (fallback relay takes over):
  ros2 launch agv_bringup agv_hil_full.launch.py map:=... cuvslam_in_hil:=false

  # With CycloneDDS for cross-machine discovery:
  export CYCLONEDDS_URI=$(ros2 pkg prefix agv_bringup)/share/agv_bringup/config/cyclonedds_hil.xml
  ros2 launch agv_bringup agv_hil_full.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')

    fusion_dir = get_package_share_directory('agv_sensor_fusion')
    nav_dir = get_package_share_directory('agv_navigation')

    # Use production configs as base, override only what differs for HIL
    ekf_local_base = os.path.join(fusion_dir, 'config', 'ekf_local.yaml')
    ekf_global_base = os.path.join(fusion_dir, 'config', 'ekf_global.yaml')
    nav2_params_base = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    nav2_hil_overrides = os.path.join(nav_dir, 'config', 'nav2_hil_overrides.yaml')
    slam_toolbox_config = os.path.join(nav_dir, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('map', description='Path to map YAML file (required)'),
        DeclareLaunchArgument('enable_slam_toolbox', default_value='true',
                              description='Enable SLAM Toolbox for mapping'),
        DeclareLaunchArgument(
            'cuvslam_in_hil', default_value='true',
            description=(
                'Run cuVSLAM on the Jetson consuming the sim-published ZED '
                'stereo + IMU. When false, agv_hil_bridges/vslam_fallback_relay '
                'synthesises /visual_slam/tracking/odometry from wheel_odom instead.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_wheel_odom_bridge', default_value='true',
            description='Run agv_hil_bridges/joint_states_to_wheel_odom on the Jetson.',
        ),
        DeclareLaunchArgument(
            'use_gt_odom', default_value='false',
            description=(
                'HIL precision-validation shortcut: publish /agv/wheel_odom as '
                'a mirror of /agv/sim/ground_truth/pose instead of integrating '
                'sim encoder telemetry. The sim drive has 5-20% efficiency so '
                'wheel_odom over-reports by 10-150× and destabilizes the EKF. '
                'With this flag, joint_states_to_wheel_odom is disabled and '
                'Nav2 sees a perfect odometry matching physical motion. '
                'Production-unsafe; never enable outside HIL.'
            ),
        ),

        # ── Robot description (URDF → static TF) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_description'), 'launch', 'description.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true',
                'use_joint_state_publisher': 'false',
            }.items(),
        ),

        # ── Covariance override (fixes zero-covariance from Gazebo bridge) ──
        # Sim publishes wheel_odom and imu/data with all-zero covariances.
        # This relay reads the raw topics and publishes with realistic covariance
        # on _cov topics that the EKF subscribes to.
        Node(
            package='agv_sensor_fusion',
            executable='covariance_override_node',
            name='covariance_override',
            namespace=ns,
            parameters=[{
                'use_sim_time': True,
                'odom_pos_cov_xy': 0.001,
                'odom_pos_cov_yaw': 0.01,
                'odom_twist_cov_xy': 0.001,
                'odom_twist_cov_yaw': 0.01,
                'vslam_pos_cov_xy': 0.01,
                'vslam_pos_cov_yaw': 0.05,
                'imu_orient_cov': 0.001,
                'imu_gyro_cov': 0.0005,
                'imu_accel_cov': 0.01,
            }],
            remappings=[
                ('wheel_odom_raw', 'wheel_odom'),        # reads sim odom
                ('wheel_odom', 'wheel_odom_cov'),        # publishes with covariance
                ('imu_raw', 'imu/data'),                 # reads sim IMU
                ('imu/data', 'imu/data_cov'),            # publishes with covariance
            ],
            output='log',
        ),

        # ── Local EKF: sim wheel_odom + sim IMU → odom→base_link ──
        # Base config + HIL overrides:
        #   frequency: 20Hz (sim rates ~11Hz odom, ~17Hz IMU — 40Hz was too high)
        #   imu0: /agv/imu/data (sim IMU, not /zed/zed_node/imu/data)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            namespace=ns,
            parameters=[ekf_local_base, {
                'use_sim_time': True,
                'frequency': 50.0,               # high freq so EKF processes data on arrival, not timer-bound
                'odom0': 'wheel_odom_cov',        # from covariance_override relay
                'imu0': '/agv/imu/data_cov',      # from covariance_override relay
                # odom0_differential kept at the production default (false)
                # when use_gt_odom:=true. In that mode wheel_odom mirrors
                # the sim ground truth, so absolute pose is reliable and
                # Kalman updates snap ekf_local straight to GT.
                #
                # If use_gt_odom:=false and the joint_states integrator is
                # on instead, the sim's 5-20% drive efficiency makes wheel
                # pose unreliable. In that case enable differential here
                # (override via params file) so only the twist is consumed.
            }],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
                # Both EKFs advertise /agv/set_pose by default — namespace
                # collision makes it impossible for the precision test to
                # target one specifically. Split into explicit names.
                ('set_pose', 'ekf_local/set_pose'),
            ],
            output='log',
        ),

        # ── Global EKF: local + sim visual odom → map→odom ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            namespace=ns,
            parameters=[ekf_global_base, {
                'use_sim_time': True,
                'frequency': 20.0,
                'odom1': '/visual_slam/tracking/odometry_cov',
            }],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
                ('set_pose', 'ekf_global/set_pose'),
            ],
            output='log',
        ),

        # ── HIL bridges: joint_states → wheel_odom (+ optional vslam fallback) ──
        # Replaces what agv_odrive does on real hardware. Consumes the sim's
        # /agv/joint_states (encoder emulation) and publishes /agv/wheel_odom.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_hil_bridges'), 'launch', 'hil_bridges.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'enable_wheel_odom_bridge': LaunchConfiguration('enable_wheel_odom_bridge'),
                'cuvslam_in_hil': LaunchConfiguration('cuvslam_in_hil'),
                'use_gt_odom': LaunchConfiguration('use_gt_odom'),
            }.items(),
        ),

        # ── ZED optical frame static TF chain (HIL-only) ──
        # In production, the ZED wrapper publishes its internal URDF TF
        # (base_link → zed_camera_link → zed_left_camera_frame → ..._optical).
        # In HIL the wrapper isn't running but the sim's pointcloud header
        # uses frame_id=zed_left_camera_frame_optical; without these statics,
        # pointcloud_to_laserscan's message filter drops every cloud.
        # Same (0.700, 0.0, +0.010) offset as agv_slam.launch.py:82 and
        # robot_params.yaml (ZED z remeasured 2026-04-18 — see
        # agv_description/config/robot_params.yaml).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hil_base_to_zed',
            arguments=['--x', '0.700', '--y', '0.0', '--z', '0.010',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'zed_camera_link'],
            parameters=[{'use_sim_time': True}],
        ),
        # zed_camera_link → zed_left_camera_frame: ZED 2i half-baseline ~0.06m to the left.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hil_zed_to_left',
            arguments=['--x', '0.0', '--y', '0.06', '--z', '0.0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'zed_camera_link', '--child-frame-id', 'zed_left_camera_frame'],
            parameters=[{'use_sim_time': True}],
        ),
        # zed_left_camera_frame → zed_left_camera_frame_optical: ROS optical
        # convention rotation (X right, Y down, Z forward). Quaternion from
        # RPY (-pi/2, 0, -pi/2) = (x=-0.5, y=0.5, z=-0.5, w=0.5).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hil_zed_to_optical',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
                       '--frame-id', 'zed_left_camera_frame',
                       '--child-frame-id', 'zed_left_camera_frame_optical'],
            parameters=[{'use_sim_time': True}],
        ),

        # ── pointcloud_to_laserscan (sim /agv/zed/point_cloud → /agv/scan) ──
        # Post 2026-04-17 sim refactor, the sim stopped publishing /agv/scan —
        # that's Jetson software work in production, and we match that here.
        # The cloud publisher on the sim (Isaac Replicator) is RELIABLE; we
        # override the subscription to RELIABLE too, otherwise the QoS
        # mismatch silently drops messages (observed blocker in Round 2).
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace=ns,
            parameters=[{
                'use_sim_time': True,
                'min_height': 0.03,
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
                'qos_overrides./agv/zed/point_cloud/cloud_registered.subscription.reliability': 'reliable',
                'qos_overrides./agv/zed/point_cloud/cloud_registered.subscription.history': 'keep_last',
                'qos_overrides./agv/zed/point_cloud/cloud_registered.subscription.depth': 10,
            }],
            remappings=[
                ('cloud_in', '/agv/zed/point_cloud/cloud_registered'),
                ('scan', 'scan'),
            ],
            output='log',
        ),

        # ── cuVSLAM on Jetson (consumes sim-published ZED stereo + IMU) ──
        # When cuvslam_in_hil:=false, skipped; vslam_fallback_relay in
        # agv_hil_bridges synthesises /visual_slam/tracking/odometry instead.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_slam'), 'launch', 'agv_slam.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true',
                'enable_foxglove': 'false',
                'enable_gui': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('cuvslam_in_hil')),
        ),

        # ── Iter-39: sim_obstacle_relay (HIL safety substitute) ──
        # agv_hil_full doesn't run the production safety_supervisor chain;
        # without it the /agv/collision_monitor_state topic has no String
        # publisher, so rail_driver + mode_arbiter never see "stop" and
        # drive straight through crates in the greenhouse USD (observed
        # iter-38 c5_drive_in hitting Crate1 at (3.5, -2.0)). This relay
        # reads the sim's ground_truth obstacles oracle and publishes the
        # expected std_msgs/String (stop|slowdown|clear) at 10 Hz.
        Node(
            package='agv_hil_bridges',
            executable='sim_obstacle_relay.py',
            name='sim_obstacle_relay',
            namespace=ns,
            parameters=[{
                'stop_distance_m': 0.50,
                'slowdown_distance_m': 1.00,
                'robot_half_width_m': 0.42,
                'publish_rate_hz': 10.0,
                'use_sim_time': True,
            }],
            output='log',
        ),

        # ── Nav2 stack ──
        # Phase 2: controller_server publishes to /agv/cmd_vel_nav instead of
        # /agv/cmd_vel. The mode_arbiter (below) routes one of
        # {cmd_vel_nav, cmd_vel_approach, cmd_vel_rail} to /agv/cmd_vel based
        # on zone + operator mode. velocity_smoother still subscribes to
        # cmd_vel (arbiter output).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_navigation'), 'launch', 'navigation.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true',
                'hil_mode': 'true',
                'map': map_yaml,
                'nav2_params': nav2_params_base,
                'nav2_params_override': nav2_hil_overrides,
                'controller_cmd_vel_topic': 'cmd_vel_nav',
            }.items(),
        ),

        # ── Phase 2: zone detector (pose-based classification) ──
        Node(
            package='agv_zone_detector',
            executable='zone_detector_node',
            name='zone_detector',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            output='log',
        ),

        # ── Phase 2 Stage J: rail_detector (ZED depth → BEV → RANSAC) ──
        # Publishes /agv/rail_detections (PoseArray, 5 Hz) so rail_driver can
        # correct lateral drift inside a rail aisle via visual feedback instead
        # of relying solely on pose. Runs only when depth + camera_info are
        # available; silent otherwise (confidence=0 → consumers fall back to
        # pose-based alignment).
        Node(
            package='agv_rail_detector',
            executable='rail_detector_node',
            name='rail_detector',
            namespace=ns,
            parameters=[
                os.path.join(
                    get_package_share_directory('agv_rail_detector'),
                    'config', 'rail_detector_params.yaml'),
                {'use_sim_time': True},
            ],
            output='log',
        ),

        # ── Phase 2: mode arbiter — owns /agv/cmd_vel in the 3-mode stack ──
        # Iter-31: also load its YAML for the same reason (iter-22
        # min_mode_dwell_s / push_published flags were similarly not
        # applied at runtime).
        Node(
            package='agv_mode_arbiter',
            executable='mode_arbiter_node',
            name='mode_arbiter',
            namespace=ns,
            parameters=[
                os.path.join(
                    get_package_share_directory('agv_mode_arbiter'),
                    'config', 'mode_arbiter_params.yaml'),
                {'use_sim_time': True},
            ],
            output='screen',
        ),

        # ── Phase 2: rail driver (longitudinal-only inside aisles) ──
        # Iter-31: CRITICAL FIX — load rail_driver_params.yaml. Prior
        # to this line the node was running on header defaults
        # (yaw_abort_rad=0.26, lateral_abort_m=0.30, speed_max_mps=1.0),
        # silently ignoring every YAML tuning pass since iter-26d. The
        # iter-26d raise to yaw_abort=0.35 for margin over rail_approach
        # settle was never applied; iter-31 c5_drive_in aborted with
        # rail_yaw_err=0.333 > default 0.26 as a direct consequence.
        Node(
            package='agv_rail_driver',
            executable='rail_driver_node',
            name='rail_driver',
            namespace=ns,
            parameters=[
                os.path.join(
                    get_package_share_directory('agv_rail_driver'),
                    'config', 'rail_driver_params.yaml'),
                {'use_sim_time': True},
            ],
            output='log',
        ),

        # ── Scan grid mapper (live occupancy grid for commissioning) ──
        Node(
            package='agv_scan_mapper',
            executable='scan_grid_mapper_node',
            name='scan_grid_mapper',
            namespace=ns,
            parameters=[{
                'resolution': 0.05,
                'width': 400,
                'height': 400,
                'origin_x': -10.0,
                'origin_y': -10.0,
                'publish_rate_hz': 1.0,
                'map_frame': 'map',
                'use_sim_time': True,
            }],
            output='log',
        ),

        # ── SLAM Toolbox (loop-closed occupancy grid for commissioning) ──
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            parameters=[slam_toolbox_config, {'use_sim_time': True}],
            output='log',
            condition=IfCondition(LaunchConfiguration('enable_slam_toolbox')),
        ),

        # ── AprilTag detection (HIL shim, Round 44 iter-6 rewrite) ──
        # apriltag_ros's image_transport subscription fails to deliver
        # images across the USB-eth CycloneDDS hop (observed in iter-1..4:
        # subscription registered but 0 messages received).
        # iter-5 used the sim's visible_markers oracle, but that oracle
        # drops floor tags (z=0, normal=+Z) due to incidence filtering
        # — so rail_approach targets (ids 2, 3, 4, 12, 13, 33–37) were
        # never emitted. iter-6 rewrites the shim to be self-sufficient:
        # it loads markers_registry.yaml directly, reads the brain's TF
        # for world→camera, and projects every registered tag that
        # passes geometric visibility (FoV + incidence) checks. No sim
        # oracle dependency for detections.
        Node(
            package='agv_hil_bridges',
            executable='apriltag_sim_shim.py',
            name='apriltag_sim_shim',
            namespace=ns,
            parameters=[{
                'registry_file': os.path.join(
                    get_package_share_directory('agv_markers'),
                    'config', 'markers_registry.yaml'),
                'camera_info_topic': '/agv/zed/left/camera_info',
                'detections_topic': '/agv/detections',
                'image_frame': 'zed_left_camera_frame_optical',
                'world_frame': 'map',
                'default_tag_size_m': 0.2,
                'publish_rate_hz': 5.0,
                # Iter-16 / Round-44 wp04 diagnosis: the AGV's ZED optical
                # frame in HIL sits 10 mm above base_link (robot_params.yaml,
                # z re-measured 2026-04-18). base_link itself rests at
                # world z=0 under the sim's /reset teleport, so the camera
                # is ~1 cm above the floor tags. Incidence against a
                # ground-plane tag normal (+Z) is 89–90°. The previous 85°
                # gate silently rejected every floor tag from the
                # registry-driven shim, making rail_approach unable to
                # find its target even though the tag was geometrically
                # in frame. 89.5° lets grazing views through while still
                # rejecting views facing the tag's back (incidence > 90°).
                # Iter-40 F1: shim now composes world→optical from GT
                # base_link pose + static base→optical TF (bypassing the
                # 2D EKFs that put base_link at z=0). With the real cam
                # height (0.21 m) restored, incidence on a 1–2 m floor-
                # tag view settles ~79–84°, well inside 89.5°.
                'max_incidence_deg': 89.5,
                'use_sim_time': True,
            }],
            output='log',
        ),

        Node(
            package='agv_markers',
            executable='marker_correction_node',
            name='marker_correction',
            namespace=ns,
            parameters=[{
                'markers_registry_file': os.path.join(
                    get_package_share_directory('agv_markers'), 'config', 'markers_registry.yaml'),
                'max_detection_range': 5.0,
                'tag_size': 0.2,
                'covariance_xy': 0.01,
                'covariance_yaw': 0.03,
                'relocalization_threshold': 1.5,
                'min_confidence': 50.0,
                'use_sim_time': True,
            }],
            output='log',
        ),

        # ── Phase 2: rail approach (AprilTag-guided 2 cm alignment) ──
        # cmd_vel remapped to cmd_vel_approach so mode_arbiter can select
        # between Nav2, rail_approach, and rail_driver.
        # Iter-41: HIL overrides for fine_servo — sim drive chain runs at
        # ~4 % efficiency, so the prod max_fine_linear_vel (0.08 m/s) cannot
        # close the last 20–30 cm within max_fine_duration_s (120 s). Raise
        # both; the real robot still uses the prod defaults via the YAML.
        Node(
            package='agv_rail_approach',
            executable='rail_approach_node',
            name='rail_approach',
            namespace=ns,
            parameters=[
                os.path.join(
                    get_package_share_directory('agv_rail_approach'),
                    'config', 'rail_approach_params.yaml'),
                {
                    'registry_file': os.path.join(
                        get_package_share_directory('agv_markers'),
                        'config', 'markers_registry.yaml'),
                    # HIL TF tree uses `zed_left_camera_frame_optical`
                    # (not the rail_approach default
                    # `zed_left_camera_optical_frame`). The name mismatch
                    # made every fine_servoing tick fall into the
                    # tf_buffer_->lookupTransform exception handler, so
                    # no cmd_vel was ever published during fine servo —
                    # robot stalled at the coarse_standoff pose
                    # (explains err≈0.33 m plateau in iter-7..9).
                    'camera_frame': 'zed_left_camera_frame_optical',
                    # Iter-41: fine_servo HIL tuning. Sim drive ~4 %
                    # efficient, so commanded 0.08 m/s → ~3 mm/s real.
                    # At that rate closing 20 cm takes 67 s and the
                    # 120 s default max_fine_duration aborts before the
                    # servo can latch. Raise linear clamp + duration in
                    # HIL only; prod keeps the tight YAML defaults.
                    'max_fine_linear_vel': 0.30,
                    # max_fine_duration_s set below at iter-46 (was 240).
                    # Iter-44 Fase 2 Arch A: swap PnP tvec.z for a
                    # TF+registry estimate on the forward axis.
                    # DISABLED after iter-44 HIL validation: 3/3
                    # rail_approach wps regressed from ~10 cm (iter-43
                    # baseline) to 18-22 cm, all NAV_TIMEOUT at 270 s
                    # (fine_servo never settled). Most likely a
                    # coordinate/frame convention bug in the cam_z
                    # derivation from map→cam_optical. Needs unit-test
                    # evidence + re-derivation before re-enabling.
                    'use_registry_longitudinal': False,
                    'registry_max_stale_s': 2.0,
                    # Iter-46 Paso 1: tighten fine_servo settle tolerance for
                    # HIL precision diagnosis. The default yaml carries
                    # tolerance_xy=0.15 (raised by iter-26c on the assumption
                    # of σ_PnP ≈ ±2 cm at grazing — pre-SQPNP). The post-
                    # iter-42 stack (SQPNP + median-15 + GT-pose shim) has a
                    # measured noise floor of σ_z ≈ 0.5 mm at the c1_approach
                    # settle geometry (Monte-Carlo N=2000, see
                    # tools/pnp_bias_sweep.py). The 12.7 cm rail_approach
                    # plateau observed in iter-43/45 traces to this
                    # tolerance gate, NOT to PnP precision. 0.05 m gives
                    # 3× headroom over 3σ noise + ~17 mm safety vs the
                    # 250 ms settle drift at HIL drive efficiency 4 %.
                    # Predicted iter-46 rail_approach mean err_xy: 3-5 cm.
                    'tolerance_xy': 0.05,
                    # Iter-46 Paso 1.c: tag-visibility floor. The HIL sim
                    # camera intrinsics (fy=235, image height=376, 2 %
                    # margin) and physical geometry (cam height 0.21 m
                    # above floor tags z=0.002 m) impose a geometric
                    # minimum cam-to-tag forward distance of ~0.349 m
                    # before the tag's near corners (cam_z = center -
                    # 0.10 m) project below the image bottom. apriltag_
                    # sim_shim correctly rejects out-of-frame tags
                    # (`v_oob` counters in /agv/apriltag_sim_shim diag),
                    # rail_approach then loses the tag, tag_reacquire_
                    # timeout (3 s) fires, and the approach aborts.
                    # iter-45 (tolerance_xy=0.15) escaped this trap by
                    # latching SETTLED at the loose 13 cm err — robot
                    # never reached the visibility floor. iter-46 Paso
                    # 1.b (tolerance 0.05, no offset bump) hit the wall
                    # in 5/5 c*_approach wps.
                    # Raise the controller's desired offset to 0.40 m
                    # (5 cm margin over the visibility floor); the
                    # waypoint goals in waypoints_tagged_v5.yaml are
                    # adjusted to match so err_xy continues to measure
                    # actual standoff vs target standoff. The 2 cm
                    # Phase-2 target requires a taller camera mount or
                    # wall tags with vertical normal — tracked as
                    # Phase 3 (factor graph + hybrid 2.5-D).
                    'default_offset_x': 0.40,
                    # Iter-46 Paso 1.b: PI + stiction FF on the forward axis.
                    # Rationale (full derivation in iter-46 analysis):
                    #
                    # (1) Plant ID via /tmp/plant_id.py step-response sweep:
                    #     - V_stiction breakaway: cmd ≈ 0.020 m/s
                    #     - Reliable motion: cmd ≥ 0.025 m/s → 1-3 mm/s real
                    #     - Linear efficiency: ~4-8 % in cmd ∈ [0.035, 0.200]
                    #     - Plant gain K ≈ 0.05 (5 %)
                    # (2) Loop delay budget:
                    #     - Sample rate: apriltag_sim_shim @ 5 Hz = 200 ms
                    #     - Median filter (window=15): group delay ≈ 1.5 s
                    #     - Total dead time L ≈ 1.7 s
                    # (3) Control law layered:
                    #     a. P (Kp_linear yaml default 0.15) — dominant at
                    #        err > 0.25 m (cmd > stiction by factor of 2)
                    #     b. Stiction FF (stiction_ff_vel_mps=0.035) —
                    #        deterministic break-through when PI cmd
                    #        falls below the deadband. Inhibited inside
                    #        tolerance so the robot stops in band.
                    #     c. I (Ki_linear=0.05) — conservative to stay
                    #        stable with 1.7 s dead time. Lambda-tuning:
                    #        Ti ≥ L → Ki ≤ Kp/Ti = 0.15/1.7 ≈ 0.09;
                    #        0.05 leaves margin for sim jitter.
                    # (4) Anti-windup: conditional integration (integrator
                    #     frozen while FF active) + hard clamp at ±25 %
                    #     of max_linear_mps contribution (0.075 m/s).
                    # (5) Zero-cross reset on error sign change kills
                    #     the classic PI overshoot.
                    # Unit tests: see test_fine_servo_controller.cpp
                    # "FineServoPI" suite (10 cases, all green).
                    'Ki_linear': 0.05,
                    'stiction_ff_vel_mps': 0.035,
                    'max_fine_duration_s': 360.0,
                    'use_sim_time': True,
                },
            ],
            remappings=[('cmd_vel', 'cmd_vel_approach')],
            output='log',
        ),

        # ── C++ Image Server (camera + depth MJPEG on port 8091) ──
        Node(
            package='agv_image_server',
            executable='image_server_node',
            name='image_server',
            namespace=ns,
            parameters=[{
                'port': 8091,
                # Round 44: sim publishes under /agv/zed/*, not /zed/zed_node/*.
                'camera_topic': '/agv/zed/left/image_rect_color',
                'depth_topic': '/agv/zed/depth/depth_registered',
                'jpeg_quality': 70,
                'max_width': 640,
                'use_sim_time': True,
            }],
            output='log',
        ),

        # ── Operator backend (TypeScript, dashboard + teleop + REST + WS) ──
        Node(
            package='agv_ui_backend',
            executable='teleop_backend',
            name='teleop_server',
            namespace=ns,
            additional_env={
                'AGV_PORT': '8090',
                'AGV_NAMESPACE': 'agv',
                'AGV_DATA_DIR': '/home/orza/agv_data',
            },
            output='log',
        ),
    ])
