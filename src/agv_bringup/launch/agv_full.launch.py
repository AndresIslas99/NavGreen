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

        # ── Robot description (URDF → static TF) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_description'), 'launch', 'description.launch.py'
                ])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # ── ODrive motor control (immediate) ──
        # With map: listens to cmd_vel_safe, which is the gated output of the
        # safety chain (teleop/Nav2 → velocity_smoother → collision_monitor →
        # cmd_vel_collision_safe → cmd_vel_gate → cmd_vel_safe → ODrive).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_odrive'), 'launch', 'odrive.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'cmd_vel_topic': 'cmd_vel_safe',
            }.items(),
            condition=IfCondition(has_map),
        ),
        # Mapping-first mode (no map): Nav2 and safety chain are not launched,
        # so the ODrive subscribes directly to /agv/cmd_vel from teleop_server.
        # This mode is commissioning-only — the operator is expected to drive
        # line-of-sight at low velocity to build the initial map.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_odrive'), 'launch', 'odrive.launch.py'
                ])),
            launch_arguments={
                'namespace': ns,
                'cmd_vel_topic': 'cmd_vel',
            }.items(),
            condition=UnlessCondition(has_map),
        ),

        # ── Ground-filtered LaserScan from ZED point cloud (production pipeline) ──
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
        ),

        # ── C++ Image Server (camera + depth MJPEG on port 8091) ──
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
            ],
            respawn=True,
            respawn_delay=2.0,
            output='log',
        ),

        # ── SLAM pipeline (t=3s, TF DISABLED — EKF owns transforms) ──
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
        ),

        # ── IMU vibration filter (t=3.5s — must start before EKF) ──
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
                    launch_arguments={'namespace': ns}.items(),
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
                    }],
                    output='log',
                ),
            ],
        ),

        # ── Nav2 stack (t=6s, only if map provided) ──
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
                        'use_sim_time': 'false',
                        'map': map_yaml,
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
                    launch_arguments={'namespace': ns}.items(),
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
                        'detector.quad_decimate': 2.0,
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
                    }],
                    output='log',
                    condition=IfCondition(enable_markers),
                ),
                Node(
                    package='agv_rail_approach',
                    executable='rail_approach_node',
                    name='rail_approach',
                    namespace=ns,
                    parameters=[{
                        'registry_file': os.path.join(
                            get_package_share_directory('agv_markers'), 'config', 'markers_registry.yaml'),
                        # Runtime registry — operator-defined rail_start tags from dashboard
                        'runtime_registry_file': '/home/orza/agv_data/runtime_markers_registry.yaml',
                        'tag_size': 0.2,
                        'camera_info_topic': '/agv/zed/left/camera_info',
                    }],
                    output='log',
                    condition=IfCondition(enable_markers),
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
                    }],
                    output='log',
                    condition=IfCondition(enable_behaviors),
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
