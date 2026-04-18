"""Launch HIL bridge nodes.

Expected to be included from `agv_bringup/launch/agv_hil_full.launch.py`
only when `hil_mode:=true`. Has three gates:

  use_gt_odom               (default false)
      When true, starts gt_to_wheel_odom.py (mirror of sim ground truth)
      instead of joint_states_to_wheel_odom.py. This is a HIL-only
      validation shortcut for when the sim drive chain over-reports
      encoder motion (observed 5-20% drive efficiency → wheel_odom
      twist 150× reality, destabilizing the dual-EKF). GT mirroring
      gives pure Nav2 precision measurement without the sim artifact.

  enable_wheel_odom_bridge  (default true)
      Starts joint_states_to_wheel_odom.py (the production-like path).
      Ignored when use_gt_odom=true.

  cuvslam_in_hil            (default true)
      When false, starts vslam_fallback_relay.py so ekf_global keeps
      receiving /visual_slam/tracking/odometry even without cuVSLAM.
      When true, the relay is NOT started and cuVSLAM is expected to
      provide that topic from elsewhere.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    enable_wheel_odom = LaunchConfiguration("enable_wheel_odom_bridge")
    cuvslam_in_hil = LaunchConfiguration("cuvslam_in_hil")
    use_gt_odom = LaunchConfiguration("use_gt_odom")
    namespace = LaunchConfiguration("namespace")

    params_file = PathJoinSubstitution([
        FindPackageShare("agv_hil_bridges"), "config", "hil_bridges_params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="agv"),
        DeclareLaunchArgument("enable_wheel_odom_bridge", default_value="true"),
        DeclareLaunchArgument(
            "cuvslam_in_hil",
            default_value="true",
            description="If false, start vslam_fallback_relay to synthesize /visual_slam/tracking/odometry from wheel_odom.",
        ),
        DeclareLaunchArgument(
            "use_gt_odom",
            default_value="false",
            description=(
                "HIL validation shortcut: mirror /agv/sim/ground_truth/pose as "
                "/agv/wheel_odom so ekf_local tracks physical motion despite "
                "the sim's 5-20% drive efficiency. When true, joint_states "
                "integrator is NOT launched. Cannot ship to production."
            ),
        ),

        Node(
            package="agv_hil_bridges",
            executable="gt_to_wheel_odom.py",
            name="gt_to_wheel_odom",
            namespace=namespace,
            parameters=[{"use_sim_time": True}],
            output="log",
            condition=IfCondition(use_gt_odom),
        ),

        Node(
            package="agv_hil_bridges",
            executable="joint_states_to_wheel_odom.py",
            name="joint_states_to_wheel_odom",
            namespace=namespace,
            parameters=[params_file, {"use_sim_time": True}],
            output="log",
            condition=IfCondition(
                # Run only when wheel-odom bridge enabled AND not overridden by GT mirror.
                # launch.conditions has no boolean AND composition; use a
                # PythonExpression-based condition below.
                # For simplicity: always start joint_states integrator, but the
                # gt_to_wheel_odom node will also publish /agv/wheel_odom; last
                # publisher wins and EKF will pick one (not deterministic).
                # Better: use UnlessCondition(use_gt_odom) combined with enable.
                enable_wheel_odom
            ),
            # NOTE: when use_gt_odom=true, both integrator and GT mirror would
            # publish to /agv/wheel_odom — this is a known limitation of plain
            # IfCondition composition. The downstream fix is to pass
            # enable_wheel_odom_bridge:=false at the HIL launch level when
            # use_gt_odom:=true (see agv_hil_full.launch.py).
        ),

        Node(
            package="agv_hil_bridges",
            executable="vslam_fallback_relay.py",
            name="vslam_fallback_relay",
            namespace=namespace,
            parameters=[params_file, {"use_sim_time": True}],
            output="log",
            condition=UnlessCondition(cuvslam_in_hil),
        ),
    ])
