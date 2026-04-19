#!/usr/bin/env python3
"""
gt_to_wheel_odom — HIL-only bridge that mirrors `/agv/sim/ground_truth/pose`
as `/agv/wheel_odom` (nav_msgs/Odometry).

Why this exists:
  The sim's drive chain has 5-20% efficiency (cmd 0.5 m/s → ~25-50 mm/s real).
  `joint_states_to_wheel_odom` integrates wheel encoder *rotation* which
  matches the commanded velocity (not the physical motion), so it over-reports
  position by 10-150x. The resulting wheel_odom destabilizes the dual-EKF:
  once the brain believes the robot has moved past the goal, Nav2 declares
  "reached" while the real robot is still far away (observed: round 14 wp01
  SUCCEEDED at GT err=0.33 m in 10.5 s, with wheel_odom reporting 3.16 m/s
  twist while GT moved 20 mm/s).

  In production, the ODrive CAN telemetry matches physical motion because
  real wheels don't slip 150x — so this problem is HIL-only.

Approach:
  Subscribe to `/agv/sim/ground_truth/pose` (PoseStamped in world frame,
  published by sim_isaac_handler at ~10 Hz). Republish as nav_msgs/Odometry
  on `/agv/wheel_odom` with frame_id='odom' and child_frame_id='base_link',
  at a higher fixed rate (50 Hz) so ekf_local has enough samples per cycle.

  Twist is NOT derived by finite differencing (noisy). Publish zero twist
  and let ekf_local integrate pose differences via its own Kalman update.

Frames:
  GT comes in 'world' frame (Isaac's internal). We publish in 'odom' frame.
  Since sim_isaac_handler keeps world aligned with the Nav2 map spawn (both
  are at the USD origin), 'world' = 'map' = 'odom' for HIL purposes.

IMPORTANT: this is a PURE VALIDATION helper. It cannot go to production —
 it depends on sim ground truth that doesn't exist on the real robot.
Only launched under `hil_mode:=true` via `use_gt_odom:=true` (new arg).
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class GtToWheelOdom(Node):
    def __init__(self) -> None:
        super().__init__("gt_to_wheel_odom")

        self.declare_parameter("gt_topic", "/agv/sim/ground_truth/pose")
        self.declare_parameter("output_topic", "/agv/wheel_odom")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("pose_cov_xy", 0.0001)
        self.declare_parameter("pose_cov_yaw", 0.001)

        in_topic = str(self.get_parameter("gt_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.odom_frame = str(self.get_parameter("odom_frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame_id").value)
        self.cov_xy = float(self.get_parameter("pose_cov_xy").value)
        self.cov_yaw = float(self.get_parameter("pose_cov_yaw").value)

        self.last_gt: Optional[PoseStamped] = None
        self._prev_pose: Optional[object] = None
        self._prev_stamp = None
        self._last_vx = 0.0
        self._last_vyaw = 0.0

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(PoseStamped, in_topic, self._on_gt, qos_sub)
        self.pub = self.create_publisher(Odometry, out_topic, qos_pub)
        # Also pin ekf_global by republishing GT as an absolute pose that
        # ekf_global's pose0 input (marker_pose) consumes. Without this,
        # ekf_global integrates ekf_local's differential output and drifts
        # over longer navs (round 23 wp03: 2 m EKF drift by end of nav).
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/agv/marker_pose", qos_pub
        )
        self.create_timer(1.0 / self.rate, self._publish_tick)

        self.get_logger().info(
            f"gt_to_wheel_odom: {in_topic} -> {out_topic} @ {self.rate} Hz "
            f"(cov_xy={self.cov_xy}, cov_yaw={self.cov_yaw})"
        )

    def _on_gt(self, msg: PoseStamped) -> None:
        # Compute twist on each *new* GT message (not on publish timer — the
        # timer fires at 50 Hz but GT arrives at ~10 Hz, so 4 out of 5 timer
        # ticks would finite-difference identical poses and report vx=0.
        # That drives ekf_local's velocity estimate toward zero while pose
        # is still advancing → cumulative pose lag (round 19 wp05: 0.13 m).
        now = self.get_clock().now()
        vx = 0.0
        vyaw = 0.0
        if self._prev_pose is not None and self._prev_stamp is not None:
            dt = (now - self._prev_stamp).nanoseconds / 1e9
            if dt > 1e-3:
                prev = self._prev_pose
                dx = msg.pose.position.x - prev.position.x
                dy = msg.pose.position.y - prev.position.y
                pyaw = self._quat_yaw(prev.orientation)
                cyaw = self._quat_yaw(msg.pose.orientation)
                c, s = math.cos(-pyaw), math.sin(-pyaw)
                vx = (c * dx - s * dy) / dt
                dtheta = cyaw - pyaw
                dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
                vyaw = dtheta / dt
        self._prev_pose = msg.pose
        self._prev_stamp = now
        self.last_gt = msg
        self._last_vx = vx
        self._last_vyaw = vyaw

    def _publish_tick(self) -> None:
        if self.last_gt is None:
            return
        now = self.get_clock().now()
        out = Odometry()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose = self.last_gt.pose
        cov = [0.0] * 36
        cov[0] = self.cov_xy
        cov[7] = self.cov_xy
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = self.cov_yaw
        out.pose.covariance = cov
        # Zero twist with huge covariance — ekf_local's odom0_config has
        # (vx, vyaw) enabled, but the absolute pose channel (x, y, yaw) is
        # the authoritative one here. Round 21 showed that computing twist
        # from finite-differenced GT poses caused the EKF to over-predict
        # between pose updates (est drifted 1.2 m ahead of gt). Letting
        # the EKF derive velocity internally from pose deltas is stabler.
        out.twist.twist.linear.x = 0.0
        out.twist.twist.angular.z = 0.0
        tcov = [0.0] * 36
        tcov[0] = 1e6; tcov[7] = 1e6; tcov[14] = 1e6
        tcov[21] = 1e6; tcov[28] = 1e6; tcov[35] = 1e6
        out.twist.covariance = tcov
        self.pub.publish(out)

        # Also publish PoseWithCovarianceStamped to pin ekf_global to GT.
        pose_out = PoseWithCovarianceStamped()
        pose_out.header.stamp = now.to_msg()
        pose_out.header.frame_id = "map"
        pose_out.pose.pose = self.last_gt.pose
        pose_out.pose.covariance = cov
        self.pose_pub.publish(pose_out)

    @staticmethod
    def _quat_yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> None:
    rclpy.init()
    node = GtToWheelOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
