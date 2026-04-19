#!/usr/bin/env python3
"""
vslam_fallback_relay — OPTIONAL HIL bridge that republishes
`/agv/wheel_odom` as `/visual_slam/tracking/odometry` so that
`ekf_global` still has an odom1 input when cuVSLAM is not running.

Activated only when the launch arg `cuvslam_in_hil:=false`. When cuVSLAM
is running normally, this relay is NOT launched — the real cuVSLAM pose
takes over.

The relay intentionally adjusts covariance upward (0.02 x/y) compared to
wheel_odom's 0.005 so ekf_global weights it less; otherwise the EKF
would double-count wheel input (since ekf_local's odom0 is already
wheel_odom).

The output is published in DIFFERENTIAL mode semantics (ekf_global
config `odom1_differential: true` consumes it as deltas rather than
absolute pose). Frame_id = `odom`.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class VslamFallbackRelay(Node):
    def __init__(self) -> None:
        super().__init__("vslam_fallback_relay")

        self.declare_parameter("input_topic", "/agv/wheel_odom")
        self.declare_parameter("output_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("pose_cov_xy", 0.02)
        self.declare_parameter("pose_cov_yaw", 5e-3)

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self.cov_xy = float(self.get_parameter("pose_cov_xy").value)
        self.cov_yaw = float(self.get_parameter("pose_cov_yaw").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(Odometry, out_topic, qos)
        self.create_subscription(Odometry, in_topic, self._on_odom, qos)

        self.get_logger().info(
            f"vslam_fallback_relay active: {in_topic} -> {out_topic} "
            f"(cov_xy={self.cov_xy}, cov_yaw={self.cov_yaw})"
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist
        pcov = [0.0] * 36
        pcov[0] = self.cov_xy
        pcov[7] = self.cov_xy
        pcov[14] = 1e6
        pcov[21] = 1e6
        pcov[28] = 1e6
        pcov[35] = self.cov_yaw
        out.pose.covariance = pcov
        tcov = [0.0] * 36
        tcov[0] = self.cov_xy
        tcov[7] = 1e6
        tcov[14] = 1e6
        tcov[21] = 1e6
        tcov[28] = 1e6
        tcov[35] = self.cov_yaw
        out.twist.covariance = tcov
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = VslamFallbackRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
