#!/usr/bin/env python3
"""
joint_states_to_wheel_odom — HIL bridge that integrates wheel encoder
positions from `/agv/joint_states` (sim-published) into the
`/agv/wheel_odom` nav_msgs/Odometry topic that ekf_local expects, using
the SAME kinematic constants as the real `agv_odrive` node so that the
EKF behaves identically in HIL and production.

Why this exists:
  After agv-greenhouse-sim commit 3d44cec, the sim no longer publishes
  /agv/wheel_odom — that's Jetson software work on the real robot (the
  ODrive CAN reader does it). In HIL, the sim provides joint_states as
  encoder emulation; this node does the same integration math as
  odrive_can_node but from joint positions instead of CAN telemetry.

Kinematic constants (loaded as ROS params, defaulting to the values in
src/agv_odrive/config/odrive_params.yaml):
  wheel_radius_m:  0.0781   (calibrated 2026-04-08)
  track_width_m:   0.960
  publish_rate_hz: 50
  left_joint:      "left_wheel_joint"
  right_joint:     "right_wheel_joint"

Covariance policy:
  pose.covariance  x,y = 0.005,  yaw = 1e-6
  twist.covariance vx  = 0.005,  vyaw = 1e-6
  matches what the existing covariance_override_node would enforce
  downstream.

Frames:
  frame_id = "odom", child_frame_id = "base_link". Does NOT publish TF
  (ekf_local is the sole owner of odom→base_link per ekf_local.yaml
  publish_tf:true + the 2026-04-17 sim refactor that stopped duplicating
  this TF from the sim side).
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class JointStatesToWheelOdom(Node):
    def __init__(self) -> None:
        super().__init__("joint_states_to_wheel_odom")

        self.declare_parameter("wheel_radius_m", 0.0781)
        self.declare_parameter("track_width_m", 0.960)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("left_joint", "left_wheel_joint")
        self.declare_parameter("right_joint", "right_wheel_joint")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("joint_states_topic", "/agv/joint_states")
        self.declare_parameter("wheel_odom_topic", "/agv/wheel_odom")

        self.r = float(self.get_parameter("wheel_radius_m").value)
        self.L = float(self.get_parameter("track_width_m").value)
        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.left_name = str(self.get_parameter("left_joint").value)
        self.right_name = str(self.get_parameter("right_joint").value)
        self.odom_frame = str(self.get_parameter("odom_frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame_id").value)
        js_topic = str(self.get_parameter("joint_states_topic").value)
        odom_topic = str(self.get_parameter("wheel_odom_topic").value)

        # Integrated pose in the odom frame.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Last encoder angles (rad) — used for delta integration.
        self.last_left: Optional[float] = None
        self.last_right: Optional[float] = None

        # Last observed wheel angular velocity (rad/s), for twist reporting.
        self.last_vx = 0.0
        self.last_vyaw = 0.0
        self.last_stamp = self.get_clock().now()

        js_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(JointState, js_topic, self._on_js, js_qos)
        self.pub = self.create_publisher(Odometry, odom_topic, odom_qos)
        self.create_timer(1.0 / self.rate, self._publish_tick)

        self.get_logger().info(
            f"joint_states_to_wheel_odom: r={self.r:.4f} m, L={self.L:.4f} m, "
            f"rate={self.rate:.0f} Hz, sub={js_topic}, pub={odom_topic}"
        )

    def _on_js(self, msg: JointState) -> None:
        try:
            li = msg.name.index(self.left_name)
            ri = msg.name.index(self.right_name)
        except ValueError:
            return
        if li >= len(msg.position) or ri >= len(msg.position):
            return
        left = float(msg.position[li])
        right = float(msg.position[ri])

        if self.last_left is None or self.last_right is None:
            self.last_left = left
            self.last_right = right
            self.last_stamp = self.get_clock().now()
            return

        dl = left - self.last_left
        dr = right - self.last_right
        self.last_left = left
        self.last_right = right

        # Arc lengths (m) per side.
        sl = dl * self.r
        sr = dr * self.r
        # Body-frame displacement.
        ds = 0.5 * (sl + sr)
        dtheta = (sr - sl) / self.L

        # Integrate in world frame using midpoint yaw.
        mid_theta = self.theta + 0.5 * dtheta
        self.x += ds * math.cos(mid_theta)
        self.y += ds * math.sin(mid_theta)
        self.theta = _wrap(self.theta + dtheta)

        # Twist from instantaneous wheel velocities (fallback: velocity field).
        vx = 0.0
        vyaw = 0.0
        if li < len(msg.velocity) and ri < len(msg.velocity):
            wl = float(msg.velocity[li])
            wr = float(msg.velocity[ri])
            vx = 0.5 * (wl + wr) * self.r
            vyaw = (wr - wl) * self.r / self.L
        else:
            now = self.get_clock().now()
            dt = (now - self.last_stamp).nanoseconds / 1e9
            if dt > 1e-3:
                vx = ds / dt
                vyaw = dtheta / dt
        self.last_vx = vx
        self.last_vyaw = vyaw
        self.last_stamp = self.get_clock().now()

    def _publish_tick(self) -> None:
        if self.last_left is None:
            return
        now = self.get_clock().now()
        m = Odometry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.position.z = 0.0
        half = 0.5 * self.theta
        m.pose.pose.orientation.z = math.sin(half)
        m.pose.pose.orientation.w = math.cos(half)
        cov = [0.0] * 36
        cov[0] = 0.005          # x
        cov[7] = 0.005          # y
        cov[14] = 1e6           # z (unused)
        cov[21] = 1e6           # roll
        cov[28] = 1e6           # pitch
        cov[35] = 1e-6          # yaw
        m.pose.covariance = cov
        m.twist.twist.linear.x = self.last_vx
        m.twist.twist.angular.z = self.last_vyaw
        tcov = [0.0] * 36
        tcov[0] = 0.005
        tcov[7] = 1e6
        tcov[14] = 1e6
        tcov[21] = 1e6
        tcov[28] = 1e6
        tcov[35] = 1e-6
        m.twist.covariance = tcov
        self.pub.publish(m)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> None:
    rclpy.init()
    node = JointStatesToWheelOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
