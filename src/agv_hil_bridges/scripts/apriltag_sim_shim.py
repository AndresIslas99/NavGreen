#!/usr/bin/env python3
"""apriltag_sim_shim — HIL-only bypass for the broken apriltag_ros image
pipeline.

Why this exists
---------------
`apriltag_ros`'s `image_transport`-based subscription fails to deliver
images across the USB-eth CycloneDDS hop (observed in Round 44 iter-1..4:
`ros2 node info /agv/apriltag_node` shows the subscription, but
"Image messages received: 0" even though rclpy raw subscriptions on the
same topic see ~20 Hz). Without detections, `marker_correction` +
`rail_approach` cannot localize against floor tags → every
`rail_approach` waypoint times out at start pose.

The sim's visible_markers oracle already knows which AprilTags are in
the ZED's FoV each tick, along with each tag's world-frame pose
(added by sim commit dd2cf06, `tag_world_pose` key). This shim:

  1. Subscribes `/agv/sim/ground_truth/visible_markers` (String JSON).
  2. Looks up `world → zed_left_camera_frame_optical` in the brain's TF
     tree (map → odom → base_link → … → optical).
  3. For each visible tag, projects the 3D corner positions (derived
     from `tag_world_pose` + known tag_size) into image pixels using
     the cached camera_info intrinsics.
  4. Publishes `apriltag_msgs/AprilTagDetectionArray` on `/agv/detections`
     with the same shape apriltag_ros would have produced if image
     delivery worked — same `id`, same `corners[4]`, same `centre`,
     same `family` ("tag36h11").

rail_approach then runs its own solvePnP on the synthesized corners, so
its control loop is unchanged. marker_correction sees the id + corners,
queries its own world pose from markers_registry.yaml, and emits the
pose correction exactly as if apriltag_ros were working.

IMPORTANT: this is HIL-only. Real deployments rely on the ZED image
stream + apriltag_ros; the shim depends on the sim-side oracle which
does not exist in production. Gated via `hil_mode:=true` in
agv_hil_full.launch.py.

Wire-format input (from sim commit dd2cf06):
  { "t_sim": ...,
    "robot_pose": [x,y,yaw],
    "camera_pose": [x,y,z],
    "count": N,
    "visible": [ { "id": ..., "distance_m": ..., "bearing_rad": ...,
                   "incidence_deg": ..., "tag_world_pose": {
                       "x": ..., "y": ..., "z": ...,
                       "qx": ..., "qy": ..., "qz": ..., "qw": ... } }, ... ] }

Behavior when inputs are missing
--------------------------------
- camera_info not yet received → drop silently (first ~1 s after start).
- TF lookup fails (tree incomplete, EKF just teleported) → drop this
  tick; next oracle message will try again.
- tag_world_pose absent (sim running older build) → skip that tag but
  keep processing others; shim emits a WARN once per start-up.
- Projected corner outside the camera FoV (behind, sideways, or past
  frame edges) → tag is dropped; consistent with apriltag_ros's "out
  of FoV → no detection" behavior.
"""
from __future__ import annotations

import json
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Return 3x3 rotation matrix from a unit quaternion.

    Small and allocation-free — called once per visible tag per tick.
    """
    # Normalize defensively (sim oracle should already, but float drift).
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


def _transform_to_matrix(ts) -> np.ndarray:
    """Turn a geometry_msgs/TransformStamped into a 4x4 homogeneous matrix."""
    t = ts.transform.translation
    q = ts.transform.rotation
    M = np.eye(4)
    M[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
    M[0, 3] = t.x
    M[1, 3] = t.y
    M[2, 3] = t.z
    return M


class AprilTagSimShim(Node):
    def __init__(self) -> None:
        super().__init__(
            "apriltag_sim_shim",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )
        self.declare_parameter(
            "visible_markers_topic", "/agv/sim/ground_truth/visible_markers")
        self.declare_parameter(
            "camera_info_topic", "/agv/zed/left/camera_info")
        self.declare_parameter("detections_topic", "/agv/detections")
        self.declare_parameter(
            "image_frame", "zed_left_camera_frame_optical")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("default_tag_size_m", 0.2)
        self.declare_parameter("tf_lookup_timeout_s", 0.2)

        self._image_frame = self.get_parameter("image_frame").value
        self._world_frame = self.get_parameter("world_frame").value
        self._tag_size = float(self.get_parameter("default_tag_size_m").value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_s").value))

        self._cam_info: Optional[CameraInfo] = None
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        self._warned_no_pose = False

        rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_cam_info, rel,
        )
        self.create_subscription(
            String,
            self.get_parameter("visible_markers_topic").value,
            self._on_markers, rel,
        )
        self._pub = self.create_publisher(
            AprilTagDetectionArray,
            self.get_parameter("detections_topic").value, rel,
        )
        self.get_logger().info(
            f"apriltag_sim_shim up: "
            f"{self.get_parameter('visible_markers_topic').value} → "
            f"{self.get_parameter('detections_topic').value} "
            f"(cam_frame={self._image_frame}, world_frame={self._world_frame}, "
            f"tag_size={self._tag_size:.3f} m)")

    def _on_cam_info(self, msg: CameraInfo) -> None:
        if self._cam_info is None and msg.k[0] > 0:
            self._cam_info = msg
            self.get_logger().info(
                f"camera intrinsics received: "
                f"fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} "
                f"cx={msg.k[2]:.1f} cy={msg.k[5]:.1f} "
                f"size={msg.width}x{msg.height}")

    def _lookup_world_to_cam(self) -> Optional[np.ndarray]:
        try:
            # Latest-available — sim_time is already synced via use_sim_time.
            ts = self._tf_buf.lookup_transform(
                self._image_frame, self._world_frame,
                rclpy.time.Time(), self._tf_timeout)
        except tf2_ros.TransformException:
            return None
        return _transform_to_matrix(ts)

    def _on_markers(self, msg: String) -> None:
        if self._cam_info is None:
            return
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        markers = data.get("visible") or data.get("markers") or []
        if not markers:
            return

        M_world_to_cam = self._lookup_world_to_cam()
        if M_world_to_cam is None:
            return

        fx, fy = self._cam_info.k[0], self._cam_info.k[4]
        cx, cy = self._cam_info.k[2], self._cam_info.k[5]
        w_px, h_px = int(self._cam_info.width), int(self._cam_info.height)
        half = self._tag_size / 2.0

        # Local tag corners on the Z=0 plane. ApritlTag convention is
        # bottom-left, bottom-right, top-right, top-left (CCW when viewed
        # from +Z), matching apriltag_ros's Corners order.
        local_corners = np.array([
            [-half, -half, 0.0, 1.0],
            [ half, -half, 0.0, 1.0],
            [ half,  half, 0.0, 1.0],
            [-half,  half, 0.0, 1.0],
        ]).T  # shape 4x4

        out = AprilTagDetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._image_frame

        for m in markers:
            if not isinstance(m, dict):
                continue
            tid = m.get("id")
            pose = m.get("tag_world_pose")
            if tid is None or pose is None:
                if not self._warned_no_pose:
                    self.get_logger().warn(
                        "visible_markers missing tag_world_pose; "
                        "sim likely pre-dd2cf06. Shim returns no detections.")
                    self._warned_no_pose = True
                continue

            M_tag_world = np.eye(4)
            M_tag_world[:3, :3] = _quat_to_matrix(
                float(pose.get("qx", 0.0)), float(pose.get("qy", 0.0)),
                float(pose.get("qz", 0.0)), float(pose.get("qw", 1.0)))
            M_tag_world[0, 3] = float(pose.get("x", 0.0))
            M_tag_world[1, 3] = float(pose.get("y", 0.0))
            M_tag_world[2, 3] = float(pose.get("z", 0.0))

            world_corners = M_tag_world @ local_corners
            cam_corners = M_world_to_cam @ world_corners  # 4x4

            pixels = []
            all_in_frame = True
            for i in range(4):
                Xc, Yc, Zc = cam_corners[0, i], cam_corners[1, i], cam_corners[2, i]
                if Zc <= 0.01:
                    # Tag corner behind camera plane — drop whole tag.
                    all_in_frame = False
                    break
                u = fx * Xc / Zc + cx
                v = fy * Yc / Zc + cy
                # Allow modest overshoot (~1 % per side) so tags near the
                # edge are still reported; apriltag_ros accepts the same.
                margin = 0.02
                if u < -margin * w_px or u > (1.0 + margin) * w_px:
                    all_in_frame = False
                    break
                if v < -margin * h_px or v > (1.0 + margin) * h_px:
                    all_in_frame = False
                    break
                pixels.append((u, v))
            if not all_in_frame or len(pixels) != 4:
                continue

            det = AprilTagDetection()
            det.family = "tag36h11"
            det.id = int(tid)
            det.hamming = 0
            det.goodness = 1.0
            det.decision_margin = 100.0
            det.centre.x = sum(p[0] for p in pixels) / 4.0
            det.centre.y = sum(p[1] for p in pixels) / 4.0
            for i, (u, v) in enumerate(pixels):
                det.corners[i].x = u
                det.corners[i].y = v
            # Homography is not used by rail_approach / marker_correction
            # (they only consume corners + id). Leave as zeros.
            out.detections.append(det)

        if out.detections:
            self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = AprilTagSimShim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
