#!/usr/bin/env python3
"""apriltag_sim_shim — HIL-only AprilTag detection synthesizer.

Why this exists
---------------
`apriltag_ros`'s `image_transport`-based subscription fails to deliver
images across the USB-eth CycloneDDS hop (observed Round 44 iter-1..4:
subscription registered, "Image messages received: 0" even though rclpy
raw subscriptions on the same topic see ~20 Hz). Without detections,
`marker_correction` + `rail_approach` cannot localize against floor
tags → every `rail_approach` waypoint times out at start pose.

Design (iter-6 rewrite, self-sufficient)
----------------------------------------
iter-5 relied on the sim's `/agv/sim/ground_truth/visible_markers`
oracle. That worked for wall tags but the sim filters out **floor
tags** (z=0) because the viewing incidence angle against their
straight-up normal exceeds the oracle's threshold — and every target
for `rail_approach` is a floor tag (ids 2, 3, 4, 12, 13, 33–37).

This version drops the oracle dependency entirely. At startup the
node loads `markers_registry.yaml` (same file marker_correction uses,
so the tag list is authoritative). At 5 Hz it reads the brain's TF
tree to learn the camera's current world pose and, for every
registered tag, checks visibility from geometry alone:

  1. transform the tag's 4 world corners into camera-optical frame,
  2. reject the tag if any corner is behind the image plane (Z ≤ 0.01 m),
  3. project each corner with the pinhole intrinsics from camera_info,
  4. reject the tag if any pixel is outside the image (±2 % margin), and
  5. reject the tag if its outward normal makes an angle > `max_incidence_deg`
     with the camera→tag direction (camera looking at the back of the tag).

Tags that pass all five gates are emitted as
`apriltag_msgs/AprilTagDetectionArray` on `/agv/detections` with the
same `id` / `corners[4]` / `centre` / `family` shape apriltag_ros would
produce — rail_approach runs its own `solvePnP` on the corners, so its
control loop is unchanged.

Tag orientation
---------------
The registry records only `yaw` (Z-axis rotation in world). The
geometric face direction depends on tag type, inferred from `z`:

  - **floor tag** (`z < 0.05 m`): plane = world XY. Normal = world +Z.
    Local corner axes = RotZ(yaw) applied to (1,0,0) and (0,1,0).
  - **wall tag** (`z ≥ 0.05 m`): plane = vertical. Normal points
    outward horizontally; the registry's `yaw` sweeps the normal in
    world XY, so normal = (cos(yaw), sin(yaw), 0). Local `right` axis
    is perpendicular to both normal and world +Z = (−sin(yaw),
     cos(yaw), 0) rotated; `up` axis = world +Z.

Both cases share the same frame convention apriltag_ros uses:
corners indexed bottom-left, bottom-right, top-right, top-left (CCW
viewed from the outward face), tag face in the local XY plane.

IMPORTANT: this is HIL-only. Real deployments keep apriltag_ros on a
working ZED stream. Gated via `agv_hil_full.launch.py` + TASK.yaml
`dev_only: true`.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
import yaml
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


FLOOR_Z_THRESHOLD = 0.05  # tags below this z-height are treated as floor.


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
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
    t = ts.transform.translation
    q = ts.transform.rotation
    M = np.eye(4)
    M[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
    M[0, 3], M[1, 3], M[2, 3] = t.x, t.y, t.z
    return M


class TagGeometry:
    """Precomputed world-frame corner + normal for one registered tag."""

    __slots__ = ("id", "corners_world", "normal_world")

    def __init__(self, tag_id: int, corners_world: np.ndarray,
                 normal_world: np.ndarray) -> None:
        self.id = tag_id
        # 4x4 homogeneous coordinates (columns), corners order BL, BR, TR, TL.
        self.corners_world = corners_world
        # Unit normal in world frame.
        self.normal_world = normal_world


def _build_tag_geometry(tag_id: int, x: float, y: float, z: float,
                        yaw: float, tag_size: float) -> TagGeometry:
    half = tag_size / 2.0
    cy, sy = math.cos(yaw), math.sin(yaw)
    if z < FLOOR_Z_THRESHOLD:
        # Floor tag: face up (+Z). Local axes rotated about Z by `yaw`.
        # Corners lie in the world XY plane at the tag's (x, y, z).
        local_right = np.array([cy, sy, 0.0])
        local_up = np.array([-sy, cy, 0.0])
        normal = np.array([0.0, 0.0, 1.0])
    else:
        # Wall tag: normal swept in world XY by `yaw`. Up direction is
        # world +Z. `right` perpendicular to both, preserving the
        # BL/BR/TR/TL CCW order when viewed from the outward face.
        normal = np.array([cy, sy, 0.0])
        local_up = np.array([0.0, 0.0, 1.0])
        # `right = up × normal` so (right, up, normal) is right-handed.
        local_right = np.cross(local_up, normal)
    centre = np.array([x, y, z])
    corners = np.zeros((4, 4))
    # BL, BR, TR, TL
    offsets = [(-half, -half), (half, -half), (half, half), (-half, half)]
    for i, (u, v) in enumerate(offsets):
        pt = centre + u * local_right + v * local_up
        corners[0, i] = pt[0]
        corners[1, i] = pt[1]
        corners[2, i] = pt[2]
        corners[3, i] = 1.0
    return TagGeometry(tag_id, corners, normal)


def _load_registry(path: Path, default_tag_size: float) -> list[TagGeometry]:
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return []
    raw = data.get("markers", [])
    tags: list[TagGeometry] = []
    for entry in raw:
        if not isinstance(entry, dict):
            continue
        try:
            tag_id = int(entry["id"])
            x = float(entry["x"])
            y = float(entry["y"])
            z = float(entry["z"])
            yaw = float(entry.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        tag_size = float(entry.get("size", default_tag_size))
        tags.append(_build_tag_geometry(tag_id, x, y, z, yaw, tag_size))
    return tags


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
            "camera_info_topic", "/agv/zed/left/camera_info")
        self.declare_parameter("detections_topic", "/agv/detections")
        self.declare_parameter(
            "image_frame", "zed_left_camera_frame_optical")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("registry_file", "")
        self.declare_parameter("default_tag_size_m", 0.2)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("max_incidence_deg", 85.0)
        self.declare_parameter("image_margin_frac", 0.02)
        self.declare_parameter("tf_lookup_timeout_s", 0.2)
        self.declare_parameter("family", "tag36h11")

        self._image_frame = self.get_parameter("image_frame").value
        self._world_frame = self.get_parameter("world_frame").value
        self._default_tag_size = float(
            self.get_parameter("default_tag_size_m").value)
        self._max_cos_offaxis = math.cos(math.radians(
            float(self.get_parameter("max_incidence_deg").value)))
        self._image_margin = float(
            self.get_parameter("image_margin_frac").value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_s").value))
        self._family = str(self.get_parameter("family").value)

        registry_path_s = str(self.get_parameter("registry_file").value)
        if not registry_path_s:
            self.get_logger().error(
                "registry_file parameter required — set it to the absolute "
                "path of markers_registry.yaml at launch.")
            raise RuntimeError("registry_file unset")
        registry_path = Path(registry_path_s)
        self._tags = _load_registry(registry_path, self._default_tag_size)
        if not self._tags:
            self.get_logger().error(
                f"no markers loaded from {registry_path}; detections would "
                f"always be empty")
        else:
            floor = sum(
                1 for t in self._tags
                if t.corners_world[2, 0] < FLOOR_Z_THRESHOLD)
            wall = len(self._tags) - floor
            self.get_logger().info(
                f"loaded {len(self._tags)} tags from {registry_path.name} "
                f"(floor={floor}, wall={wall})")

        self._cam_info: Optional[CameraInfo] = None
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

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
        self._pub = self.create_publisher(
            AprilTagDetectionArray,
            self.get_parameter("detections_topic").value, rel,
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(
            f"apriltag_sim_shim up: registry-driven projector "
            f"(cam_frame={self._image_frame}, world_frame={self._world_frame}, "
            f"rate={rate:.1f} Hz, "
            f"max_incidence={math.degrees(math.acos(self._max_cos_offaxis)):.0f}°)")

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
            ts = self._tf_buf.lookup_transform(
                self._image_frame, self._world_frame,
                rclpy.time.Time(), self._tf_timeout)
        except tf2_ros.TransformException:
            return None
        return _transform_to_matrix(ts)

    def _on_tick(self) -> None:
        if self._cam_info is None or not self._tags:
            return
        M_world_to_cam = self._lookup_world_to_cam()
        if M_world_to_cam is None:
            return
        # Camera position in world = -R_cw^T t_cw. We use it for the
        # incidence check against each tag's normal.
        R_cw = M_world_to_cam[:3, :3]
        t_cw = M_world_to_cam[:3, 3]
        cam_pos_world = -R_cw.T @ t_cw

        fx, fy = self._cam_info.k[0], self._cam_info.k[4]
        cx, cy = self._cam_info.k[2], self._cam_info.k[5]
        w_px, h_px = int(self._cam_info.width), int(self._cam_info.height)
        m_w = self._image_margin * w_px
        m_h = self._image_margin * h_px

        out = AprilTagDetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._image_frame

        for tg in self._tags:
            # Incidence test: camera should be on the outward side.
            tag_centre = np.array([
                tg.corners_world[0, :].mean(),
                tg.corners_world[1, :].mean(),
                tg.corners_world[2, :].mean(),
            ])
            view = cam_pos_world - tag_centre
            view_n = np.linalg.norm(view)
            if view_n < 1e-6:
                continue
            view /= view_n
            cos_incidence = float(np.dot(tg.normal_world, view))
            if cos_incidence < self._max_cos_offaxis:
                # Camera is too oblique to the tag, or looking at the back.
                continue

            cam_corners = M_world_to_cam @ tg.corners_world  # 4x4

            pixels = []
            in_frame = True
            for i in range(4):
                Xc, Yc, Zc = cam_corners[0, i], cam_corners[1, i], cam_corners[2, i]
                if Zc <= 0.01:
                    in_frame = False
                    break
                u = fx * Xc / Zc + cx
                v = fy * Yc / Zc + cy
                if u < -m_w or u > w_px + m_w:
                    in_frame = False
                    break
                if v < -m_h or v > h_px + m_h:
                    in_frame = False
                    break
                pixels.append((u, v))
            if not in_frame or len(pixels) != 4:
                continue

            det = AprilTagDetection()
            det.family = self._family
            det.id = int(tg.id)
            det.hamming = 0
            det.goodness = 1.0
            det.decision_margin = 100.0
            det.centre.x = sum(p[0] for p in pixels) / 4.0
            det.centre.y = sum(p[1] for p in pixels) / 4.0
            for i, (u, v) in enumerate(pixels):
                det.corners[i].x = u
                det.corners[i].y = v
            out.detections.append(det)

        if out.detections:
            self._pub.publish(out)


def main() -> None:
    rclpy.init()
    try:
        node = AprilTagSimShim()
    except RuntimeError:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
