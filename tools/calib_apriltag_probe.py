#!/usr/bin/env python3
"""AprilTag detection smoke-check for the FF calibration session.

Subscribes to /agv/detections (apriltag_msgs/AprilTagDetectionArray) and
/agv/zed/left/camera_info. For each detection of the configured id, runs
solvePnP locally with SQPNP — same algorithm as
src/agv_markers/src/marker_correction_node.cpp:280-314 — and reports range
plus tvec in the camera optical frame.

Output is intentionally close-loop friendly: prints a one-line status every
second so a human watching the terminal can confirm detection stability
before kicking off the experiment.

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///tmp/agv_cyclonedds_runtime.xml
  python3 tools/calib_apriltag_probe.py --duration 5

Exit codes:
  0 — stable detection (>= 10 hits in any 3-second window)
  1 — no usable camera_info or no detections
  2 — degraded detection (< 10 hits in any 3-second window during run)
"""
from __future__ import annotations

import argparse
import collections
import sys
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo


# Object points: a tag_size × tag_size square centered on tag origin (z=0).
# Same convention as marker_correction_node.cpp:280-289.
def make_object_points(tag_size: float) -> np.ndarray:
    h = tag_size / 2.0
    return np.array(
        [
            [-h, -h, 0.0],
            [+h, -h, 0.0],
            [+h, +h, 0.0],
            [-h, +h, 0.0],
        ],
        dtype=np.float64,
    )


class ProbeNode(Node):
    def __init__(self, tag_id: int, tag_size: float):
        super().__init__("calib_apriltag_probe")

        self.tag_id = tag_id
        self.tag_size = tag_size
        self.obj_pts = make_object_points(tag_size)

        # State
        self.K: np.ndarray | None = None
        self.dist: np.ndarray | None = None
        self.image_size: tuple[int, int] | None = None  # (width, height)
        self.last_detection_t: float = 0.0
        self.hit_times: collections.deque[float] = collections.deque(maxlen=200)
        self.last_pose: tuple[float, float, float] | None = None
        self.last_decision_margin: float = 0.0
        self.last_hamming: int = 0

        # Publishers/subscribers — match QoS used by ZED + apriltag_node:
        # SensorDataQoS = best_effort + keep_last(5). camera_info is RELIABLE
        # by ZED wrapper convention; we use a permissive QoS to accept either.
        camera_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            CameraInfo,
            "/agv/zed/left/camera_info",
            self.on_camera_info,
            camera_info_qos,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            "/agv/detections",
            self.on_detections,
            det_qos,
        )

        self.create_timer(1.0, self.print_status)

        self.get_logger().info(
            f"Probe armed: id={tag_id}, tag_size={tag_size:.3f} m. "
            f"Waiting for /agv/zed/left/camera_info and /agv/detections..."
        )

    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.K is None:
            k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            d = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5)
            self.K = k
            self.dist = d
            self.image_size = (msg.width, msg.height)
            self.get_logger().info(
                f"camera_info captured: {msg.width}x{msg.height}, "
                f"fx={k[0,0]:.2f} fy={k[1,1]:.2f} cx={k[0,2]:.2f} cy={k[1,2]:.2f}"
            )

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        if self.K is None:
            return
        for det in msg.detections:
            if det.id != self.tag_id:
                continue
            corners_px = np.array(
                [[c.x, c.y] for c in det.corners], dtype=np.float64
            )
            if corners_px.shape != (4, 2):
                continue
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_pts,
                    corners_px,
                    self.K,
                    self.dist,
                    flags=cv2.SOLVEPNP_SQPNP,
                )
            except cv2.error:
                continue
            if not ok:
                continue
            tvec = tvec.flatten()
            now = time.time()
            self.hit_times.append(now)
            self.last_detection_t = now
            self.last_pose = (float(tvec[0]), float(tvec[1]), float(tvec[2]))
            self.last_decision_margin = float(det.decision_margin)
            self.last_hamming = int(det.hamming)

    def hits_in_window(self, window_s: float) -> int:
        cutoff = time.time() - window_s
        # deque is monotonic-ish; iterate from right
        return sum(1 for t in self.hit_times if t >= cutoff)

    def print_status(self) -> None:
        if self.K is None:
            self.get_logger().warn("Still waiting for camera_info...")
            return
        hits_3s = self.hits_in_window(3.0)
        if self.last_pose is None:
            self.get_logger().warn(
                f"No detections of id={self.tag_id} yet. hits/3s=0"
            )
            return
        tx, ty, tz = self.last_pose
        rng = (tx**2 + ty**2 + tz**2) ** 0.5
        age_s = time.time() - self.last_detection_t
        marker = "OK " if hits_3s >= 10 else "DEG"
        self.get_logger().info(
            f"[{marker}] id={self.tag_id}  range={rng:.3f} m  "
            f"tvec_optical=(x={tx:+.3f}, y={ty:+.3f}, z={tz:+.3f}) m  "
            f"margin={self.last_decision_margin:.0f}  "
            f"hamming={self.last_hamming}  "
            f"hits/3s={hits_3s}  age={age_s:.2f}s"
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--id", type=int, default=12, help="AprilTag id to track (default: 12)")
    parser.add_argument(
        "--tag-size",
        type=float,
        default=0.20,
        help="Tag side length in meters, including the white border per "
        "apriltag_node convention (default: 0.20)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Run for this many seconds then exit. 0 = run until Ctrl+C.",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = ProbeNode(args.id, args.tag_size)

    deadline = None if args.duration <= 0.0 else time.time() + args.duration
    peak_hits = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if deadline is not None and time.time() >= deadline:
                break
            peak_hits = max(peak_hits, node.hits_in_window(3.0))
    except KeyboardInterrupt:
        pass
    finally:
        # Final summary
        if node.K is None:
            node.get_logger().error(
                "camera_info never received — is the ZED wrapper publishing? "
                "Check /agv/zed/left/camera_info with `ros2 topic hz`."
            )
            rc = 1
        elif node.last_pose is None:
            node.get_logger().error(
                f"No detections of id={args.id} during the run. "
                f"Check /agv/detections, lighting, and tag visibility."
            )
            rc = 1
        else:
            tx, ty, tz = node.last_pose
            rng = (tx**2 + ty**2 + tz**2) ** 0.5
            node.get_logger().info(
                f"Final: range={rng:.3f} m, peak hits/3s={peak_hits}, "
                f"decision_margin={node.last_decision_margin:.0f}"
            )
            rc = 0 if peak_hits >= 10 else 2

        node.destroy_node()
        rclpy.shutdown()
        return rc


if __name__ == "__main__":
    sys.exit(main())
