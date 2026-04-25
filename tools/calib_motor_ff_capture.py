#!/usr/bin/env python3
"""Auto-watch motion characterization (operator drives, script measures).

The operator drives the robot freely with the dashboard joystick. This
script watches /agv/wheel_odom for motion and captures Δtag (AprilTag)
vs Δodom (wheel encoders) for each "leg" (motion-then-stop cycle).

Flow per leg:
  IDLE        → robot still: keep an updated reservoir of recent tag poses
  MOVING      → |wheel velocity| > velocity_threshold sustained
                 (use the most recent reservoir as PRE-pose)
  SETTLING    → velocity dropped below threshold, wait 1.0 s
  CAPTURED    → take fresh median POST-pose; print delta; back to IDLE

Stop the script with Ctrl+C. The CSV is written on exit.

Usage (no operator prompts, no cmd_vel publishing):
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///tmp/agv_cyclonedds_runtime.xml
  python3 tools/calib_motor_ff_capture.py
"""
from __future__ import annotations

import argparse
import collections
import csv
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry


def make_object_points(tag_size: float) -> np.ndarray:
    h = tag_size / 2.0
    return np.array(
        [[-h, -h, 0.0], [+h, -h, 0.0], [+h, +h, 0.0], [-h, +h, 0.0]],
        dtype=np.float64,
    )


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


@dataclass
class TagSample:
    t: float
    tx: float
    ty: float
    tz: float


@dataclass
class OdomSample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float


@dataclass
class LegRow:
    leg_idx: int
    dt_s: float
    dtag_x_cam: float
    dtag_y_cam: float
    dtag_z_cam: float
    forward_proxy: float       # = -dtag_z_cam (robot-forward axis ≈ camera optical Z)
    dodom_x: float
    dodom_y: float
    dodom_yaw: float
    dodom_distance: float
    error_distance: float
    n_pre: int
    n_post: int
    peak_vx: float


class CaptureNode(Node):
    def __init__(self, tag_id: int, tag_size: float):
        super().__init__("calib_motor_ff_capture")
        self.tag_id = tag_id
        self.tag_size = tag_size
        self.obj_pts = make_object_points(tag_size)

        self.K: np.ndarray | None = None
        self.dist: np.ndarray | None = None
        self.tag_samples: collections.deque[TagSample] = collections.deque(maxlen=900)
        self.odom_samples: collections.deque[OdomSample] = collections.deque(maxlen=600)

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        rel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(CameraInfo, "/agv/zed/left/camera_info",
                                 self.on_camera_info, rel_qos)
        self.create_subscription(AprilTagDetectionArray, "/agv/detections",
                                 self.on_detections, be_qos)
        self.create_subscription(Odometry, "/agv/wheel_odom",
                                 self.on_odom, be_qos)

    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5)
            self.get_logger().info(
                f"camera_info: {msg.width}x{msg.height}, fx={self.K[0,0]:.2f}"
            )

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        if self.K is None:
            return
        for det in msg.detections:
            if det.id != self.tag_id:
                continue
            corners_px = np.array([[c.x, c.y] for c in det.corners], dtype=np.float64)
            if corners_px.shape != (4, 2):
                continue
            try:
                ok, _rvec, tvec = cv2.solvePnP(
                    self.obj_pts, corners_px, self.K, self.dist,
                    flags=cv2.SOLVEPNP_SQPNP,
                )
            except cv2.error:
                continue
            if not ok:
                continue
            tvec = tvec.flatten()
            self.tag_samples.append(TagSample(
                t=time.time(),
                tx=float(tvec[0]), ty=float(tvec[1]), tz=float(tvec[2]),
            ))

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_samples.append(OdomSample(
            t=time.time(),
            x=float(p.x), y=float(p.y),
            yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
            vx=float(msg.twist.twist.linear.x),
        ))

    # ── Helpers ──────────────────────────────────────────────────────────
    def hits_in_window(self, window_s: float) -> int:
        cutoff = time.time() - window_s
        return sum(1 for s in self.tag_samples if s.t >= cutoff)

    def median_tag_in_window(self, window_s: float) -> tuple[TagSample, int] | None:
        cutoff = time.time() - window_s
        chosen = [s for s in self.tag_samples if s.t >= cutoff]
        if len(chosen) < 5:
            return None
        return TagSample(
            t=chosen[-1].t,
            tx=statistics.median(s.tx for s in chosen),
            ty=statistics.median(s.ty for s in chosen),
            tz=statistics.median(s.tz for s in chosen),
        ), len(chosen)

    def odom_at_or_before(self, t_target: float) -> OdomSample | None:
        # Pick the most recent odom sample with t <= t_target
        best = None
        for s in self.odom_samples:
            if s.t <= t_target:
                if best is None or s.t > best.t:
                    best = s
            else:
                break  # samples are time-ordered
        return best

    def latest_odom(self) -> OdomSample | None:
        return self.odom_samples[-1] if self.odom_samples else None

    def is_moving(self, threshold_mps: float, window_s: float = 0.3) -> bool:
        """Robot is moving if any wheel_odom sample in the last window
        exceeded the threshold. We don't average — if at least one
        sample saw motion, we believe we're moving."""
        cutoff = time.time() - window_s
        for s in reversed(self.odom_samples):
            if s.t < cutoff:
                break
            if abs(s.vx) >= threshold_mps:
                return True
        return False

    def peak_vx_since(self, t0: float) -> float:
        peak = 0.0
        for s in self.odom_samples:
            if s.t < t0:
                continue
            if abs(s.vx) > abs(peak):
                peak = s.vx
        return peak


def fmt_tag(s: TagSample | None) -> str:
    if s is None:
        return "(none)"
    return f"({s.tx:+.3f},{s.ty:+.3f},{s.tz:+.3f})"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--id", type=int, default=12)
    parser.add_argument("--tag-size", type=float, default=0.20)
    parser.add_argument("--velocity-threshold", type=float, default=0.02,
                        help="m/s; below this we consider robot still")
    parser.add_argument("--settle-s", type=float, default=1.0,
                        help="seconds of stillness before capturing post-pose")
    parser.add_argument("--min-leg-distance", type=float, default=0.005,
                        help="discard legs where forward_proxy < this (m)")
    parser.add_argument("--out-dir", type=str,
                        default=str(Path(__file__).parent / "calib_runs"),
                        help="directory to write CSV (default: tools/calib_runs)")
    args = parser.parse_args(argv)

    rclpy.init()
    node = CaptureNode(args.id, args.tag_size)

    # Wait for camera_info + first detection
    print(f"Esperando detección del tag id={args.id} y wheel_odom…")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.K is not None
                and node.hits_in_window(2.0) >= 5
                and node.latest_odom() is not None):
            break
    if node.K is None or node.hits_in_window(2.0) < 5 or node.latest_odom() is None:
        print("ERROR: no hay detección estable o no llega wheel_odom.")
        node.destroy_node(); rclpy.shutdown()
        return 1

    print("OK. Mueve el robot con el joystick cuando quieras. "
          "Cada arranque-y-paro genera una leg.")
    print("Ctrl+C para terminar y guardar el CSV.\n")

    rows: list[LegRow] = []
    leg_idx = 0
    state = "IDLE"
    pre_tag: TagSample | None = None
    pre_n = 0
    pre_odom: OdomSample | None = None
    motion_start_t = 0.0
    last_motion_t = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            moving = node.is_moving(args.velocity_threshold)

            if state == "IDLE":
                # Continuously refresh the candidate pre-pose using the
                # most recent ~1 s of stillness. When motion starts, this
                # window is taken as PRE.
                if moving:
                    # Take pre-pose from JUST BEFORE motion started.
                    pre = node.median_tag_in_window(window_s=1.5)
                    pre_odom_candidate = node.odom_at_or_before(now - 0.1)
                    if pre is None or pre_odom_candidate is None:
                        # No usable pre — skip this motion event.
                        print("    (motion, but no clean pre-pose; ignoring)")
                        # Wait for it to stop again before resuming IDLE
                        state = "MOVING_NO_PRE"
                        last_motion_t = now
                    else:
                        pre_tag, pre_n = pre
                        pre_odom = pre_odom_candidate
                        motion_start_t = now
                        last_motion_t = now
                        leg_idx += 1
                        print(f"── Leg {leg_idx} ── PRE tag={fmt_tag(pre_tag)} "
                              f"odom=({pre_odom.x:+.3f},{pre_odom.y:+.3f}) "
                              f"[{pre_n} samples]")
                        state = "MOVING"
            elif state == "MOVING":
                if moving:
                    last_motion_t = now
                else:
                    if now - last_motion_t >= args.settle_s:
                        # Capture post
                        post = node.median_tag_in_window(window_s=args.settle_s)
                        post_odom = node.latest_odom()
                        if post is None or post_odom is None:
                            print(f"  Leg {leg_idx}: post-pose no disponible. Descartada.")
                            state = "IDLE"; continue
                        post_tag, post_n = post
                        peak = node.peak_vx_since(motion_start_t)
                        # Compute deltas
                        dtag_x = post_tag.tx - pre_tag.tx
                        dtag_y = post_tag.ty - pre_tag.ty
                        dtag_z = post_tag.tz - pre_tag.tz
                        forward_proxy = -dtag_z
                        dodom_x = post_odom.x - pre_odom.x
                        dodom_y = post_odom.y - pre_odom.y
                        dodom_yaw = post_odom.yaw - pre_odom.yaw
                        dodom_distance = math.hypot(dodom_x, dodom_y)
                        if dodom_x < 0:
                            dodom_distance = -dodom_distance
                        error = forward_proxy - dodom_distance
                        dt_s = post_odom.t - pre_odom.t

                        if abs(forward_proxy) < args.min_leg_distance:
                            print(f"  Leg {leg_idx}: |Δtag|={abs(forward_proxy)*1000:.1f}mm "
                                  f"< {args.min_leg_distance*1000:.0f}mm umbral, descartada.\n")
                            state = "IDLE"; continue

                        rows.append(LegRow(
                            leg_idx=leg_idx, dt_s=dt_s,
                            dtag_x_cam=dtag_x, dtag_y_cam=dtag_y, dtag_z_cam=dtag_z,
                            forward_proxy=forward_proxy,
                            dodom_x=dodom_x, dodom_y=dodom_y, dodom_yaw=dodom_yaw,
                            dodom_distance=dodom_distance, error_distance=error,
                            n_pre=pre_n, n_post=post_n, peak_vx=peak,
                        ))
                        ratio_str = ""
                        if abs(forward_proxy) > 5e-3:
                            ratio = dodom_distance / forward_proxy
                            ratio_str = f", Δodom/Δtag={ratio:+.3f} ({(ratio-1)*100:+.1f}%)"
                        print(
                            f"  Leg {leg_idx} POST tag={fmt_tag(post_tag)} "
                            f"[{post_n} samples] peak={peak:+.3f} m/s\n"
                            f"             Δtag(fwd)={forward_proxy*100:+.2f} cm, "
                            f"Δodom={dodom_distance*100:+.2f} cm, "
                            f"err={error*100:+.2f} cm{ratio_str}, "
                            f"dt={dt_s:.2f} s\n"
                        )
                        state = "IDLE"
            elif state == "MOVING_NO_PRE":
                if not moving and now - last_motion_t >= args.settle_s:
                    state = "IDLE"
    except KeyboardInterrupt:
        print("\nInterrumpido.")
    finally:
        Path(args.out_dir).mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%dT%H%M%S")
        csv_path = Path(args.out_dir) / f"capture_{ts}.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "leg", "dt_s",
                "dtag_x_cam", "dtag_y_cam", "dtag_z_cam", "forward_proxy",
                "dodom_x", "dodom_y", "dodom_yaw", "dodom_distance",
                "error_distance", "peak_vx",
                "n_pre", "n_post",
            ])
            for r in rows:
                w.writerow([
                    r.leg_idx, f"{r.dt_s:.3f}",
                    f"{r.dtag_x_cam:.5f}", f"{r.dtag_y_cam:.5f}",
                    f"{r.dtag_z_cam:.5f}", f"{r.forward_proxy:.5f}",
                    f"{r.dodom_x:.5f}", f"{r.dodom_y:.5f}",
                    f"{r.dodom_yaw:.5f}", f"{r.dodom_distance:.5f}",
                    f"{r.error_distance:.5f}", f"{r.peak_vx:.4f}",
                    r.n_pre, r.n_post,
                ])
        print(f"CSV: {csv_path}")
        if rows:
            useful = [r for r in rows if abs(r.forward_proxy) > 5e-3]
            if useful:
                ratios = [r.dodom_distance / r.forward_proxy for r in useful]
                m = statistics.mean(ratios)
                s = statistics.stdev(ratios) if len(ratios) >= 2 else float("nan")
                print(f"Resumen: N={len(useful)} legs útiles, "
                      f"Δodom/Δtag mean={m:+.3f} (= {(m-1.0)*100:+.1f}% sesgo odom), "
                      f"std={s:.3f}")

        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
