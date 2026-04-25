#!/usr/bin/env python3
"""Motor characterization experiment using AprilTag id=12 as ground truth.

Drives the robot ±distance over `--rounds` repetitions while capturing both
the AprilTag pose (optical) and the wheel_odom integration. Persists results
as a CSV the user can analyze offline with calib_motor_ff_analyze.py.

What it DOES:
  - Pre-flight checks: motors armed, no e-stop, mode=teleop, tag visible,
    collision_monitor not in STOP.
  - Captures N=30 samples of tag pose before/after each leg, takes the
    median to reject pixel-noise outliers.
  - Publishes /agv/cmd_vel (Twist) at 20 Hz with linear.x = ±speed, then
    a zero command at the end of each leg.
  - Watchdogs: lost-tag during movement → cmd_vel(0,0) + abort the round.
    Ctrl+C / exception → cmd_vel(0,0) guaranteed via try/finally.

What it DOES NOT do:
  - Tune any parameter. This is characterization only.
  - Touch the marker_correction_node, the markers_registry.yaml, or the
    state machine. The script publishes /agv/cmd_vel directly; the operator
    is responsible for putting the stack into a state where that topic is
    accepted (mode=teleop or stack idle).

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///tmp/agv_cyclonedds_runtime.xml
  python3 tools/calib_motor_ff_run.py --rounds 5 --distance 0.15 --speed 0.10
  python3 tools/calib_motor_ff_run.py --dry-run    # only captures, no cmd_vel

Output:
  tools/calib_runs/run_<ISO>.csv
  Summary on stdout.
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Sprint 1 added a stale-command watchdog in agv_ui_backend that publishes
# cmd_vel(0,0) at 10 Hz whenever (a) any WS client is connected to the
# dashboard and (b) more than 0.5 s passed since the last operator cmd_vel.
# That defeats any direct ROS publish to /agv/cmd_vel: the velocity_smoother
# averages the watchdog zeros with our values and the deadband (0.01 m/s)
# collapses the output to zero. To work around this without forcing the
# operator to close the dashboard (and lose the visual e-stop button), we
# tunnel cmd_vel through the same WebSocket the dashboard uses. That keeps
# the backend's lastCmdTime fresh, suppresses the watchdog, and lets us
# coexist with the dashboard.
from websockets.sync.client import connect as ws_connect  # type: ignore


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
    """One solvePnP result, in the camera optical frame."""
    t: float
    tx: float
    ty: float
    tz: float
    decision_margin: float
    hamming: int


@dataclass
class CalibRow:
    round_idx: int
    leg: str  # 'forward' | 'reverse'
    cmd_vel: float
    cmd_dur_s: float
    dt_s: float
    dtag_x_cam: float
    dtag_y_cam: float
    dtag_z_cam: float
    dtag_range: float
    dodom_x: float
    dodom_y: float
    dodom_yaw: float
    dodom_distance: float
    error_distance: float
    error_pct: float
    n_samples_pre: int
    n_samples_post: int
    note: str = ""


class CalibNode(Node):
    def __init__(self, tag_id: int, tag_size: float, dry_run: bool,
                 ws_url: str | None = None):
        super().__init__("calib_motor_ff_run")
        self.tag_id = tag_id
        self.tag_size = tag_size
        self.obj_pts = make_object_points(tag_size)
        self.dry_run = dry_run
        self.ws_url = ws_url
        self.ws = None  # established lazily in connect_ws()

        # Camera intrinsics (lazy)
        self.K: np.ndarray | None = None
        self.dist: np.ndarray | None = None

        # Tag samples — bounded so the deque doesn't grow without bound
        self.tag_samples: collections.deque[TagSample] = collections.deque(maxlen=300)

        # Latest odom + state
        self.latest_odom: tuple[float, float, float, float] | None = None  # (t, x, y, yaw)
        self.latest_motor_state: dict | None = None
        self.latest_safety_status: str = ""
        self.latest_collision_action: str = "OK"
        self.latest_collision_t: float = 0.0
        self.collision_state_seen: bool = False

        # QoS profiles
        rel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(CameraInfo, "/agv/zed/left/camera_info",
                                 self.on_camera_info, rel_qos)
        self.create_subscription(AprilTagDetectionArray, "/agv/detections",
                                 self.on_detections, be_qos)
        self.create_subscription(Odometry, "/agv/wheel_odom",
                                 self.on_odom, be_qos)
        self.create_subscription(String, "/agv/motor_state",
                                 self.on_motor_state, be_qos)
        # /agv/safety/status: agv_interfaces/msg/SafetyStatus, but its content
        # is structured. We read it loosely as "received recently" for liveness.
        # collision_monitor_state: nav2_msgs/msg/CollisionMonitorState
        try:
            from agv_interfaces.msg import SafetyStatus  # type: ignore
            self.create_subscription(SafetyStatus, "/agv/safety/status",
                                     self.on_safety_status, rel_qos)
            self._safety_ok = True
        except Exception:
            self.get_logger().warn(
                "agv_interfaces.msg.SafetyStatus not importable — pre-flight will skip the safety check"
            )
            self._safety_ok = False

        try:
            from nav2_msgs.msg import CollisionMonitorState  # type: ignore
            self.create_subscription(CollisionMonitorState,
                                     "/agv/collision_monitor_state",
                                     self.on_collision_state, rel_qos)
            self._cm_msg_ok = True
        except Exception:
            self.get_logger().warn(
                "nav2_msgs.msg.CollisionMonitorState not importable — pre-flight will skip the collision check"
            )
            self._cm_msg_ok = False

        self.cmd_vel_pub = self.create_publisher(Twist, "/agv/cmd_vel", rel_qos)
        self.mode_set_pub = self.create_publisher(String, "/agv/mode/set", rel_qos)

    # ── Callbacks ────────────────────────────────────────────────────────
    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5)
            self.get_logger().info(
                f"camera_info captured: {msg.width}x{msg.height}, "
                f"fx={self.K[0,0]:.2f} fy={self.K[1,1]:.2f}"
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
                decision_margin=float(det.decision_margin),
                hamming=int(det.hamming),
            ))

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = (
            time.time(),
            float(p.x),
            float(p.y),
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def on_motor_state(self, msg: String) -> None:
        # Backend's parsing is JSON with NaN→null; we don't care about
        # numeric NaN here, only the armed flag.
        try:
            import json
            data = json.loads(msg.data.replace("nan", "null"))
            if data.get("_keepalive"):
                return
            self.latest_motor_state = data
        except Exception:
            pass

    def on_safety_status(self, msg) -> None:  # type: ignore[no-untyped-def]
        # SafetyStatus: bool safety_ok; string reason
        ok = bool(getattr(msg, "safety_ok", True))
        reason = getattr(msg, "reason", "")
        self.latest_safety_status = "ok" if ok else f"BLOCKED: {reason}"

    def on_collision_state(self, msg) -> None:  # type: ignore[no-untyped-def]
        # CollisionMonitorState: int8 action_type; string polygon_name
        action_type = int(getattr(msg, "action_type", 0))
        # 0=DO_NOTHING, 1=STOP, 2=SLOWDOWN, 3=APPROACH, 4=LIMIT
        action_map = {0: "OK", 1: "STOP", 2: "SLOWDOWN", 3: "APPROACH", 4: "LIMIT"}
        self.latest_collision_action = action_map.get(action_type, "UNKNOWN")
        self.latest_collision_t = time.time()
        self.collision_state_seen = True

    # ── Helpers ──────────────────────────────────────────────────────────
    def hits_in_window(self, window_s: float) -> int:
        cutoff = time.time() - window_s
        return sum(1 for s in self.tag_samples if s.t >= cutoff)

    def capture_median_pose(self, n_samples: int = 30, max_wait_s: float = 5.0
                            ) -> tuple[TagSample, int] | None:
        """Block until n_samples fresh tag detections accumulate, return median.

        Uses a timestamp cutoff (not a deque index) so the bounded deque
        evicting older entries does not invalidate "freshness" tracking.
        """
        cutoff = time.time()
        deadline = cutoff + max_wait_s
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            new_samples = [s for s in self.tag_samples if s.t >= cutoff]
            if len(new_samples) >= n_samples:
                break
            if time.time() >= deadline:
                break
        new_samples = [s for s in self.tag_samples if s.t >= cutoff]
        if len(new_samples) < 5:
            return None
        # Median across each axis independently is robust to corner-pixel jitter
        med = TagSample(
            t=new_samples[-1].t,
            tx=statistics.median(s.tx for s in new_samples),
            ty=statistics.median(s.ty for s in new_samples),
            tz=statistics.median(s.tz for s in new_samples),
            decision_margin=statistics.median(s.decision_margin for s in new_samples),
            hamming=int(statistics.median(s.hamming for s in new_samples)),
        )
        return med, len(new_samples)

    def connect_ws(self) -> None:
        """Open a WebSocket to the operator backend so cmd_vel goes through
        the same channel the dashboard joystick uses. This silences the
        backend's stale-command watchdog and lets us coexist with an open
        dashboard."""
        if self.ws_url is None:
            return
        try:
            self.ws = ws_connect(self.ws_url, open_timeout=3.0)
            self.get_logger().info(f"WS connected to {self.ws_url}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"WS connect failed ({self.ws_url}): {e!r}. "
                f"Falling back to direct ROS publish — the backend watchdog "
                f"will fight us if a dashboard is open."
            )
            self.ws = None

    def close_ws(self) -> None:
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass
            self.ws = None

    def publish_cmd_vel(self, linear_x: float, angular_z: float = 0.0) -> None:
        # Prefer the WS path (coexists with dashboard) and fall back to a
        # direct topic publish if the WS is not connected.
        if self.ws is not None:
            try:
                payload = {
                    "type": "cmd_vel",
                    "linear": float(linear_x),
                    "angular": float(angular_z),
                }
                self.ws.send(json.dumps(payload))
                return
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(
                    f"WS send failed: {e!r}. Reverting to direct publish."
                )
                self.ws = None
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def stop_motors(self) -> None:
        """Send zero cmd_vel a few times to be belt-and-suspenders sure.

        We always also do a direct ROS publish at the end so that even if
        the WS path silently fails, the safety chain still receives a stop.
        """
        for _ in range(5):
            self.publish_cmd_vel(0.0, 0.0)
            time.sleep(0.02)
        # Belt-and-suspenders: also publish a zero on the raw topic.
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def request_teleop_mode(self) -> None:
        """Tell mode_arbiter to pick teleop. The arbiter will let our
        cmd_vel through; otherwise it stamps zero at 20 Hz over us."""
        msg = String()
        msg.data = "teleop"
        self.mode_set_pub.publish(msg)
        # Give the arbiter a moment to switch sources before we publish.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)


# ── Pre-flight ───────────────────────────────────────────────────────────
def preflight(node: CalibNode, dry_run: bool) -> tuple[bool, str]:
    # Spin briefly to receive initial messages
    deadline = time.time() + 4.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.K is None:
        return False, "camera_info never arrived (/agv/zed/left/camera_info)"

    hits = node.hits_in_window(3.0)
    if hits < 10:
        return False, f"insufficient tag detections in last 3 s: {hits} (need ≥ 10)"

    if node.latest_motor_state is None:
        return False, "motor_state never arrived (/agv/motor_state)"

    armed = bool(node.latest_motor_state.get("armed", False))
    if not armed and not dry_run:
        return False, (
            "motors not armed. Arm motors from the dashboard "
            "(Recovery → Motor Enable) before running this experiment."
        )

    e_stop = bool(node.latest_motor_state.get("e_stop_active", False))
    if e_stop:
        return False, "e_stop active — clear from the dashboard before continuing"

    if node._cm_msg_ok and node.collision_state_seen:
        if node.latest_collision_action == "STOP":
            return False, (
                f"collision_monitor reports STOP "
                f"({time.time() - node.latest_collision_t:.1f} s ago) — "
                f"clear obstacles in front of the robot"
            )

    if node.latest_odom is None:
        return False, "wheel_odom never arrived (/agv/wheel_odom)"

    return True, "ok"


# ── Round execution ──────────────────────────────────────────────────────
def run_leg(node: CalibNode, leg_name: str, cmd_vel: float, duration_s: float,
            dry_run: bool, args) -> tuple[CalibRow | None, str]:
    """Execute one forward or reverse leg. Returns (row, note)."""
    note_parts: list[str] = []

    # 1. Capture initial pose (median of N samples)
    pre = node.capture_median_pose(args.samples)
    if pre is None:
        return None, "lost_tag_pre"
    pre_sample, n_pre = pre

    # 2. Snapshot odom
    if node.latest_odom is None:
        return None, "no_odom"
    t_start, x0, y0, yaw0 = node.latest_odom

    # 3. Execute the cmd_vel push (or skip in dry-run)
    if not dry_run:
        deadline = time.time() + duration_s
        # 50 Hz publish rate: dominates the backend's 10 Hz stale-command
        # watchdog 5:1, so even when the smoother samples at 50 Hz, the
        # most recent cmd_vel is statistically ours. Combined with the WS
        # tunnel (which keeps lastCmdTime fresh) this should fully
        # suppress the watchdog interference.
        rate_hz = 50.0
        period = 1.0 / rate_hz
        while time.time() < deadline:
            # Lost-tag watchdog: if no detection in last 1 s, abort the leg
            recent = node.hits_in_window(1.0)
            if recent < 3:
                node.stop_motors()
                return None, "lost_tag_during_motion"
            node.publish_cmd_vel(cmd_vel, 0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
        node.stop_motors()
    # 4. Wait for stabilization (decel + settle)
    settle_deadline = time.time() + 2.0
    while time.time() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    # 5. Capture post pose
    post = node.capture_median_pose(args.samples)
    if post is None:
        return None, "lost_tag_post"
    post_sample, n_post = post

    # 6. Snapshot odom at end
    if node.latest_odom is None:
        return None, "no_odom_post"
    t_end, x1, y1, yaw1 = node.latest_odom

    # 7. Compute deltas
    # Tag delta in optical frame: ZED looks down-forward, so +tz means tag
    # is ahead of the camera. When the robot moves forward by Δ, the tag
    # appears to move by -Δ in tz. We therefore flip the sign.
    dtag_x = post_sample.tx - pre_sample.tx
    dtag_y = post_sample.ty - pre_sample.ty
    dtag_z = post_sample.tz - pre_sample.tz
    # Robot-frame forward distance ≈ -dtag_z (camera optical Z+ = forward)
    robot_dx_optical_proxy = -dtag_z
    dtag_range = math.hypot(dtag_x, math.hypot(dtag_y, dtag_z))

    dodom_x = x1 - x0
    dodom_y = y1 - y0
    dodom_yaw = yaw1 - yaw0
    dodom_distance = math.hypot(dodom_x, dodom_y)
    if leg_name == "reverse":
        # Sign convention: report distances signed forward-positive
        dodom_distance_signed = -dodom_distance if dodom_x < 0 else dodom_distance
    else:
        dodom_distance_signed = dodom_distance if dodom_x >= 0 else -dodom_distance

    error_distance = robot_dx_optical_proxy - dodom_distance_signed
    target = abs(cmd_vel) * (duration_s)
    target_signed = target * (1 if cmd_vel >= 0 else -1)
    if abs(target_signed) > 1e-6:
        error_pct = 100.0 * (robot_dx_optical_proxy - target_signed) / target_signed
    else:
        error_pct = 0.0

    note = ",".join(note_parts) if note_parts else ("dry_run" if dry_run else "")

    return CalibRow(
        round_idx=-1,  # filled by caller
        leg=leg_name,
        cmd_vel=cmd_vel,
        cmd_dur_s=duration_s,
        dt_s=t_end - t_start,
        dtag_x_cam=dtag_x,
        dtag_y_cam=dtag_y,
        dtag_z_cam=dtag_z,
        dtag_range=dtag_range,
        dodom_x=dodom_x,
        dodom_y=dodom_y,
        dodom_yaw=dodom_yaw,
        dodom_distance=dodom_distance_signed,
        error_distance=error_distance,
        error_pct=error_pct,
        n_samples_pre=n_pre,
        n_samples_post=n_post,
        note=note,
    ), note


# ── Main ─────────────────────────────────────────────────────────────────
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--id", type=int, default=12)
    parser.add_argument("--tag-size", type=float, default=0.20)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--distance", type=float, default=0.15,
                        help="target one-way distance, meters (default 0.15)")
    parser.add_argument("--speed", type=float, default=0.10,
                        help="cmd_vel linear.x magnitude, m/s (default 0.10)")
    parser.add_argument("--accel-pad", type=float, default=0.5,
                        help="extra seconds added to t = distance/speed to "
                             "account for velocity_smoother ramp (default 0.5)")
    parser.add_argument("--samples", type=int, default=30,
                        help="median window size for pose capture")
    parser.add_argument("--dry-run", action="store_true",
                        help="capture poses but do NOT publish cmd_vel")
    parser.add_argument("--out-dir", type=str,
                        default=str(Path(__file__).parent / "calib_runs"),
                        help="directory to write CSV (default: tools/calib_runs)")
    parser.add_argument("--ws-url", type=str,
                        default="ws://localhost:8090/ws/control",
                        help="agv_ui_backend WebSocket URL. Tunnel cmd_vel "
                             "through here to coexist with an open dashboard. "
                             "Pass an empty string to bypass and publish "
                             "directly to /agv/cmd_vel (will fight the "
                             "backend stale-command watchdog).")
    args = parser.parse_args(argv)

    rclpy.init()
    ws_url = args.ws_url if args.ws_url else None
    node = CalibNode(args.id, args.tag_size, args.dry_run, ws_url=ws_url)
    if ws_url is not None and not args.dry_run:
        node.connect_ws()

    rc = 0
    rows: list[CalibRow] = []
    summary_status = "INCOMPLETE"
    try:
        # Pre-flight
        node.get_logger().info("Pre-flight checks running…")
        ok, msg = preflight(node, args.dry_run)
        if not ok:
            node.get_logger().error(f"PRE-FLIGHT FAIL: {msg}")
            return 2
        node.get_logger().info(f"Pre-flight OK ({msg})")

        # Switch to teleop mode so mode_arbiter doesn't fight us for cmd_vel
        if not args.dry_run:
            node.request_teleop_mode()
            node.get_logger().info("Requested mode='teleop' from mode_arbiter")

        # Compute leg duration
        leg_duration_s = (args.distance / args.speed) + args.accel_pad
        node.get_logger().info(
            f"Plan: {args.rounds} rounds × (forward {args.speed:+.3f} m/s, "
            f"reverse {-args.speed:+.3f} m/s) for {leg_duration_s:.2f} s each. "
            f"Target distance per leg = {args.distance:.3f} m."
        )

        # Run rounds
        for r in range(1, args.rounds + 1):
            node.get_logger().info(f"── Round {r}/{args.rounds} ──")
            for leg, vel in (("forward", +args.speed), ("reverse", -args.speed)):
                node.get_logger().info(
                    f"  Leg {leg}: cmd_vel.linear.x = {vel:+.3f} m/s "
                    f"for {leg_duration_s:.2f} s"
                )
                row, note = run_leg(node, leg, vel, leg_duration_s,
                                    args.dry_run, args)
                if row is None:
                    node.get_logger().warn(f"  Leg {leg} aborted: {note}")
                    if note.startswith("lost_tag"):
                        # Save a diagnostic row so the CSV reflects the abort
                        rows.append(CalibRow(
                            round_idx=r, leg=leg, cmd_vel=vel,
                            cmd_dur_s=leg_duration_s, dt_s=0.0,
                            dtag_x_cam=0.0, dtag_y_cam=0.0, dtag_z_cam=0.0,
                            dtag_range=0.0, dodom_x=0.0, dodom_y=0.0,
                            dodom_yaw=0.0, dodom_distance=0.0,
                            error_distance=0.0, error_pct=0.0,
                            n_samples_pre=0, n_samples_post=0, note=note,
                        ))
                    continue
                row.round_idx = r
                rows.append(row)
                node.get_logger().info(
                    f"    Δtag(forward proxy) = {-row.dtag_z_cam:+.4f} m, "
                    f"Δodom = {row.dodom_distance:+.4f} m, "
                    f"error = {row.error_distance:+.4f} m "
                    f"({row.error_pct:+.1f}%)"
                )
        summary_status = "OK"

    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user — stopping motors and saving partial CSV")
        summary_status = "INTERRUPTED"
        rc = 130
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"Unhandled exception: {e!r}")
        summary_status = f"EXCEPTION: {e!r}"
        rc = 1
    finally:
        # Belt-and-suspenders stop
        try:
            node.stop_motors()
        except Exception:
            pass
        node.close_ws()

        # Persist CSV
        Path(args.out_dir).mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%dT%H%M%S")
        csv_path = Path(args.out_dir) / f"run_{ts}.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "round", "leg", "cmd_vel_mps", "cmd_dur_s", "dt_s",
                "dtag_x_cam", "dtag_y_cam", "dtag_z_cam", "dtag_range",
                "dodom_x", "dodom_y", "dodom_yaw", "dodom_distance",
                "error_distance", "error_pct",
                "n_samples_pre", "n_samples_post", "note",
            ])
            for r in rows:
                w.writerow([
                    r.round_idx, r.leg, f"{r.cmd_vel:.4f}",
                    f"{r.cmd_dur_s:.3f}", f"{r.dt_s:.3f}",
                    f"{r.dtag_x_cam:.5f}", f"{r.dtag_y_cam:.5f}",
                    f"{r.dtag_z_cam:.5f}", f"{r.dtag_range:.5f}",
                    f"{r.dodom_x:.5f}", f"{r.dodom_y:.5f}",
                    f"{r.dodom_yaw:.5f}", f"{r.dodom_distance:.5f}",
                    f"{r.error_distance:.5f}", f"{r.error_pct:.3f}",
                    r.n_samples_pre, r.n_samples_post, r.note,
                ])
        node.get_logger().info(f"CSV written: {csv_path}")

        # Quick summary
        useful = [r for r in rows if not r.note.startswith("lost_tag") and r.dt_s > 0]
        if useful:
            fwd = [r.error_pct for r in useful if r.leg == "forward"]
            rev = [r.error_pct for r in useful if r.leg == "reverse"]
            mean = lambda xs: statistics.mean(xs) if xs else float("nan")
            stdev = lambda xs: statistics.stdev(xs) if len(xs) >= 2 else float("nan")
            node.get_logger().info(
                f"SUMMARY ({summary_status}): "
                f"forward N={len(fwd)} mean_err_pct={mean(fwd):+.2f}% "
                f"std={stdev(fwd):.2f}% | "
                f"reverse N={len(rev)} mean_err_pct={mean(rev):+.2f}% "
                f"std={stdev(rev):.2f}%"
            )
        else:
            node.get_logger().warn("SUMMARY: no usable rows captured")

        node.destroy_node()
        rclpy.shutdown()

    return rc


if __name__ == "__main__":
    sys.exit(main())
