#!/usr/bin/env python3
"""
test_nav2_probe — 90-second HIL smoke test to confirm Nav2 closed-loop
drives the physical robot before the precision/mapping runs begin.

Rationale (plan S9.3): after bring-up, we saw Nav2 goals get accepted but
produce zero GT delta — usually because the brain's est_pose is far from
GT and the goal lies inside xy_goal_tolerance in the brain's frame. This
probe catches that state in < 2 minutes, versus discovering it 15 minutes
into a precision run.

Sequence:
  1. POST /reset (0, 0, 0)  — teleport to a known open spot
  2. arm motors
  3. sync brain EKF to GT (via /agv/set_pose)
  4. POST /goal (1.0, 0, 0)
  5. poll /state for 60 s; record gt_delta_x
  6. PASS if gt_delta_x > 0.6 m  (>60 % of nominal path)
  7. Otherwise FAIL and write probe_report.json so the caller can
     triage via docs/validation/iteration_loop.md `goal_no_motion`.

Skips cleanly without SIM_API_HOST or ROS_DOMAIN_ID=42 (same semantics as
test_waypoint_precision.py).
"""
from __future__ import annotations

import json
import math
import os
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Optional

import pytest

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from std_msgs.msg import Bool
except ImportError:
    pytest.skip("rclpy not available", allow_module_level=True)

try:
    from robot_localization.srv import SetPose
    _HAS_SETPOSE = True
except ImportError:
    SetPose = None
    _HAS_SETPOSE = False


SIM_API_HOST = os.environ.get("SIM_API_HOST")
SIM_API_PORT = int(os.environ.get("SIM_API_PORT", "8090"))
AGV_DATA_DIR = Path(os.environ.get("AGV_DATA_DIR", str(Path.home() / "agv_data")))
SET_POSE_SERVICE = os.environ.get("AGV_SET_POSE_SRV", "/agv/set_pose")

RESET_TIMEOUT_S = 4.0
SETTLE_S = 1.0
GOAL_X = 1.0
GOAL_POLL_TOTAL_S = 60.0
GOAL_POLL_INTERVAL_S = 2.0
PASS_DELTA_X_M = 0.6


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _post(path: str, payload: dict, timeout: float = 5.0) -> dict:
    url = f"http://{SIM_API_HOST}:{SIM_API_PORT}{path}"
    data = json.dumps(payload).encode()
    req = urllib.request.Request(
        url, data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def _get_state(timeout: float = 3.0) -> Optional[dict]:
    url = f"http://{SIM_API_HOST}:{SIM_API_PORT}/state"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except (urllib.error.URLError, urllib.error.HTTPError, OSError, json.JSONDecodeError):
        return None


def _gt_xy(state: dict) -> Optional[tuple[float, float, float]]:
    if not state:
        return None
    g = state.get("gt_pose") or {}
    if "x" not in g or "y" not in g:
        return None
    return (float(g["x"]), float(g["y"]), float(g.get("yaw", 0.0)))


def _sync_brain(node: Node, gt_x: float, gt_y: float, gt_yaw: float,
                timeout_s: float = 5.0) -> bool:
    if not _HAS_SETPOSE or SetPose is None:
        return False
    cli = node.create_client(SetPose, SET_POSE_SERVICE)
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False
    req = SetPose.Request()
    pwcs = PoseWithCovarianceStamped()
    pwcs.header.frame_id = "map"
    pwcs.header.stamp = node.get_clock().now().to_msg()
    pwcs.pose.pose.position.x = gt_x
    pwcs.pose.pose.position.y = gt_y
    qx, qy, qz, qw = _yaw_to_quat(gt_yaw)
    pwcs.pose.pose.orientation.x = qx
    pwcs.pose.pose.orientation.y = qy
    pwcs.pose.pose.orientation.z = qz
    pwcs.pose.pose.orientation.w = qw
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.01
    cov[14] = 1.0
    cov[21] = cov[28] = 1.0
    cov[35] = 0.01
    pwcs.pose.covariance = cov
    req.pose = pwcs
    fut = cli.call_async(req)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result() is not None
    return False


class _ResetDoneWaiter(Node):
    def __init__(self) -> None:
        super().__init__("test_nav2_probe_barrier")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.done = False
        self.create_subscription(Bool, "/agv/sim/reset_done", self._on, qos)

    def _on(self, msg: Bool) -> None:
        if msg.data:
            self.done = True

    def wait(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.done:
                return True
        return False


def _check_preconditions() -> None:
    if not SIM_API_HOST:
        pytest.skip("SIM_API_HOST env var required. See docs/validation/RUNBOOK_lan_hil.md.")
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        pytest.skip("ROS_DOMAIN_ID must be 42.")
    try:
        urllib.request.urlopen(f"http://{SIM_API_HOST}:{SIM_API_PORT}/state", timeout=3.0).read()
    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as e:
        pytest.skip(f"sim_api unreachable: {e}")


def _write_probe_report(result: dict) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = AGV_DATA_DIR / "sim_episodes" / f"nav2_probe_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "probe_report.json"
    path.write_text(json.dumps(result, indent=2, default=str))
    return path


def test_nav2_probe():
    _check_preconditions()

    rclpy.init()
    try:
        barrier = _ResetDoneWaiter()

        t0 = time.time()
        # 1. Reset
        _post("/reset", {"x": 0.0, "y": 0.0, "yaw": 0.0})
        reset_ok = barrier.wait(RESET_TIMEOUT_S)
        print(f"[probe] reset → reset_done={reset_ok}")

        # 2. Clear e_stop (latched on by /reset) and arm motors.
        #    Without clearing e_stop, sim_drive_shaping_node emits zero
        #    wheel targets even though cmd_vel is flowing.
        try:
            _post("/e_stop", {"on": False})
        except Exception as e:
            print(f"[probe] e_stop clear warn: {e}")
        try:
            _post("/motor/enable", {"on": True})
        except Exception as e:
            print(f"[probe] motor/enable warn: {e}")
        time.sleep(SETTLE_S)

        # 3. GT after reset
        state = _get_state()
        gt0 = _gt_xy(state)
        print(f"[probe] gt_after_reset = {gt0}")
        assert gt0 is not None, "no gt_pose after reset"

        # 4. Sync brain to GT (best effort)
        helper_node = rclpy.create_node("test_nav2_probe_sync")
        sync_ok = _sync_brain(helper_node, gt0[0], gt0[1], gt0[2])
        helper_node.destroy_node()
        print(f"[probe] sync_brain_to_gt → {sync_ok}")
        time.sleep(0.5)

        # 5. Send goal via sim_api (uses Nav2 NavigateToPose internally)
        goal_resp = _post("/goal", {"x": GOAL_X, "y": 0.0, "yaw": 0.0})
        print(f"[probe] /goal → {goal_resp}")

        # 6. Poll /state
        trace: list[dict] = []
        deadline = time.monotonic() + GOAL_POLL_TOTAL_S
        last_gt = gt0
        while time.monotonic() < deadline:
            time.sleep(GOAL_POLL_INTERVAL_S)
            st = _get_state()
            gt = _gt_xy(st) if st else None
            if gt is not None:
                last_gt = gt
                trace.append({"t": time.time() - t0, "gt": list(gt)})
                print(f"[probe] t={time.time()-t0:5.1f}s gt=({gt[0]:.3f}, {gt[1]:.3f}, yaw={gt[2]:.3f})")

        # 7. Verdict
        gt_final = last_gt or (0.0, 0.0, 0.0)
        delta_x = gt_final[0] - gt0[0]
        delta_y = gt_final[1] - gt0[1]
        passed = delta_x > PASS_DELTA_X_M
        effective_speed = delta_x / GOAL_POLL_TOTAL_S if GOAL_POLL_TOTAL_S > 0 else 0.0

        report = {
            "run_ts": t0,
            "reset_done": reset_ok,
            "sync_ok": sync_ok,
            "gt_start": list(gt0),
            "gt_final": list(gt_final),
            "delta_x_m": delta_x,
            "delta_y_m": delta_y,
            "goal": {"x": GOAL_X, "y": 0.0, "yaw": 0.0},
            "elapsed_s": time.time() - t0,
            "effective_x_speed_m_s": effective_speed,
            "trace": trace,
            "pass_threshold_delta_x_m": PASS_DELTA_X_M,
            "pass": passed,
        }
        path = _write_probe_report(report)
        print(f"[probe] REPORT {path}")
        print(f"[probe] delta_x={delta_x:.3f} m  pass={passed}")

        assert passed, (
            f"Nav2 probe failed: delta_x={delta_x:.3f} m <= {PASS_DELTA_X_M} m. "
            f"See {path} and docs/validation/iteration_loop.md → goal_no_motion."
        )
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    test_nav2_probe()
