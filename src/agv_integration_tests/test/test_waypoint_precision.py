#!/usr/bin/env python3
"""
test_waypoint_precision — HIL waypoint-reach precision gate.

Gate (specs/acceptance.yaml#hil_validation.waypoint_precision):
  err_xy p95 <= 0.10 m across 20 waypoints
  err_xy max <= 0.15 m
  yaw error p95 <= 0.25 rad
  success_rate >= 0.95
  zero collision events on /agv/sim/events during each run

Preconditions (see docs/validation/RUNBOOK_lan_hil.md):
  - sim host running Isaac Sim (manual Play pressed)
  - sim host `isaac_hil.launch.py validation:=true enable_api:=true`
  - Jetson running `agv_hil_full.launch.py map:=<map>`
  - env ROS_DOMAIN_ID=42
  - env SIM_API_HOST=<sim host IP>    (no default — prevents accidental localhost)
  - env AGV_DATA_DIR=<writable dir>    (default $HOME/agv_data)

The test speaks directly to Nav2's /navigate_to_pose action (NOT the
sim_api /goal endpoint) to isolate the precision metric from the HTTP
surface. /reset is via sim_api because the physical teleport lives
inside the Isaac Kit handler.
"""
from __future__ import annotations

import json
import math
import os
import statistics
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

# The mode_transitions_oracle helper sits next to this file in the test
# directory; make it importable whether pytest runs from the workspace
# root or from this directory.
sys.path.insert(0, str(Path(__file__).parent))

import pytest

try:
    import yaml
except ImportError:
    pytest.skip("python3-yaml not installed", allow_module_level=True)

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from nav2_msgs.action import NavigateToPose
except ImportError:
    pytest.skip("rclpy / nav2_msgs not available", allow_module_level=True)

try:
    from robot_localization.srv import SetPose
    _HAS_SETPOSE = True
except ImportError:
    SetPose = None
    _HAS_SETPOSE = False

SIM_API_HOST = os.environ.get("SIM_API_HOST")
SIM_API_PORT = int(os.environ.get("SIM_API_PORT", "8090"))
AGV_DATA_DIR = Path(os.environ.get("AGV_DATA_DIR", str(Path.home() / "agv_data")))
_WAYPOINTS_FILENAME = os.environ.get("AGV_WAYPOINTS_YAML", "waypoints_20.yaml")
WAYPOINTS_PATH = Path(_WAYPOINTS_FILENAME) if Path(_WAYPOINTS_FILENAME).is_absolute() \
    else Path(__file__).parent / _WAYPOINTS_FILENAME
MIN_WAYPOINTS = int(os.environ.get("AGV_MIN_WAYPOINTS", "20"))

RESET_TIMEOUT_S = 5.0  # was 3.0 — round 20 had wp03, wp04 RESET_TIMEOUT when
                       # GT didn't arrive within 3 s after the /reset POST.
                       # 5 s covers sim-host teleport latency + DDS hop.
POST_RESET_SETTLE_S = 1.0
REINIT_THRESHOLD_M = 0.30
# NAV_TIMEOUT_S is the per-waypoint wall-clock deadline for /navigate_to_pose.
# The sim runs at ~5 % drive efficiency (cmd 0.5 m/s → ~0.025 m/s real) as of
# 2026-04-17, so tolerate up to 5 min per waypoint for the 4 m-scale goals in
# waypoints_20.yaml. Override with AGV_NAV_TIMEOUT_S for faster iteration.
NAV_TIMEOUT_S = float(os.environ.get("AGV_NAV_TIMEOUT_S", "300.0"))
# After agv_hil_full.launch.py remap (2026-04-17), ekf_local and ekf_global
# each expose their own set_pose service. Both must be called: ekf_local
# resets the odom→base_link state so wheel_odom deltas stop pushing stale
# motion into ekf_global. ekf_global resets the map→odom output Nav2 plans
# from. Calling only ekf_global is absorbed within a single tick by the
# deltas ekf_local keeps emitting.
SET_POSE_LOCAL = "/agv/ekf_local/set_pose"
SET_POSE_GLOBAL = "/agv/ekf_global/set_pose"
# Legacy single-service env override kept for backward compatibility.
SET_POSE_SERVICE = os.environ.get("AGV_SET_POSE_SRV", "/agv/set_pose")
SYNC_SETTLE_S = 0.5
SYNC_TOLERANCE_M = 0.20

P95_ERR_XY_M = 0.10
MAX_ERR_XY_M = 0.15
P95_ERR_YAW_RAD = 0.25
MIN_SUCCESS_RATE = 0.95

# Self-heal (S8): optional POST /sim/restart on physics corruption.
# Default OFF — CI stays deterministic; enable from the iteration loop.
AUTO_RESTART_ENABLED = os.environ.get("AGV_TEST_AUTO_RESTART", "") == "1"
AUTO_RESTART_MAX = int(os.environ.get("AGV_TEST_AUTO_RESTART_MAX", "2"))
AUTO_RESTART_COOLDOWN_S = float(os.environ.get("AGV_TEST_AUTO_RESTART_COOLDOWN_S", "120"))
AUTO_RESTART_READY_TIMEOUT_S = 90.0


@dataclass
class WaypointResult:
    wp_id: str
    goal_x: float
    goal_y: float
    goal_yaw: float
    gt_x: Optional[float]
    gt_y: Optional[float]
    gt_yaw: Optional[float]
    brain_x: Optional[float]
    brain_y: Optional[float]
    brain_yaw: Optional[float]
    err_xy: Optional[float]
    err_yaw: Optional[float]
    brain_drift: Optional[float]
    status: str
    events_during: list
    duration_s: float
    # Phase 2 instrumentation: ordered list of unique modes observed on
    # /agv/mode/state during this waypoint, plus an expected_modes check
    # result. None/[] when the topic wasn't observed (legacy waypoints).
    modes_observed: Optional[list] = None
    modes_expected: Optional[list] = None
    modes_match: Optional[bool] = None


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _cancel_all_nav_goals(node: Node, timeout_s: float = 2.0) -> bool:
    """Cancel any currently-executing /agv/navigate_to_pose action.

    Without this, the previous waypoint's goal stays active while we
    teleport, arm motors, and sync — and Nav2 happily keeps driving
    toward the old goal during the ~2 s between /reset and the new goal
    send. Observed round 18 wp02: robot teleported to (1, 0) per /reset,
    then drove 1.06 m backward toward the (0, 0) goal of wp01 during
    the sync window.

    Uses the action's built-in /cancel_goal service (not ActionClient,
    which has a pybind11 bug in rclpy for NavigateToPose feedback).
    """
    try:
        from action_msgs.srv import CancelGoal
    except ImportError:
        return False
    cli = node.create_client(CancelGoal, "/agv/navigate_to_pose/_action/cancel_goal")
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False
    # Empty goal_info + timestamp=0 cancels ALL active goals on this action.
    req = CancelGoal.Request()
    fut = cli.call_async(req)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result() is not None
    return False


def _post_sim_api(path: str, payload: dict, timeout: float = 5.0) -> dict:
    url = f"http://{SIM_API_HOST}:{SIM_API_PORT}{path}"
    data = json.dumps(payload).encode()
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def _arm_and_clear_estop(verbose: bool = False) -> None:
    """POST /reset leaves motors disarmed AND e_stop latched on. Every
    waypoint must arm motors and explicitly clear e_stop, otherwise
    sim_drive_shaping_node emits zero wheel targets (see RUNBOOK §6.3
    + iteration_loop.md 'goal_no_motion' for full diagnosis).
    """
    try:
        _post_sim_api("/e_stop", {"on": False}, timeout=3.0)
    except Exception as e:
        if verbose:
            print(f"[arm] e_stop clear warn: {e}")
    try:
        _post_sim_api("/motor/enable", {"on": True}, timeout=3.0)
    except Exception as e:
        if verbose:
            print(f"[arm] motor enable warn: {e}")


def _post_sim_api_empty(path: str, timeout: float = 5.0) -> bool:
    url = f"http://{SIM_API_HOST}:{SIM_API_PORT}{path}"
    req = urllib.request.Request(url, data=b"", method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return 200 <= r.status < 300
    except (urllib.error.URLError, urllib.error.HTTPError, OSError):
        return False


def _get_sim_api(path: str, timeout: float = 5.0) -> Optional[dict]:
    url = f"http://{SIM_API_HOST}:{SIM_API_PORT}{path}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except (urllib.error.URLError, urllib.error.HTTPError, OSError, json.JSONDecodeError):
        return None


def _wait_for_sim_ready(timeout_s: float = AUTO_RESTART_READY_TIMEOUT_S) -> bool:
    """Poll GET /state until gt_pose is non-null — the supervisor post-restart signal."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = _get_sim_api("/state", timeout=3.0)
        if state is not None and state.get("gt_pose") is not None:
            return True
        time.sleep(3.0)
    return False


class Harness(Node):
    def __init__(self) -> None:
        super().__init__(
            "test_waypoint_precision_harness",
            # use_sim_time is mandatory: without it, node.get_clock().now()
            # returns system wall time (seconds since epoch, ~1.78e9). The
            # SetPose request uses that timestamp, which is FAR ahead of the
            # sim's clock (seconds since Isaac session start, ~1e3). The EKF
            # interprets this as "message from the future", resets its
            # timestamp cursor, and then ignores every subsequent wheel_odom
            # message as "before last filter reset" (seen in ekf_local
            # diagnostics round 17). Setting use_sim_time here makes the
            # test's clock match the brain's clock, so SetPose stamps are
            # comparable with odom stamps.
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True
            )],
        )

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.gt_pose: Optional[PoseStamped] = None
        self.brain_odom: Optional[Odometry] = None
        self.reset_done_stamp: Optional[float] = None
        self._events: list[dict] = []
        # Phase 2: record unique mode transitions per waypoint.
        from mode_transitions_oracle import ModeTransitionRecorder
        self._mode_recorder = ModeTransitionRecorder()

        self.create_subscription(
            PoseStamped,
            "/agv/sim/ground_truth/pose",
            self._on_gt,
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            "/agv/odometry/global",
            self._on_brain,
            best_effort_qos,
        )
        self.create_subscription(
            Bool,
            "/agv/sim/reset_done",
            self._on_reset_done,
            latched_qos,
        )
        self.create_subscription(
            String,
            "/agv/sim/events",
            self._on_event,
            reliable_qos,
        )
        # Phase 2: record mode transitions emitted by agv_mode_arbiter.
        self.create_subscription(
            String,
            "/agv/mode/state",
            self._on_mode_state,
            reliable_qos,
        )

        # NOTE: no ActionClient here. rclpy's internal feedback-subscription
        # triggers a pybind11 TypeError on nav2_msgs/NavigateToPose feedback
        # (observed 2026-04-17). navigate_to() uses sim_api HTTP /goal instead.

    def _on_gt(self, msg: PoseStamped) -> None:
        self.gt_pose = msg

    def _on_brain(self, msg: Odometry) -> None:
        self.brain_odom = msg

    def _on_reset_done(self, msg: Bool) -> None:
        if msg.data:
            self.reset_done_stamp = time.monotonic()

    def _on_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}
        payload["_received_wall_s"] = time.time()
        self._events.append(payload)

    def _on_mode_state(self, msg: String) -> None:
        self._mode_recorder.record_transition(msg.data)

    def begin_waypoint_modes(self) -> None:
        self._mode_recorder.begin_waypoint()

    def modes_observed(self) -> list:
        return self._mode_recorder.modes_seen()

    def spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def events_since(self, t0: float) -> list[dict]:
        return [e for e in self._events if e.get("_received_wall_s", 0.0) >= t0]

    def wait_for_reset(self, start_x: float = 0.0, start_y: float = 0.0,
                       tol_m: float = 0.30) -> bool:
        """Wait until the teleport takes effect.

        Trust ONLY the ground truth: the test POSTed /reset with a specific
        pose, so the teleport is complete when GT lands within `tol_m` of
        that pose. /agv/sim/reset_done is unreliable — the TRANSIENT_LOCAL
        latched True from prior sessions fires the callback at subscriber
        creation time, which can race ahead of the new /reset.
        """
        deadline = time.monotonic() + RESET_TIMEOUT_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            gt = self.current_gt_xy()
            if gt is not None:
                d = math.hypot(gt[0] - start_x, gt[1] - start_y)
                if d <= tol_m:
                    return True
        return False

    def current_gt_xy(self) -> Optional[tuple[float, float, float]]:
        if self.gt_pose is None:
            return None
        p = self.gt_pose.pose
        return (p.position.x, p.position.y, _quat_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def current_brain_xy(self) -> Optional[tuple[float, float, float]]:
        if self.brain_odom is None:
            return None
        p = self.brain_odom.pose.pose
        return (p.position.x, p.position.y, _quat_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def wait_for_action_server(self, timeout_s: float = 10.0) -> bool:
        # sim_api proxies NavigateToPose via its own ActionClient; we poll its
        # HTTP surface. Since /goal maps 1:1 to the action server, availability
        # of the action server implies sim_api is wired.
        try:
            urllib.request.urlopen(
                f"http://{SIM_API_HOST}:{SIM_API_PORT}/state", timeout=timeout_s
            ).read()
            return True
        except (urllib.error.URLError, urllib.error.HTTPError, OSError):
            return False

    def navigate_to(self, gx: float, gy: float, gyaw: float, timeout_s: float) -> str:
        """Send Nav2 goal via sim_api HTTP and poll GT until convergence.

        We avoid rclpy.action.ActionClient because of a pybind11 TypeError
        in rclpy when taking feedback from nav2_msgs/NavigateToPose
        (observed 2026-04-17 running Humble/Jazzy cross-distro). The sim_api
        runs its own ActionClient on the sim host and the bug does not
        manifest there.

        Convergence criteria:
          - "SUCCEEDED" when `|gt_xy - goal_xy| <= done_xy_tol` AND
            `|wrap(gt_yaw - goal_yaw)| <= done_yaw_tol`, sustained for
            `consecutive_hits_needed` polls.
          - "ABORTED" if GT xy stalls (<2 cm delta) for >60 s (the drive
            motor stuck or MPPI refused to plan — not a brain test
            failure, but the waypoint cannot converge).
          - "NAV_TIMEOUT" if deadline passes without convergence.
          - "COLLISION" short-circuited by the caller via /agv/sim/events.
        """
        # V9 / round 29b: test threshold = gate threshold (0.10 m).
        # Round 29a with done_xy_tol=0.05 caused false-ABORTs on wp01 at
        # 0.051 m which would PASS the 0.10 m gate. The tightening belongs
        # to Nav2's xy_goal_tolerance (0.05) — that makes the CONTROLLER
        # aim tighter, landing inside the gate. Test polling at 0.10 m
        # accepts the MPPI landing site.
        done_xy_tol = 0.10
        done_yaw_tol = 0.30  # ~17° — loose enough that MPPI's forward-only
                             # heading at arrival is accepted.
        poll_interval = 1.5
        consecutive_hits_needed = 5  # 5 × 1.5s = 7.5s dwell; absorbs inertia
        # Round 5 (stall=60, no retry) gave 25% / p95 0.077. Round 6 (stall=120, retry)
        # dropped to 20% / p95 0.143 — retries confused MPPI's planner when it had
        # already stopped commanding. Revert to round 5's setup.
        stall_abort_s = 90.0  # middle of 60/120 — room to creep but no runaway
        # No periodic sync during nav. Periodic SetPose mid-flight causes
        # Nav2's MPPI to see sudden "teleports" of its own est_pose, which
        # destabilizes the controller and may trigger premature SUCCEEDED
        # (when the snap lands est inside the goal tolerance even though
        # GT is still far away). Round 11f showed this: Nav2 declared
        # success while GT was 0.32 m from goal. We rely on the single
        # pre-nav sync instead; any drift is a real measurement of the
        # brain's ability to navigate under sim drive inefficiency.
        # Set to 0 to disable; or a large value to effectively disable.
        sync_interval_s = 1e9

        try:
            _post_sim_api("/goal", {"x": float(gx), "y": float(gy), "yaw": float(gyaw)}, timeout=5.0)
        except Exception:
            return "GOAL_SEND_TIMEOUT"

        consecutive_hits = 0
        deadline = time.monotonic() + timeout_s
        last_progress_mono = time.monotonic()
        last_sync_mono = time.monotonic()
        last_d = None
        while time.monotonic() < deadline:
            time.sleep(poll_interval)
            rclpy.spin_once(self, timeout_sec=0.05)
            gt = self.current_gt_xy()
            if gt is None:
                continue
            d_xy = math.hypot(gt[0] - gx, gt[1] - gy)
            d_yaw = abs(_wrap_angle(gt[2] - gyaw))
            if d_xy <= done_xy_tol and d_yaw <= done_yaw_tol:
                consecutive_hits += 1
                if consecutive_hits >= consecutive_hits_needed:
                    return "SUCCEEDED"
            else:
                consecutive_hits = 0
            if time.monotonic() - last_sync_mono > sync_interval_s:
                _sync_brain_to_gt(self, gt[0], gt[1], gt[2], timeout_s=1.5)
                last_sync_mono = time.monotonic()
            if last_d is not None and abs(last_d - d_xy) < 0.02:
                if time.monotonic() - last_progress_mono > stall_abort_s:
                    return "ABORTED"
            else:
                last_progress_mono = time.monotonic()
            last_d = d_xy
        return "NAV_TIMEOUT"


def _reinit_localization(node: Node, timeout_s: float = 3.0) -> bool:
    from std_srvs.srv import Trigger

    cli = node.create_client(Trigger, "/agv/localization/reinitialize")
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False
    fut = cli.call_async(Trigger.Request())
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result() is not None and fut.result().success
    return False


def _build_setpose_req(gt_x: float, gt_y: float, gt_yaw: float,
                       frame_id: str, stamp) -> "SetPose.Request":
    req = SetPose.Request()
    pwcs = PoseWithCovarianceStamped()
    pwcs.header.frame_id = frame_id
    pwcs.header.stamp = stamp
    pwcs.pose.pose.position.x = float(gt_x)
    pwcs.pose.pose.position.y = float(gt_y)
    qx, qy, qz, qw = _yaw_to_quat(float(gt_yaw))
    pwcs.pose.pose.orientation.x = qx
    pwcs.pose.pose.orientation.y = qy
    pwcs.pose.pose.orientation.z = qz
    pwcs.pose.pose.orientation.w = qw
    # Covariance policy: use 0.001 (matching wheel_odom) so the injected
    # pose has ~50% weight against the next wheel_odom observation. With
    # gt_to_wheel_odom mirroring ground truth (use_gt_odom:=true), this
    # allows the subsequent wheel_odom stream to refine the estimate
    # toward GT. With the old 1e-6 value, SetPose dominated so heavily
    # that wheel_odom took ~5 s to move the EKF by 1 cm; during navigation
    # this showed as EST lagging GT by 0.2+ m (round 15 manual test).
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.001
    cov[14] = 1.0
    cov[21] = cov[28] = 1.0
    cov[35] = 0.001
    pwcs.pose.covariance = cov
    req.pose = pwcs
    return req


def _call_setpose(node: Node, service: str, req: "SetPose.Request",
                  timeout_s: float = 5.0) -> bool:
    cli = node.create_client(SetPose, service)
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False
    fut = cli.call_async(req)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result() is not None
    return False


_SYNC_CLIENT_CACHE: dict = {}


def _sync_brain_to_gt(node: Node, gt_x: float, gt_y: float, gt_yaw: float,
                      timeout_s: float = 5.0, verbose: bool = False) -> bool:
    """Force BOTH EKFs (local + global) to believe pose = (gt_x, gt_y, gt_yaw).

    Calling only /agv/ekf_global/set_pose isn't enough: ekf_local keeps
    emitting wheel_odom-derived deltas that are consumed by ekf_global as
    odom0_differential, nullifying the global sync within one tick. The
    split set_pose services (post 2026-04-17 launch remap) make this
    possible.

    Local filter owns odom→base_link and operates in odom frame; pass
    frame_id='odom' there. Global filter operates in map frame.

    Caches service clients between calls — fresh create_client+wait_for_service
    inside a tight 1 s sync loop adds ~50 ms latency per call that stacks up.
    """
    if not _HAS_SETPOSE or SetPose is None:
        if verbose:
            print("[sync] SetPose srv not available")
        return False
    stamp = node.get_clock().now().to_msg()
    req_local = _build_setpose_req(gt_x, gt_y, gt_yaw, "odom", stamp)
    req_global = _build_setpose_req(gt_x, gt_y, gt_yaw, "map", stamp)

    def cached_client(svc: str):
        cli = _SYNC_CLIENT_CACHE.get(svc)
        if cli is None:
            cli = node.create_client(SetPose, svc)
            _SYNC_CLIENT_CACHE[svc] = cli
        return cli

    def call(svc: str, req) -> bool:
        cli = cached_client(svc)
        if not cli.wait_for_service(timeout_sec=timeout_s):
            if verbose:
                print(f"[sync] service {svc} not ready after {timeout_s}s")
            return False
        fut = cli.call_async(req)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if fut.done():
                return fut.result() is not None
        if verbose:
            print(f"[sync] service {svc} call_async timed out after {timeout_s}s")
        return False

    ok_local = call(SET_POSE_LOCAL, req_local)
    ok_global = call(SET_POSE_GLOBAL, req_global)
    if verbose:
        print(f"[sync] local={ok_local} global={ok_global} gt=({gt_x:.3f},{gt_y:.3f},{gt_yaw:.3f})")
    if not ok_local and not ok_global:
        req_legacy = _build_setpose_req(gt_x, gt_y, gt_yaw, "map", stamp)
        return call(SET_POSE_SERVICE, req_legacy)
    return ok_local or ok_global


class RestartBudget:
    """Tracks /sim/restart usage for the lifetime of one test invocation."""

    def __init__(self, max_restarts: int, cooldown_s: float) -> None:
        self.max_restarts = max_restarts
        self.cooldown_s = cooldown_s
        self.used = 0
        self.last_attempt_mono: Optional[float] = None
        self.events: list[dict] = []

    def remaining(self) -> int:
        return max(0, self.max_restarts - self.used)

    def cooldown_remaining(self) -> float:
        if self.last_attempt_mono is None:
            return 0.0
        return max(0.0, self.cooldown_s - (time.monotonic() - self.last_attempt_mono))

    def try_restart(self, reason: str, wp_id: Optional[str]) -> bool:
        if not AUTO_RESTART_ENABLED:
            return False
        if self.remaining() <= 0:
            self.events.append({
                "ts": time.time(),
                "reason": reason,
                "waypoint_id": wp_id,
                "success": False,
                "skipped_reason": "budget_exhausted",
            })
            return False
        cd = self.cooldown_remaining()
        if cd > 0:
            self.events.append({
                "ts": time.time(),
                "reason": reason,
                "waypoint_id": wp_id,
                "success": False,
                "skipped_reason": f"cooldown_{cd:.1f}s",
            })
            return False
        self.used += 1
        self.last_attempt_mono = time.monotonic()
        print(f"[self-heal] POST /sim/restart (reason={reason}, wp={wp_id}, used={self.used}/{self.max_restarts})")
        post_ok = _post_sim_api_empty("/sim/restart", timeout=10.0)
        ready = post_ok and _wait_for_sim_ready()
        self.events.append({
            "ts": time.time(),
            "reason": reason,
            "waypoint_id": wp_id,
            "success": bool(ready),
            "post_ok": post_ok,
        })
        return bool(ready)


def _check_preconditions() -> None:
    if not SIM_API_HOST:
        pytest.skip("SIM_API_HOST env var required. See docs/validation/RUNBOOK_lan_hil.md.")
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        pytest.skip("ROS_DOMAIN_ID must be 42 to see /agv/sim/* topics. See docs/validation/RUNBOOK_lan_hil.md.")
    if not WAYPOINTS_PATH.is_file():
        pytest.fail(f"missing waypoint spec: {WAYPOINTS_PATH}")
    try:
        urllib.request.urlopen(f"http://{SIM_API_HOST}:{SIM_API_PORT}/state", timeout=3.0).read()
    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as e:
        pytest.skip(f"sim_api at {SIM_API_HOST}:{SIM_API_PORT} unreachable: {e}")


def _run_one_waypoint(
    harness: Harness,
    wp: dict,
    budget: Optional[RestartBudget] = None,
) -> WaypointResult:
    wp_id = wp["id"]
    start = wp["start"]
    goal = wp["goal"]
    expected_modes = wp.get("expected_modes")  # may be None for legacy yaml

    # Phase 2: clear the recorder so modes_observed() tracks only this wp.
    harness.begin_waypoint_modes()

    # 0. Cancel any active Nav2 action from the previous waypoint before
    #    teleporting — otherwise Nav2 keeps driving the robot toward the old
    #    goal during the post-reset settle window (round 18 wp02 symptom).
    _cancel_all_nav_goals(harness)

    # 1. Teleport, with one self-heal retry on RESET_TIMEOUT if the budget allows.
    t_reset_issue = time.time()
    reset_ok = False
    for attempt in range(2):
        try:
            _post_sim_api(
                "/reset",
                {"x": float(start["x"]), "y": float(start["y"]), "yaw": float(start["yaw"])},
            )
        except (urllib.error.URLError, urllib.error.HTTPError, OSError):
            # sim_api unreachable — probably restarting; fall through to wait.
            pass
        if harness.wait_for_reset(
            start_x=float(start["x"]), start_y=float(start["y"]), tol_m=0.30
        ):
            reset_ok = True
            break
        if attempt == 0 and budget is not None and budget.try_restart(
            reason="reset_timeout", wp_id=wp_id
        ):
            # After restart, Nav2 may need its action server re-registered.
            harness.wait_for_action_server(timeout_s=15.0)
            continue
        break
    if not reset_ok:
        return WaypointResult(
            wp_id=wp_id,
            goal_x=goal["x"], goal_y=goal["y"], goal_yaw=goal["yaw"],
            gt_x=None, gt_y=None, gt_yaw=None,
            brain_x=None, brain_y=None, brain_yaw=None,
            err_xy=None, err_yaw=None, brain_drift=None,
            status="RESET_TIMEOUT",
            events_during=[],
            duration_s=time.time() - t_reset_issue,
            modes_observed=harness.modes_observed(),
            modes_expected=expected_modes if expected_modes else None,
            modes_match=None,  # cannot assert — wp never actually ran
        )

    # 1b. Clear e_stop (latched on by /reset) and arm motors.
    _arm_and_clear_estop()

    # 2. Settle — let ekf_local absorb teleport.
    time.sleep(POST_RESET_SETTLE_S)
    rclpy.spin_once(harness, timeout_sec=0.1)

    # 3. Pre-goal sync: force brain EKF to reflect GT pose so Nav2 plans
    #    from reality, not from the stale cold-start pose. Falls back to
    #    /agv/localization/reinitialize if SetPose is unavailable.
    sync_ok = False
    sync_residual = None
    gt_after_reset = harness.current_gt_xy()
    if gt_after_reset is not None:
        sync_ok = _sync_brain_to_gt(
            harness, gt_after_reset[0], gt_after_reset[1], gt_after_reset[2],
            verbose=True,
        )
        if sync_ok:
            time.sleep(SYNC_SETTLE_S)
            rclpy.spin_once(harness, timeout_sec=0.1)
            brain_after_sync = harness.current_brain_xy()
            if brain_after_sync is not None:
                sync_residual = math.hypot(
                    gt_after_reset[0] - brain_after_sync[0],
                    gt_after_reset[1] - brain_after_sync[1],
                )
                if sync_residual > SYNC_TOLERANCE_M:
                    # Sync was consumed by wheel_odom or ignored — fallback.
                    _reinit_localization(harness)
                    harness.spin_until(lambda: False, 0.5)

    # 4. Residual check (legacy fallback, when sync unavailable).
    if not sync_ok:
        gt = harness.current_gt_xy()
        brain = harness.current_brain_xy()
        if gt is not None and brain is not None:
            residual = math.hypot(gt[0] - brain[0], gt[1] - brain[1])
            if residual > REINIT_THRESHOLD_M:
                _reinit_localization(harness)
                harness.spin_until(lambda: False, 0.5)

    # 4. Navigate.
    t_nav_start = time.time()
    t_events_start = time.time()
    status = harness.navigate_to(
        float(goal["x"]), float(goal["y"]), float(goal["yaw"]), NAV_TIMEOUT_S
    )

    # 5. Snapshot both poses at terminal.
    rclpy.spin_once(harness, timeout_sec=0.2)
    gt_final = harness.current_gt_xy()
    brain_final = harness.current_brain_xy()
    events_during = harness.events_since(t_events_start)

    err_xy = None
    err_yaw = None
    brain_drift = None
    if gt_final is not None:
        err_xy = math.hypot(gt_final[0] - goal["x"], gt_final[1] - goal["y"])
        err_yaw = abs(_wrap_angle(gt_final[2] - goal["yaw"]))
    if gt_final is not None and brain_final is not None:
        brain_drift = math.hypot(gt_final[0] - brain_final[0], gt_final[1] - brain_final[1])

    # 6. Collision hard-fail
    if any(e.get("event") == "collision" for e in events_during):
        status = "COLLISION"

    # Phase 2: snapshot mode transitions observed across this waypoint.
    modes_obs = harness.modes_observed()
    modes_match: Optional[bool] = None
    if expected_modes:
        from mode_transitions_oracle import is_subsequence
        modes_match = is_subsequence(expected_modes, modes_obs)

    return WaypointResult(
        wp_id=wp_id,
        goal_x=goal["x"], goal_y=goal["y"], goal_yaw=goal["yaw"],
        gt_x=gt_final[0] if gt_final else None,
        gt_y=gt_final[1] if gt_final else None,
        gt_yaw=gt_final[2] if gt_final else None,
        brain_x=brain_final[0] if brain_final else None,
        brain_y=brain_final[1] if brain_final else None,
        brain_yaw=brain_final[2] if brain_final else None,
        err_xy=err_xy,
        err_yaw=err_yaw,
        brain_drift=brain_drift,
        status=status,
        events_during=events_during,
        duration_s=time.time() - t_nav_start,
        modes_observed=modes_obs,
        modes_expected=expected_modes if expected_modes else None,
        modes_match=modes_match,
    )


def _summarize(results: list[WaypointResult]) -> dict:
    successful = [r for r in results if r.status == "SUCCEEDED" and r.err_xy is not None]
    n_total = len(results)
    n_success = len(successful)
    err_xy_values = [r.err_xy for r in successful if r.err_xy is not None]
    err_yaw_values = [r.err_yaw for r in successful if r.err_yaw is not None]

    def p95(values: list[float]) -> float:
        if not values:
            return float("nan")
        s = sorted(values)
        idx = max(0, math.ceil(0.95 * len(s)) - 1)
        return s[idx]

    return {
        "sample_size": n_total,
        "success_rate": n_success / n_total if n_total else 0.0,
        "p95_err_xy_m": p95(err_xy_values),
        "max_err_xy_m": max(err_xy_values) if err_xy_values else float("nan"),
        "mean_err_xy_m": statistics.mean(err_xy_values) if err_xy_values else float("nan"),
        "p95_err_yaw_rad": p95(err_yaw_values),
        "collision_count": sum(1 for r in results if r.status == "COLLISION"),
        "status_histogram": {
            s: sum(1 for r in results if r.status == s)
            for s in sorted({r.status for r in results})
        },
    }


def _write_report(
    results: list[WaypointResult],
    summary: dict,
    budget: Optional[RestartBudget] = None,
) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = AGV_DATA_DIR / "sim_episodes" / f"precision_run_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "run_id": f"precision_run_{ts}",
        "gates": {
            "p95_err_xy_m": P95_ERR_XY_M,
            "max_err_xy_m": MAX_ERR_XY_M,
            "p95_err_yaw_rad": P95_ERR_YAW_RAD,
            "min_success_rate": MIN_SUCCESS_RATE,
        },
        "self_heal": {
            "enabled": AUTO_RESTART_ENABLED,
            "max": AUTO_RESTART_MAX,
            "cooldown_s": AUTO_RESTART_COOLDOWN_S,
            "used": budget.used if budget else 0,
            "remaining": budget.remaining() if budget else AUTO_RESTART_MAX,
            "events": budget.events if budget else [],
        },
        "summary": summary,
        "waypoints": [r.__dict__ for r in results],
    }
    path = out_dir / "report.json"
    path.write_text(json.dumps(report, indent=2, default=str))
    return path


def test_waypoint_precision():
    print(f"[waypoint_precision] starting (sim_api={SIM_API_HOST}:{SIM_API_PORT}, "
          f"auto_restart={AUTO_RESTART_ENABLED}, nav_timeout={NAV_TIMEOUT_S}s)", flush=True)
    _check_preconditions()
    print(f"[waypoint_precision] preconditions OK, loading waypoints…", flush=True)

    with open(WAYPOINTS_PATH) as f:
        waypoints = yaml.safe_load(f)["waypoints"]
    assert len(waypoints) >= MIN_WAYPOINTS, \
        f"need at least {MIN_WAYPOINTS} waypoints, got {len(waypoints)}"

    rclpy.init()
    try:
        harness = Harness()
        assert harness.wait_for_action_server(timeout_s=15.0), \
            f"{os.environ.get('AGV_NAV_ACTION', '/agv/navigate_to_pose')} action server not available — is Nav2 up?"

        # DDS discovery warm-up: wait for first GT message before starting the
        # main loop. Cross-host CycloneDDS discovery takes ~5-6 s on first
        # subscription — if we kick off wp01 immediately, its 3 s wait_for_reset
        # times out before discovery even completes. Block here until we see GT.
        print("[waypoint_precision] warming up DDS discovery (up to 15 s)…", flush=True)
        warmup_deadline = time.monotonic() + 15.0
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(harness, timeout_sec=0.1)
            if harness.current_gt_xy() is not None:
                print(f"[waypoint_precision] GT discovered: {harness.current_gt_xy()}", flush=True)
                break
        else:
            print("[waypoint_precision] WARNING: GT not received within 15 s — proceeding anyway", flush=True)

        budget = RestartBudget(AUTO_RESTART_MAX, AUTO_RESTART_COOLDOWN_S)

        results: list[WaypointResult] = []
        for wp in waypoints[:MIN_WAYPOINTS]:
            print(f"[wp {wp['id']}] → ({wp['goal']['x']}, {wp['goal']['y']}, {wp['goal']['yaw']:.2f})")
            results.append(_run_one_waypoint(harness, wp, budget=budget))
            r = results[-1]
            print(f"    {r.status}  err_xy={r.err_xy}  err_yaw={r.err_yaw}  drift={r.brain_drift}  dur={r.duration_s:.1f}s")

        summary = _summarize(results)
        report_path = _write_report(results, summary, budget=budget)
        print(f"\nREPORT: {report_path}")
        print(f"SUMMARY: {json.dumps(summary, indent=2)}")
        if budget.events:
            print(f"SELF_HEAL: {budget.used}/{budget.max_restarts} restarts used")

        assert summary["collision_count"] == 0, \
            f"{summary['collision_count']} collision event(s) during precision run"
        assert summary["success_rate"] >= MIN_SUCCESS_RATE, \
            f"success_rate {summary['success_rate']:.2f} < {MIN_SUCCESS_RATE}"
        assert summary["p95_err_xy_m"] <= P95_ERR_XY_M, \
            f"p95_err_xy {summary['p95_err_xy_m']:.3f} m > {P95_ERR_XY_M} m gate"
        assert summary["max_err_xy_m"] <= MAX_ERR_XY_M, \
            f"max_err_xy {summary['max_err_xy_m']:.3f} m > {MAX_ERR_XY_M} m gate"
        assert summary["p95_err_yaw_rad"] <= P95_ERR_YAW_RAD, \
            f"p95_err_yaw {summary['p95_err_yaw_rad']:.3f} rad > {P95_ERR_YAW_RAD} rad gate"
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    test_waypoint_precision()
