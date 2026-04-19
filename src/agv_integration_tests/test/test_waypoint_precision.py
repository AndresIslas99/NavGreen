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
    import tf2_ros
except ImportError:
    pytest.skip("rclpy / nav2_msgs / tf2_ros not available",
                allow_module_level=True)

try:
    from robot_localization.srv import SetPose
    _HAS_SETPOSE = True
except ImportError:
    SetPose = None
    _HAS_SETPOSE = False

try:
    from std_srvs.srv import Trigger as _TriggerSrv
    _HAS_TRIGGER = True
except ImportError:
    _TriggerSrv = None
    _HAS_TRIGGER = False

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
    # Phase 2 G: which driver was dispatched for this waypoint.
    dispatch_used: Optional[str] = None
    # Round 44 Q3: oracle-derived diagnostics. All optional so legacy
    # report.json consumers stay backward-compatible.
    localization: Optional[dict] = None             # peak + rmse stats
    visible_markers_at_start: Optional[list] = None
    visible_markers_at_end: Optional[list] = None
    nearest_obstacle_at_end: Optional[dict] = None
    event_histogram: Optional[dict] = None          # {collision, drift, watchdog_recovery, sim_unstick, ...}
    sim_api_snapshots: Optional[dict] = None        # {pre: {...}, post: {...}, reset: {...}}
    episode_summary: Optional[dict] = None
    snapshot_paths: Optional[dict] = None           # {fail_jpg, events_json} when failure
    reset_wait_ms: Optional[float] = None


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
    """Legacy two-call arm sequence — kept as fallback when /motor/prepare
    is unavailable. iter-22 workflow prefers _motor_prepare() below.
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


def _motor_prepare(verbose: bool = False) -> dict:
    """Iter-22: atomic arm-and-clear via /motor/prepare. The sim handler
    clears e_stop, arms motors, and confirms via /agv/motor_state before
    returning. Response: {"ok": bool, "armed": bool, "e_stop": bool,
    "wait_ms": int}. Replaces the previous two-call sequence which had a
    50-100 ms race window where cmd_vel_rail could leak through.
    """
    try:
        resp = _post_sim_api("/motor/prepare", {}, timeout=5.0)
        if verbose:
            print(f"[motor_prepare] {resp}")
        return resp
    except Exception as e:
        if verbose:
            print(f"[motor_prepare] warn: {e}; falling back to legacy arm")
        _arm_and_clear_estop(verbose=verbose)
        return {"ok": False, "armed": False, "e_stop": True, "wait_ms": 0}


def _sim_clock_pause() -> bool:
    """Iter-22: pause physics without killing watchdog. sim_time freezes
    until /sim/clock_resume. Used to eliminate post-teleport race windows
    by completing all brain-side setup before physics resumes.
    """
    return _post_sim_api_empty("/sim/clock_pause", timeout=3.0)


def _sim_clock_resume() -> bool:
    """Iter-22: resume physics after setup complete."""
    return _post_sim_api_empty("/sim/clock_resume", timeout=3.0)


def _rail_driver_cancel_goal(harness: Node, timeout_s: float = 2.0) -> bool:
    """Iter-22: call /agv/rail_driver/cancel_goal to clear have_goal_
    synchronously in rail_driver. Replaces the iter-7/8/9 hack of
    publishing a zero-distance PoseStamped as "cancel". Expected to be
    idempotent — safe to call even when rail_driver has no goal.
    Returns True on successful response, False on timeout or error.
    """
    if not _HAS_TRIGGER:
        return False
    client = harness.create_client(_TriggerSrv, "/agv/rail_driver/cancel_goal")
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False
        req = _TriggerSrv.Request()
        fut = client.call_async(req)
        # Spin the harness node while waiting so the future resolves.
        deadline = time.monotonic() + timeout_s
        while not fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(harness, timeout_sec=0.1)
        if not fut.done():
            return False
        result = fut.result()
        return bool(result and getattr(result, "success", False))
    except Exception:
        return False
    finally:
        harness.destroy_client(client)


def _wait_for_fresh_tf(
    harness: Node,
    target_frame: str = "base_link",
    source_frame: str = "map",
    consecutive_hits: int = 5,
    timeout_s: float = 3.0,
    poll_interval_s: float = 0.05,
) -> bool:
    """Iter-22 brain 1.3: gate a nav2 dispatch behind a fresh-TF check.

    Nav2's rotation_shim_controller does lookupTransform(map, base_link,
    now) on every tick. Post-teleport there is a 100–300 ms window where
    ekf_global has not yet published the map→odom matching the new robot
    pose; during that window the shim logs "Failed to transform pose to
    base frame!" and MPPI never commands motion. If the harness
    dispatches into that window, the robot stalls and stall_abort
    (>90 s no delta) returns ABORTED with err ≈ full distance.

    This helper polls `can_transform(source, target, Time())` on the
    Harness's tf_buffer and requires `consecutive_hits` successes in a
    row before declaring the TF fresh. Returns True on success, False on
    timeout.

    Only the nav2 dispatch path needs this — rail_approach uses its own
    camera→base TF chain (separate from map→odom) and rail_driver /
    rail_exit operate on odom frame directly.
    """
    deadline = time.monotonic() + timeout_s
    hits = 0
    # Drain any pending TF messages first.
    rclpy.spin_once(harness, timeout_sec=0.1)
    while time.monotonic() < deadline:
        try:
            ok = harness.tf_buffer.can_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except Exception:
            ok = False
        if ok:
            hits += 1
            if hits >= consecutive_hits:
                return True
        else:
            hits = 0
        rclpy.spin_once(harness, timeout_sec=poll_interval_s)
    return False


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


# Round 44 iter-5: detect sim-side self-heal and wait for brain TF cache
# to catch up to the new sim_time before issuing the next goal.
#
# The sim exposes (commit dd2cf06):
#   self_heal_restarts_total — monotonic restart count
#   clock_healthy_streak_s   — seconds since last /clock gap >2 s
#
# Pattern: before each waypoint, compare current restart count vs the
# last observed value; if incremented, pause until the streak is >=
# SIM_STABLE_STREAK_S so NEW TF messages have populated the brain's
# buffer at the new sim_time range.
SIM_STABLE_STREAK_S = 30.0
SIM_STABLE_POLL_TIMEOUT_S = 180.0


def _wait_sim_healthy(last_restarts: int) -> int:
    """Block until the sim has been publishing /clock cleanly for ≥30 s.

    Returns the latest `self_heal_restarts_total`, for the caller to
    carry forward as the next baseline. When the sim is already healthy
    (no recent restart, long streak) this returns immediately.

    If `/sim/telemetry` is unavailable, skip with a warning — we cannot
    validate without the new fields but the waypoint itself can still
    run (degraded detection, not a blocker).
    """
    deadline = time.monotonic() + SIM_STABLE_POLL_TIMEOUT_S
    warned = False
    while time.monotonic() < deadline:
        t = _get_sim_api("/sim/telemetry", timeout=3.0)
        if t is None:
            if not warned:
                print("[sim-heal] telemetry unreachable; skipping health gate", flush=True)
                warned = True
            return last_restarts
        rc = int(t.get("self_heal_restarts_total", 0))
        streak = float(t.get("clock_healthy_streak_s", 0.0))
        if "clock_healthy_streak_s" not in t:
            # Older sim build — no streak field, just use restart count.
            return rc
        if rc > last_restarts:
            print(f"[sim-heal] detected restart (#{rc}); waiting for "
                  f"clock_healthy_streak_s≥{SIM_STABLE_STREAK_S}s "
                  f"(current {streak:.1f}s)", flush=True)
            last_restarts = rc
            time.sleep(5.0)
            continue
        if streak >= SIM_STABLE_STREAK_S:
            return rc
        # Newly-booted sim, still warming up — short poll until streak
        # reaches the threshold.
        time.sleep(2.0)
    print(f"[sim-heal] WARN: still not streak-healthy after "
          f"{SIM_STABLE_POLL_TIMEOUT_S:.0f}s; proceeding anyway", flush=True)
    return last_restarts


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
        # Phase 2: raw state JSON strings — consumed by _wait_for_state_value.
        self.last_rail_approach_state: Optional[str] = None
        self.last_rail_driver_state: Optional[str] = None
        # Phase 2: lazy-created publisher for /agv/rail_driver/goal.
        self._rail_goal_pub = None

        # Round 44 Q1: sim-side ground-truth oracle caches. Parsed once
        # per message; the harness does not dedupe identical payloads.
        self.last_visible_markers: list[dict] = []  # [{id, distance_m, bearing_rad, incidence_deg}]
        self.obstacle_catalog: list[dict] = []      # latched once; static list
        self._loc_err_window: list[dict] = []       # bounded-length ring of recent localization_error samples
        self._loc_err_window_cap = 600              # ~10 min at 1 Hz
        self.last_localization_error: Optional[dict] = None
        self.last_episode_summary: Optional[dict] = None
        # Round 44 Q1: per-waypoint accumulators, cleared by begin_waypoint_modes.
        self._event_types_since: dict[str, int] = {}
        self._events_cursor: int = 0

        # Iter-22 brain 1.3: TF buffer for the fresh-TF gate pre-nav2 dispatch.
        # wp02 flakiness (~50 % ABORT 1.0 m) traced to Nav2
        # rotation_shim_controller's lookupTransform(map, base_link, now)
        # failing during the 100–300 ms post-teleport window where
        # ekf_global has not yet emitted an up-to-date map→odom. Gating
        # on this buffer avoids dispatching into a stale TF state.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
        # Phase 2: rail_approach + rail_driver state subscriptions feed the
        # dispatch router (_wait_for_state_value) and the reporter.
        # Iter-20: state subscriptions use depth=1 (latest-only) so the
        # harness's _wait_for_state_value never reads a buffered stale
        # message. Previous depth=10 reliable_qos kept up to 10 messages
        # of history: after a rail_approach ABORTED for a prior waypoint
        # (iter-17's SETTLED/ABORTED fix kept the terminal label
        # latched until the next execute call), the queue held 5–10
        # "aborted" messages that were still being processed one per
        # rclpy.spin_once call in the polling loop. The first poll in
        # the next waypoint's wait matched on the stale "aborted" and
        # reported ABORTED dur=0.0s (observed wp07/wp11/wp12 across
        # iter-17/18/19). Depth-1 KEEP_LAST guarantees the callback
        # always sees the newest publish.
        latest_only_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String,
            "/agv/rail_approach/state",
            lambda m: setattr(self, "last_rail_approach_state", m.data),
            latest_only_qos,
        )
        self.create_subscription(
            String,
            "/agv/rail_driver/state",
            lambda m: setattr(self, "last_rail_driver_state", m.data),
            latest_only_qos,
        )

        # Round 44 Q1: sim-side ground-truth oracle subscriptions. visible_markers
        # + localization_error stream at 5/1 Hz; obstacles is latched (catalogue
        # parsed once). episode_summary is latched per Nav2 episode.
        self.create_subscription(
            String,
            "/agv/sim/ground_truth/visible_markers",
            self._on_visible_markers,
            reliable_qos,
        )
        self.create_subscription(
            String,
            "/agv/sim/ground_truth/obstacles",
            self._on_obstacles,
            latched_qos,
        )
        self.create_subscription(
            String,
            "/agv/sim/localization_error",
            self._on_localization_error,
            reliable_qos,
        )
        self.create_subscription(
            String,
            "/agv/sim/episode_summary",
            self._on_episode_summary,
            latched_qos,
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
        # Round 44 Q1: accumulate type histogram for the current waypoint.
        # begin_waypoint_modes() resets the counters at the start of each wp.
        etype = payload.get("event")
        if isinstance(etype, str):
            self._event_types_since[etype] = self._event_types_since.get(etype, 0) + 1

    # ── Round 44 Q1: sim-side oracle parsers ──────────────────────────

    def _on_visible_markers(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        # Sim wire format: {"t_sim": ..., "robot_pose": [...],
        #                   "camera_pose": [...], "count": N, "visible": [...]}
        # Also accept top-level list / {"markers": [...]} for other oracles.
        if isinstance(data, list):
            markers = data
        elif isinstance(data, dict):
            markers = data.get("visible", data.get("markers", []))
        else:
            markers = []
        if isinstance(markers, list):
            self.last_visible_markers = [m for m in markers if isinstance(m, dict)]

    def _on_obstacles(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        # Sim wire format: {"t_sim": ..., "source": "...", "count": N,
        #                   "static_obstacles": [...]}. Also accept
        # top-level list or {"obstacles": [...]} for robustness.
        if isinstance(data, list):
            items = data
        elif isinstance(data, dict):
            items = data.get("static_obstacles",
                             data.get("obstacles", []))
        else:
            items = []
        if isinstance(items, list):
            self.obstacle_catalog = [o for o in items if isinstance(o, dict)]

    def _on_localization_error(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(data, dict):
            return
        data["_received_wall_s"] = time.time()
        self.last_localization_error = data
        self._loc_err_window.append(data)
        if len(self._loc_err_window) > self._loc_err_window_cap:
            # Drop oldest; ring behaviour.
            self._loc_err_window = self._loc_err_window[-self._loc_err_window_cap:]

    def _on_episode_summary(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(data, dict):
            self.last_episode_summary = data

    def _on_mode_state(self, msg: String) -> None:
        self._mode_recorder.record_transition(msg.data)

    def begin_waypoint_modes(self) -> None:
        self._mode_recorder.begin_waypoint()
        # Round 44 Q1: the per-waypoint diagnostic accumulators share the
        # same lifecycle as the mode recorder.
        self._event_types_since = {}
        self._events_cursor = len(self._events)
        self._loc_err_window = []  # bounded localization-error stream for this wp

    def modes_observed(self) -> list:
        return self._mode_recorder.modes_seen()

    # ── Round 44 Q1: oracle-derived snapshots + aggregates ─────────────

    def visible_markers_snapshot(self) -> list:
        """Return the latest visible_markers (deep-copied, no system fields)."""
        return [dict(m) for m in self.last_visible_markers]

    def localization_stats(self) -> dict:
        """Aggregate peak + rmse over the localization_error ring buffer.

        Returns a dict with peak_pos_err_m, peak_yaw_err_rad, rmse_pos_m,
        rmse_yaw_rad, sample_count. All-zero + count=0 when no samples
        arrived during the waypoint.
        """
        samples = self._loc_err_window
        if not samples:
            return {
                "peak_pos_err_m": 0.0,
                "peak_yaw_err_rad": 0.0,
                "rmse_pos_m": 0.0,
                "rmse_yaw_rad": 0.0,
                "sample_count": 0,
            }
        pos = [float(s.get("pos_err_m", 0.0)) for s in samples]
        yaw = [float(s.get("yaw_err_rad", 0.0)) for s in samples]
        return {
            "peak_pos_err_m": max(pos) if pos else 0.0,
            "peak_yaw_err_rad": max(abs(v) for v in yaw) if yaw else 0.0,
            # Prefer the node's own rolling RMSE when available (it's
            # computed over its own window_s), else fall back to the last
            # sample field the node exposes.
            "rmse_pos_m": float(samples[-1].get("rmse_pos_m", 0.0)),
            "rmse_yaw_rad": float(samples[-1].get("rmse_yaw_rad", 0.0)),
            "sample_count": len(samples),
        }

    def nearest_obstacle(self, x: float, y: float) -> Optional[dict]:
        """Find the closest entry in obstacle_catalog to (x, y).

        Returns {name, kind, distance_m, obstacle_x, obstacle_y} or None
        when the catalog is empty (oracle not yet received).
        """
        if not self.obstacle_catalog:
            return None
        best = None
        best_d = float("inf")
        for o in self.obstacle_catalog:
            # Expected shape per spec: {name, kind, pose: {x, y}, size: {...}}
            pose = o.get("pose") if isinstance(o.get("pose"), dict) else o
            ox = float(pose.get("x", 0.0))
            oy = float(pose.get("y", 0.0))
            d = math.hypot(x - ox, y - oy)
            if d < best_d:
                best_d = d
                best = {
                    "name": o.get("name"),
                    "kind": o.get("kind"),
                    "distance_m": d,
                    "obstacle_x": ox,
                    "obstacle_y": oy,
                }
        return best

    def event_histogram(self) -> dict:
        """Counts per event type accumulated since begin_waypoint_modes."""
        return dict(self._event_types_since)

    def events_since_cursor(self) -> list:
        """Slice of raw events captured during the current waypoint."""
        return list(self._events[self._events_cursor:])

    def ensure_rail_goal_publisher(self):
        """Lazy-create the one-shot publisher for /agv/rail_driver/goal.

        Transient-local depth=1 so the rail_driver picks up the latest goal
        even if the test races ahead of subscriber discovery.
        """
        if self._rail_goal_pub is None:
            latched = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._rail_goal_pub = self.create_publisher(
                PoseStamped, "/agv/rail_driver/goal", latched)
        return self._rail_goal_pub

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
                       start_yaw: Optional[float] = None,
                       tol_m: float = 0.15,
                       tol_yaw_rad: float = 0.15) -> bool:
        """Wait until the teleport takes effect.

        Trust ONLY the ground truth: the test POSTed /reset with a specific
        pose, so the teleport is complete when GT lands within `tol_m` of
        the target xy AND (when `start_yaw` is provided) `tol_yaw_rad` of
        the target heading. `/agv/sim/reset_done` is unreliable — the
        TRANSIENT_LOCAL latched True from prior sessions fires the callback
        at subscriber creation time, which can race ahead of the new /reset.

        Round 44 iter-6 hardening: the xy tolerance was 30 cm and yaw was
        not validated at all. wp04/wp07/wp10/wp15 passed the gate with
        stale GT (robot still at the previous waypoint's goal) because
        the xy distance happened to be under 30 cm OR the yaw was wrong.
        The tighter 15 cm + 0.15 rad (~8.6°) thresholds match what the
        sim's `/reset` response claims as its own convergence criteria.
        """
        deadline = time.monotonic() + RESET_TIMEOUT_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            gt = self.current_gt_xy()
            if gt is None:
                continue
            d = math.hypot(gt[0] - start_x, gt[1] - start_y)
            if d > tol_m:
                continue
            if start_yaw is None:
                return True
            dyaw = abs(_wrap_angle(gt[2] - start_yaw))
            if dyaw <= tol_yaw_rad:
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
        # /sim/telemetry is the stable liveness endpoint; /state can return
        # 500 transiently when the brain's est_pose pipeline is stale.
        try:
            urllib.request.urlopen(
                f"http://{SIM_API_HOST}:{SIM_API_PORT}/sim/telemetry",
                timeout=timeout_s,
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
    # Ping the sim_api liveness. /sim/telemetry is lightweight and always
    # reflects the sim host's health; /state can throw 500 transiently when
    # the brain's est_pose path is stale, which is NOT a reason to skip the
    # HIL test itself.
    try:
        urllib.request.urlopen(
            f"http://{SIM_API_HOST}:{SIM_API_PORT}/sim/telemetry",
            timeout=3.0).read()
    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as e:
        pytest.skip(f"sim_api at {SIM_API_HOST}:{SIM_API_PORT} unreachable: {e}")


# ── Phase 2: per-waypoint dispatch router ────────────────────────────
# Each waypoint can carry a `dispatch` field to pick the driver that
# navigates to its goal:
#   * "nav2"          — HTTP POST /goal to sim_api → Nav2 action (legacy path).
#   * "rail_approach" — call /agv/rail_approach/execute service with tag_id +
#                       offsets; wait for /agv/rail_approach/state → "settled".
#   * "rail_drive"    — publish PoseStamped to /agv/rail_driver/goal; wait for
#                       /agv/rail_driver/state → "reached".
# When `dispatch` is absent we derive it from the last element of
# `expected_modes` so legacy tagged yaml continues to work.
_DISPATCH_FROM_LAST_MODE = {
    "corridor_nav":         "nav2",
    "rail_approach_pend":   "nav2",        # pre-ACTIVE → Nav2 still owns cmd_vel
    "rail_approach_active": "rail_approach",
    "rail_drive":           "rail_drive",
    "rail_exit":            "rail_exit",
    "blocked_handoff":      "nav2",
    "teleop":               "nav2",
    "idle":                 "nav2",
}
_VALID_DISPATCH = {"nav2", "rail_approach", "rail_drive", "rail_exit"}

# Floor-tag ID lookup for aisles {-4.4, -2.2, 0, +2.2, +4.4} (idx 0..4).
# REAR entry at x=4.0 facing +Z: IDs 33..37.
# FRONT entry at x=7.0 facing +Z: IDs 2, 3, 4, 12, 13.
_REAR_APPROACH_TAGS  = [33, 34, 35, 36, 37]
_FRONT_APPROACH_TAGS = [2,  3,  4,  12, 13]


def _dispatch_for(wp: dict) -> str:
    """Pick a dispatcher for this waypoint.

    Explicit `dispatch` wins. Otherwise derive from `expected_modes[-1]`.
    Fall back to "nav2" for legacy/ambiguous entries.
    """
    explicit = wp.get("dispatch")
    if isinstance(explicit, str) and explicit in _VALID_DISPATCH:
        return explicit
    expected = wp.get("expected_modes") or []
    if expected:
        last = expected[-1]
        if last in _DISPATCH_FROM_LAST_MODE:
            return _DISPATCH_FROM_LAST_MODE[last]
    return "nav2"


def _derive_tag_id(wp: dict) -> Optional[int]:
    """Look up the floor tag for a rail_approach waypoint.

    Priority: explicit `tag_id` → aisle lookup via goal's y and expected zone.
    Returns None when it cannot be resolved (caller reports as failure).
    """
    explicit = wp.get("tag_id")
    if isinstance(explicit, int):
        return explicit
    goal = wp.get("goal") or {}
    goal_x = float(goal.get("x", 0.0))
    goal_y = float(goal.get("y", 0.0))
    # Aisle index by nearest center.
    aisle_centers = [-4.4, -2.2, 0.0, 2.2, 4.4]
    aisle_idx = min(range(len(aisle_centers)),
                    key=lambda i: abs(goal_y - aisle_centers[i]))
    if abs(goal_y - aisle_centers[aisle_idx]) > 0.35:
        return None  # goal not aisle-aligned
    if 3.9 <= goal_x <= 4.6:
        return _REAR_APPROACH_TAGS[aisle_idx]
    if 6.9 <= goal_x <= 7.6:
        return _FRONT_APPROACH_TAGS[aisle_idx]
    return None


def _call_rail_approach(harness: "Harness", tag_id: int, offset_x: float,
                        offset_y: float, timeout_s: float) -> str:
    """Call /agv/rail_approach/execute and wait for the state to settle.

    Returns one of: "SUCCEEDED" (state=="settled"), "ABORTED" (state==
    "aborted"), "NAV_TIMEOUT" (neither within timeout_s), or
    "SERVICE_UNAVAILABLE" (client failed to connect).
    """
    from agv_interfaces.srv import RailApproach
    cli = harness.create_client(RailApproach, "/agv/rail_approach/execute")
    if not cli.wait_for_service(timeout_sec=5.0):
        return "SERVICE_UNAVAILABLE"
    req = RailApproach.Request()
    req.tag_id = int(tag_id)
    req.offset_x = float(offset_x)
    req.offset_y = float(offset_y)
    fut = cli.call_async(req)
    # Spin briefly so the service call doesn't sit in the queue.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not fut.done():
        rclpy.spin_once(harness, timeout_sec=0.1)
    # Poll the state topic for settled/aborted.
    return _wait_for_state_value(
        harness, topic_attr="last_rail_approach_state",
        target_values={"SUCCEEDED": ["settled"], "ABORTED": ["aborted"]},
        timeout_s=timeout_s)


def _publish_rail_goal(harness: "Harness", goal_x: float, goal_y: float,
                       timeout_s: float) -> str:
    """Publish /agv/rail_driver/goal and wait for rail_driver reached.

    Returns "SUCCEEDED" on reached, "ABORTED" on blocked_*, or
    "NAV_TIMEOUT" otherwise.
    """
    pub = harness.ensure_rail_goal_publisher()
    goal = PoseStamped()
    goal.header.stamp = harness.get_clock().now().to_msg()
    goal.header.frame_id = "map"
    goal.pose.position.x = float(goal_x)
    goal.pose.position.y = float(goal_y)
    goal.pose.orientation.w = 1.0
    pub.publish(goal)
    return _wait_for_state_value(
        harness, topic_attr="last_rail_driver_state",
        target_values={
            "SUCCEEDED": ["reached"],
            "ABORTED": ["blocked_lateral", "blocked_misaligned"],
        },
        timeout_s=timeout_s)


def _publish_rail_exit_and_await_corridor(
    harness: "Harness", goal_x: float, goal_y: float, timeout_s: float) -> str:
    """Publish an initial rail_drive goal and observe the full RAIL_EXIT flow.

    Sequence expected (observed via /agv/mode/state):
      1. arbiter transitions to `rail_drive` (shortcut on rail_driver
         state == "driving").
      2. arbiter transitions to `rail_exit` after rail_driver "reached" —
         arbiter publishes the internal push goal 1.5 m past the exit tag.
      3. arbiter releases to `corridor_nav` once rail_exit_clearance_m ≥ 1
         AND the zone is no longer rail/approach.

    Returns:
      "SUCCEEDED" — all three transitions observed within timeout.
      "ABORTED"   — rail_driver reported blocked_* (bail without corridor release).
      "NAV_TIMEOUT" — some step did not complete in time.
    """
    pub = harness.ensure_rail_goal_publisher()
    goal = PoseStamped()
    goal.header.stamp = harness.get_clock().now().to_msg()
    goal.header.frame_id = "map"
    goal.pose.position.x = float(goal_x)
    goal.pose.position.y = float(goal_y)
    goal.pose.orientation.w = 1.0
    pub.publish(goal)

    deadline = time.monotonic() + timeout_s
    saw_rail_exit = False
    saw_corridor_after_exit = False
    rail_exit_index = None
    while time.monotonic() < deadline:
        rclpy.spin_once(harness, timeout_sec=0.1)
        # Early abort on rail_driver blocked.
        driver_payload = getattr(harness, "last_rail_driver_state", None)
        if isinstance(driver_payload, str) and driver_payload:
            try:
                state = json.loads(driver_payload).get("state")
            except (json.JSONDecodeError, TypeError):
                state = None
            if state in ("blocked_lateral", "blocked_misaligned"):
                return "ABORTED"
        modes = harness.modes_observed()
        # Iter-21: track sequence instead of latching on modes[-1].
        # The FSM oscillates rail_exit ↔ corridor_nav ↔ rail_drive during
        # the push phase (observed iter-20 wp13: ~300 ms corridor_nav
        # window sandwiched between rail_exit and rail_drive). At 10 Hz
        # polling the harness often missed the corridor_nav tick because
        # modes[-1] had already advanced. The intent is "release to
        # corridor_nav was observed after rail_exit," so record a sticky
        # flag the moment the pattern appears in the history.
        if "rail_exit" in modes and rail_exit_index is None:
            rail_exit_index = modes.index("rail_exit")
            saw_rail_exit = True
        if saw_rail_exit and rail_exit_index is not None:
            tail = modes[rail_exit_index + 1:]
            if "corridor_nav" in tail:
                saw_corridor_after_exit = True
        if saw_corridor_after_exit:
            return "SUCCEEDED"
    return "NAV_TIMEOUT"


def _wait_for_state_value(harness: "Harness", topic_attr: str,
                           target_values: dict, timeout_s: float) -> str:
    """Poll a harness attribute (JSON string) for a top-level "state" match.

    `target_values` maps a status label ("SUCCEEDED"/"ABORTED"/...) to a
    list of JSON state strings that should return it. Returns
    "NAV_TIMEOUT" on deadline.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(harness, timeout_sec=0.1)
        payload = getattr(harness, topic_attr, None)
        if isinstance(payload, str) and payload:
            # Lightweight JSON extractor (matches the arbiter's C++ helper).
            try:
                parsed = json.loads(payload)
                state = parsed.get("state")
            except (json.JSONDecodeError, TypeError, AttributeError):
                state = None
            if state:
                for status, values in target_values.items():
                    if state in values:
                        return status
    return "NAV_TIMEOUT"


def _run_one_waypoint(
    harness: Harness,
    wp: dict,
    budget: Optional[RestartBudget] = None,
    report_dir: Optional[Path] = None,
) -> WaypointResult:
    wp_id = wp["id"]
    start = wp["start"]
    goal = wp["goal"]
    expected_modes = wp.get("expected_modes")  # may be None for legacy yaml

    # Phase 2: clear the recorder so modes_observed() tracks only this wp.
    harness.begin_waypoint_modes()

    # Round 44 Q2: snapshot sim_api telemetry BEFORE the teleport so the
    # post-waypoint delta captures watchdog/unstick counters for THIS wp.
    pre_telemetry = _get_sim_api("/sim/telemetry", timeout=3.0)

    # 0. Cancel any active Nav2 action from the previous waypoint before
    #    teleporting — otherwise Nav2 keeps driving the robot toward the old
    #    goal during the post-reset settle window (round 18 wp02 symptom).
    _cancel_all_nav_goals(harness)

    # Iter-22 workflow: reset → validate → motor_prepare → cancel_goal →
    # sync → TF-gate → dispatch. No /sim/clock_pause — pausing breaks
    # brain timer callbacks (use_sim_time=true → timer-driven TF and EKF
    # publishing both freeze). The combination of /reset's enhanced
    # readiness flags (velocities_zeroed + encoders_reset) +
    # /motor/prepare (atomic arm) + rail_driver cancel_goal service
    # (iter-22 brain 1.1) closes the post-teleport race without needing
    # to freeze the sim clock.

    # 1. Teleport. Response carries 5 readiness flags.
    t_reset_issue = time.time()
    reset_ok = False
    reset_response: Optional[dict] = None
    for attempt in range(2):
        try:
            reset_response = _post_sim_api(
                "/reset",
                {"x": float(start["x"]), "y": float(start["y"]), "yaw": float(start["yaw"])},
            )
        except (urllib.error.URLError, urllib.error.HTTPError, OSError):
            # sim_api unreachable — probably restarting; fall through to wait.
            pass
        # Prefer the sim's own convergence flags when available. Fall back
        # to the legacy GT-based wait_for_reset if the response is partial.
        if (
            isinstance(reset_response, dict)
            and reset_response.get("pose_converged") is True
            and reset_response.get("velocities_zeroed") is True
            and reset_response.get("encoders_reset") is True
        ):
            reset_ok = True
            break
        if harness.wait_for_reset(
            start_x=float(start["x"]),
            start_y=float(start["y"]),
            start_yaw=float(start["yaw"]),
            tol_m=0.15,
            tol_yaw_rad=0.15,
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
            dispatch_used=_dispatch_for(wp),
        )

    # 1b. Atomic motor prepare: single REST call replaces the old
    #     /e_stop + /motor/enable two-step (which had a 50–100 ms window
    #     where cmd_vel_rail could leak through because motors were
    #     armed before e_stop was cleared).
    prepare_resp = _motor_prepare(verbose=False)
    if not prepare_resp.get("armed", False):
        print(f"[warn {wp_id}] /motor/prepare did not confirm armed: "
              f"{prepare_resp}", flush=True)

    # 1c. Iter-22 brain 1.1: synchronously cancel any stale rail_driver
    #     goal BEFORE physics resumes. This replaces the iter-7/8/9
    #     "publish zero-distance PoseStamped" hack which had a race
    #     with the arbiter's auto-EXIT_PUSH. With the new service the
    #     cancel is atomic: have_goal_ clears in the service callback,
    #     rail_driver emits one tick of state="canceled", then falls to
    #     "idle". Safe to call even when there is no pending goal.
    _rail_driver_cancel_goal(harness, timeout_s=2.0)

    # 2. Settle — still give ekf_local a beat to ingest the first
    #    post-teleport /agv/joint_states (v≈0, per encoders_reset).
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

    # Round 44 Q1: snapshot the markers the robot sees right BEFORE dispatch.
    # This is "at_start" — after teleport + sync, with the camera looking at
    # whatever tag the waypoint targets.
    markers_at_start = harness.visible_markers_snapshot()

    # 4. Navigate — pick the driver based on the waypoint's dispatch.
    t_nav_start = time.time()
    t_events_start = time.time()
    dispatch = _dispatch_for(wp)
    if dispatch == "rail_approach":
        tag_id = _derive_tag_id(wp)
        if tag_id is None:
            status = "DISPATCH_ERROR"
            print(f"    [dispatch] cannot derive tag_id for rail_approach wp; "
                  f"goal={goal}", flush=True)
        else:
            offset_x = float(wp.get("offset_x", 0.3))
            offset_y = float(wp.get("offset_y", 0.0))
            print(f"    [dispatch] rail_approach tag_id={tag_id} "
                  f"offsets=({offset_x:.2f}, {offset_y:.2f})", flush=True)
            # rail_approach runs a Nav2 coarse phase followed by a
            # 20 Hz visual servo to 2 cm. Under sim RTF ~0.20 the full
            # sequence runs ~4 min wall-clock. 1.5× NAV_TIMEOUT matches
            # the rail_exit budget and covers the tail.
            status = _call_rail_approach(
                harness, tag_id, offset_x, offset_y, NAV_TIMEOUT_S * 1.5)
    elif dispatch == "rail_drive":
        print(f"    [dispatch] rail_drive goal=({goal['x']:.2f}, "
              f"{goal['y']:.2f})", flush=True)
        status = _publish_rail_goal(
            harness, float(goal["x"]), float(goal["y"]), NAV_TIMEOUT_S)
    elif dispatch == "rail_exit":
        # Publish the initial rail_drive goal (at the approach tag), then
        # observe the arbiter's RAIL_DRIVE → RAIL_EXIT → CORRIDOR_NAV flow.
        # The initial goal is the tag position; the arbiter itself emits
        # the follow-up 1.5 m push goal once rail_driver reports "reached".
        print(f"    [dispatch] rail_exit initial_goal=({goal['x']:.2f}, "
              f"{goal['y']:.2f})", flush=True)
        # rail_exit needs longer than rail_drive — two sequential reaches.
        status = _publish_rail_exit_and_await_corridor(
            harness, float(goal["x"]), float(goal["y"]),
            NAV_TIMEOUT_S * 1.5)
    else:
        # Default: Nav2 via sim_api HTTP /goal.
        # Iter-22 brain 1.3: gate on fresh TF before dispatch so Nav2's
        # rotation_shim doesn't fail lookupTransform and stall.
        if not _wait_for_fresh_tf(harness, timeout_s=3.0):
            print(f"    [dispatch] nav2 skipped — map→base_link TF not "
                  f"fresh after 3 s", flush=True)
            status = "TF_NOT_READY"
        else:
            status = harness.navigate_to(
                float(goal["x"]), float(goal["y"]), float(goal["yaw"]),
                NAV_TIMEOUT_S)

    # 5. Snapshot both poses at terminal.
    rclpy.spin_once(harness, timeout_sec=0.2)
    gt_final = harness.current_gt_xy()
    brain_final = harness.current_brain_xy()
    events_during = harness.events_since(t_events_start)

    # Round 44 Q1/Q2: post-dispatch oracle snapshots. visible_markers_at_end
    # captures what was visible at the terminal pose (useful for diagnosing
    # rail_approach that settled on the wrong tag). post_telemetry closes
    # the per-waypoint sim-side delta begun by pre_telemetry above.
    markers_at_end = harness.visible_markers_snapshot()
    post_telemetry = _get_sim_api("/sim/telemetry", timeout=3.0)
    loc_stats = harness.localization_stats()
    event_hist = harness.event_histogram()
    nearest_obs = None
    if gt_final is not None:
        nearest_obs = harness.nearest_obstacle(gt_final[0], gt_final[1])

    # For rail_exit waypoints, the meaningful terminal pose is the push
    # target (≥ 1 m past the tag), not the initial rail_drive goal at the
    # tag. The harness records err_xy against exit_goal when supplied.
    eval_goal_x = goal["x"]
    eval_goal_y = goal["y"]
    eval_goal_yaw = goal["yaw"]
    if dispatch == "rail_exit" and isinstance(wp.get("exit_goal"), dict):
        eg = wp["exit_goal"]
        eval_goal_x = float(eg.get("x", goal["x"]))
        eval_goal_y = float(eg.get("y", goal["y"]))
        # yaw stays the initial goal's — rail_driver kept wz=0 throughout.

    err_xy = None
    err_yaw = None
    brain_drift = None
    if gt_final is not None:
        err_xy = math.hypot(gt_final[0] - eval_goal_x, gt_final[1] - eval_goal_y)
        err_yaw = abs(_wrap_angle(gt_final[2] - eval_goal_yaw))
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

    # Round 44 Q2: on failure, preserve forensic artefacts alongside
    # report.json so iteration_report.py can link to them.
    snapshot_paths: Optional[dict] = None
    if report_dir is not None and status in ("COLLISION", "NAV_TIMEOUT", "ABORTED",
                                              "RESET_TIMEOUT", "DISPATCH_ERROR",
                                              "GOAL_SEND_TIMEOUT"):
        snap_dir = report_dir / "snapshots"
        snap_dir.mkdir(parents=True, exist_ok=True)
        jpg_path = snap_dir / f"{wp_id}_fail.jpg"
        events_path = snap_dir / f"{wp_id}_events.json"
        try:
            url = f"http://{SIM_API_HOST}:{SIM_API_PORT}/snapshot.jpg"
            with urllib.request.urlopen(url, timeout=5.0) as r:
                jpg_path.write_bytes(r.read())
        except (urllib.error.URLError, urllib.error.HTTPError, OSError):
            jpg_path = None
        events_tail = _get_sim_api(
            f"/events?since={t_nav_start:.3f}", timeout=5.0)
        if events_tail is not None:
            try:
                events_path.write_text(json.dumps(events_tail, indent=2))
            except OSError:
                events_path = None
        else:
            events_path = None
        snapshot_paths = {
            "fail_jpg": str(jpg_path) if jpg_path else None,
            "events_json": str(events_path) if events_path else None,
        }

    # Round 44 Q2/Q3: bundle sim_api snapshots for the report.
    sim_api_snapshots = {
        "pre": pre_telemetry,
        "post": post_telemetry,
        "reset": reset_response,
    }
    reset_wait_ms = None
    if isinstance(reset_response, dict):
        w = reset_response.get("wait_ms")
        try:
            reset_wait_ms = float(w) if w is not None else None
        except (TypeError, ValueError):
            reset_wait_ms = None

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
        dispatch_used=dispatch,
        localization=loc_stats,
        visible_markers_at_start=markers_at_start,
        visible_markers_at_end=markers_at_end,
        nearest_obstacle_at_end=nearest_obs,
        event_histogram=event_hist,
        sim_api_snapshots=sim_api_snapshots,
        episode_summary=harness.last_episode_summary,
        snapshot_paths=snapshot_paths,
        reset_wait_ms=reset_wait_ms,
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


def _make_report_dir() -> Path:
    """Create the per-run report directory and refresh `latest/` symlink.

    Round 44 Q7: a stable `latest/` symlink lets iteration_report.py default
    to the newest run without threading the timestamp around.
    """
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = AGV_DATA_DIR / "sim_episodes"
    out_dir = base / f"precision_run_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)
    latest = base / "latest"
    try:
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(out_dir.name)  # relative symlink
    except OSError:
        pass  # filesystem may not support symlinks — non-fatal
    return out_dir


def _write_report(
    results: list[WaypointResult],
    summary: dict,
    out_dir: Path,
    budget: Optional[RestartBudget] = None,
) -> Path:
    report = {
        "run_id": out_dir.name,
        "report_version": 2,
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

        # Round 44 Q2/Q7: the report directory is created up front so each
        # waypoint can drop failure snapshots into its `snapshots/` subdir.
        report_dir = _make_report_dir()
        print(f"[waypoint_precision] report dir: {report_dir}", flush=True)

        # Round 44 iter-5: baseline the sim-side restart counter so a
        # mid-run self-heal is detectable from the first recheck onward.
        _initial = _get_sim_api("/sim/telemetry", timeout=3.0) or {}
        last_sim_restarts = int(_initial.get("self_heal_restarts_total", 0))

        results: list[WaypointResult] = []
        for wp in waypoints[:MIN_WAYPOINTS]:
            # Before each waypoint, verify the sim hasn't silently self-healed.
            # If it has, pause until clock has been flowing cleanly for 30 s,
            # then re-sync the brain EKF to GT so TF lookups aren't left in
            # the previous sim_time window.
            rc = _wait_sim_healthy(last_sim_restarts)
            if rc != last_sim_restarts:
                gt = harness.current_gt_xy()
                if gt is not None:
                    print(f"[sim-heal] re-syncing brain to GT after restart", flush=True)
                    _sync_brain_to_gt(harness, gt[0], gt[1], gt[2], verbose=False)
                    time.sleep(SYNC_SETTLE_S)
                last_sim_restarts = rc
            print(f"[wp {wp['id']}] → ({wp['goal']['x']}, {wp['goal']['y']}, {wp['goal']['yaw']:.2f})")
            results.append(_run_one_waypoint(
                harness, wp, budget=budget, report_dir=report_dir))
            r = results[-1]
            print(f"    {r.status}  err_xy={r.err_xy}  err_yaw={r.err_yaw}  drift={r.brain_drift}  dur={r.duration_s:.1f}s")

        summary = _summarize(results)
        report_path = _write_report(results, summary, report_dir, budget=budget)
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
