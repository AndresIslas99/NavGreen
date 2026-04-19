#!/usr/bin/env python3
"""
test_full_flow — end-to-end HIL validation: mapping -> save -> load ->
20-waypoint precision.

Gate (specs/acceptance.yaml#hil_validation.full_flow):
  map_fidelity pass (map_diff.py gates)
  waypoint_precision pass (test_waypoint_precision gates)
  /agv/localization/state reaches LOCALIZED within 15 s after load

Because the mapping drive itself is long-running (several minutes of
scripted teleop), this test accepts a pre-built map by name. The mapping
step is performed by scripts/run_mapping_trajectory.py (or equivalently,
by hand) BEFORE invoking this test. Rationale: CI cannot reliably drive
a SLAM session within a single pytest timeout, and re-driving is
wasteful when the user already has a good map.

Env:
  MAP_NAME           required — map to validate + navigate against
  SIM_API_HOST       required — sim host IP (same as test_waypoint_precision)
  ROS_DOMAIN_ID=42   required
  AGV_DATA_DIR       default $HOME/agv_data
  SKIP_MAP_DIFF=1    optional — skip the geometry comparison and only run
                     the waypoint precision phase (if map_diff dependencies
                     like scipy are not installed).
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Optional

import pytest

try:
    import yaml
except ImportError:
    pytest.skip("python3-yaml not installed", allow_module_level=True)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String
    from agv_interfaces.srv import LoadMap
except ImportError:
    pytest.skip("rclpy / agv_interfaces not available", allow_module_level=True)


SIM_API_HOST = os.environ.get("SIM_API_HOST")
SIM_API_PORT = int(os.environ.get("SIM_API_PORT", "8090"))
MAP_NAME = os.environ.get("MAP_NAME")
AGV_DATA_DIR = Path(os.environ.get("AGV_DATA_DIR", str(Path.home() / "agv_data")))
SKIP_MAP_DIFF = os.environ.get("SKIP_MAP_DIFF", "") == "1"

MAP_ARTIFACT_SUFFIXES = (".pgm", ".yaml")
LOCALIZED_TIMEOUT_S = 15.0

HERE = Path(__file__).parent
MAP_DIFF_SCRIPT = (HERE.parent / "scripts" / "map_diff.py").resolve()
PRECISION_TEST = HERE / "test_waypoint_precision.py"


def _check_preconditions() -> None:
    if not SIM_API_HOST:
        pytest.skip("SIM_API_HOST env var required. See docs/validation/RUNBOOK_lan_hil.md.")
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        pytest.skip("ROS_DOMAIN_ID must be 42.")
    if not MAP_NAME:
        pytest.skip("MAP_NAME env var required — pre-build a map and pass its name.")
    for suffix in MAP_ARTIFACT_SUFFIXES:
        p = AGV_DATA_DIR / "maps" / f"{MAP_NAME}{suffix}"
        if not p.is_file():
            pytest.skip(f"map artifact missing: {p}. Run the mapping phase first.")
    try:
        urllib.request.urlopen(f"http://{SIM_API_HOST}:{SIM_API_PORT}/state", timeout=3.0).read()
    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as e:
        pytest.skip(f"sim_api unreachable: {e}")


class LocWatcher(Node):
    def __init__(self) -> None:
        super().__init__("full_flow_loc_watcher")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.last: Optional[str] = None
        self.create_subscription(String, "/agv/localization/state", self._on, qos)

    def _on(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.last = payload.get("action")
        except json.JSONDecodeError:
            self.last = None


def _snapshot_obstacles_to_file(out_path: Path, timeout_s: float = 10.0) -> bool:
    """Subscribe once to /agv/sim/ground_truth/obstacles (latched) and write JSON."""
    rclpy.init()
    try:
        node = rclpy.create_node("full_flow_obs_snapshot")
        received: list[str] = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(
            String, "/agv/sim/ground_truth/obstacles",
            lambda m: received.append(m.data), qos,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not received:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if not received:
            return False
        out_path.write_text(received[0])
        return True
    finally:
        rclpy.shutdown()


def _call_load_map(map_name: str, timeout_s: float = 30.0) -> bool:
    rclpy.init()
    try:
        node = rclpy.create_node("full_flow_load_map_caller")
        cli = node.create_client(LoadMap, "/agv/map_manager/load_map")
        if not cli.wait_for_service(timeout_sec=timeout_s):
            node.destroy_node()
            return False
        req = LoadMap.Request()
        req.name = map_name
        fut = cli.call_async(req)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not fut.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if not fut.done():
            return False
        resp = fut.result()
        return bool(resp and resp.success)
    finally:
        rclpy.shutdown()


def _wait_for_localized(timeout_s: float = LOCALIZED_TIMEOUT_S) -> bool:
    rclpy.init()
    try:
        watcher = LocWatcher()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(watcher, timeout_sec=0.1)
            if watcher.last == "LOCALIZED":
                watcher.destroy_node()
                return True
        watcher.destroy_node()
        return False
    finally:
        rclpy.shutdown()


def _run_map_diff(out_dir: Path) -> tuple[bool, dict]:
    obs_file = out_dir / "obstacles_snapshot.json"
    if not _snapshot_obstacles_to_file(obs_file):
        return False, {"error": "no obstacles snapshot received"}
    pgm = AGV_DATA_DIR / "maps" / f"{MAP_NAME}.pgm"
    yml = AGV_DATA_DIR / "maps" / f"{MAP_NAME}.yaml"
    cmd = [
        sys.executable, str(MAP_DIFF_SCRIPT),
        "--pgm", str(pgm),
        "--yaml", str(yml),
        "--obstacles-json", str(obs_file),
        "--out", str(out_dir),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    report_path = out_dir / f"map_diff_{MAP_NAME}.json"
    if report_path.is_file():
        data = json.loads(report_path.read_text())
    else:
        data = {"error": "map_diff did not produce a report",
                "stdout_tail": proc.stdout[-2000:],
                "stderr_tail": proc.stderr[-2000:]}
    return proc.returncode == 0, data


def _run_precision(out_dir: Path) -> tuple[bool, dict]:
    # Delegate to the same body as test_waypoint_precision by invoking
    # pytest on that file. The test writes its own report.json to
    # AGV_DATA_DIR/sim_episodes/precision_run_<ts>/; we locate the
    # newest.
    env = dict(os.environ)
    cmd = [
        sys.executable, "-m", "pytest", "-q", "-s",
        str(PRECISION_TEST),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    runs_dir = AGV_DATA_DIR / "sim_episodes"
    if runs_dir.is_dir():
        subs = sorted(runs_dir.glob("precision_run_*"), key=lambda p: p.stat().st_mtime)
        if subs:
            rpt = subs[-1] / "report.json"
            if rpt.is_file():
                return proc.returncode == 0, json.loads(rpt.read_text())
    return proc.returncode == 0, {
        "error": "no precision report found",
        "stdout_tail": proc.stdout[-2000:],
        "stderr_tail": proc.stderr[-2000:],
    }


def test_full_flow() -> None:
    _check_preconditions()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = AGV_DATA_DIR / "sim_episodes" / f"full_flow_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)

    summary = {
        "run_id": f"full_flow_{ts}",
        "map_name": MAP_NAME,
        "stages": {},
        "pass": False,
    }

    # Stage 1: map_diff (fidelity).
    if not SKIP_MAP_DIFF:
        md_ok, md_data = _run_map_diff(out_dir)
        summary["stages"]["map_fidelity"] = {"pass": md_ok, "report": md_data}
    else:
        summary["stages"]["map_fidelity"] = {"pass": None, "skipped": True}

    # Stage 2: load map + wait for LOCALIZED.
    load_ok = _call_load_map(MAP_NAME)
    loc_ok = load_ok and _wait_for_localized()
    summary["stages"]["load_and_localize"] = {
        "load_map_ok": load_ok,
        "localized_within_s": LOCALIZED_TIMEOUT_S,
        "localized": loc_ok,
    }

    # Stage 3: waypoint precision.
    if loc_ok:
        prec_ok, prec_data = _run_precision(out_dir)
        summary["stages"]["waypoint_precision"] = {"pass": prec_ok, "report": prec_data}
    else:
        summary["stages"]["waypoint_precision"] = {"pass": False, "skipped_reason": "not localized"}

    # Final pass: all stages pass (map_fidelity can be None if skipped).
    mf = summary["stages"]["map_fidelity"].get("pass")
    wp = summary["stages"]["waypoint_precision"].get("pass")
    summary["pass"] = bool(wp) and (mf is None or mf is True) and loc_ok

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
    print(json.dumps(summary, indent=2, default=str))

    assert loc_ok, "LOCALIZED not reached within timeout after load_map"
    if not SKIP_MAP_DIFF:
        assert mf is True, f"map_fidelity gate failed: {summary['stages']['map_fidelity']}"
    assert wp is True, "waypoint_precision gate failed — see report"


if __name__ == "__main__":
    test_full_flow()
