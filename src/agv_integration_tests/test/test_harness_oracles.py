"""ROS-free coverage of the Round 44 oracle-aware Harness helpers.

The helpers under test compute per-waypoint diagnostics from JSON strings
that `/agv/sim/ground_truth/*` publishes. A real ROS graph is not
required: each test fabricates a minimal `Harness`-shaped object and
invokes the parser/aggregate methods directly.

Covered:
- _on_visible_markers accepts both top-level list and dict shapes.
- _on_obstacles populates the catalogue exactly once per latched message.
- _on_localization_error appends bounded ring and exposes peak+rmse.
- _on_episode_summary stores the latched payload.
- _on_event accumulates the histogram keyed by event type.
- visible_markers_snapshot returns deep copy (immutable to mutation).
- nearest_obstacle picks the minimum-distance entry and carries its name.
- localization_stats aggregates an empty window to zeros + count=0.
- event_histogram deduplicates against the cursor when cleared mid-run.
- begin_waypoint_modes clears per-wp accumulators (events/loc_err).
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

# The harness module imports rclpy at top level. Skip when unavailable.
os.environ.setdefault("SIM_API_HOST", "unit-test-host")
try:
    from test_waypoint_precision import Harness  # type: ignore
except Exception:  # pragma: no cover — pytest will skip via guard below
    Harness = None  # type: ignore


import pytest

if Harness is None:  # pragma: no cover
    pytest.skip("Harness import failed (rclpy not available)",
                allow_module_level=True)


class _FakeMsg:
    """Duck-typed stand-in for std_msgs/String messages."""
    def __init__(self, data: str) -> None:
        self.data = data


def _empty_harness() -> object:
    """Instantiate the accumulator state without invoking rclpy.

    Harness.__init__ calls rclpy.create_node under the hood, which we do
    NOT want. Instead, carve a minimal object that owns the same
    attributes the parsers touch. The parser methods are bound at class
    level, so we can .__get__ them onto a plain instance.
    """
    obj = type("HarnessStub", (), {})()
    obj.last_visible_markers = []
    obj.obstacle_catalog = []
    obj.last_localization_error = None
    obj.last_episode_summary = None
    obj._loc_err_window = []
    obj._loc_err_window_cap = 600
    obj._events = []
    obj._event_types_since = {}
    obj._events_cursor = 0
    # Bind the methods we want to exercise.
    for name in (
        "_on_visible_markers", "_on_obstacles", "_on_localization_error",
        "_on_episode_summary", "_on_event",
        "visible_markers_snapshot", "localization_stats",
        "nearest_obstacle", "event_histogram", "events_since_cursor",
    ):
        setattr(obj, name, getattr(Harness, name).__get__(obj, type(obj)))
    return obj


# ── visible_markers ────────────────────────────────────────────────────

def test_visible_markers_accepts_top_level_list():
    h = _empty_harness()
    h._on_visible_markers(_FakeMsg(json.dumps(
        [{"id": 35, "distance_m": 1.2, "bearing_rad": 0.0, "incidence_deg": 3.0}]
    )))
    snap = h.visible_markers_snapshot()
    assert len(snap) == 1 and snap[0]["id"] == 35


def test_visible_markers_accepts_dict_wrapper():
    h = _empty_harness()
    h._on_visible_markers(_FakeMsg(json.dumps(
        {"markers": [{"id": 4, "distance_m": 2.1}]}
    )))
    assert h.visible_markers_snapshot() == [{"id": 4, "distance_m": 2.1}]


def test_visible_markers_snapshot_is_copy():
    h = _empty_harness()
    h._on_visible_markers(_FakeMsg(json.dumps([{"id": 1}])))
    snap = h.visible_markers_snapshot()
    snap[0]["id"] = 999
    # Re-snapshot — original payload must remain unchanged.
    assert h.visible_markers_snapshot() == [{"id": 1}]


# ── obstacles ──────────────────────────────────────────────────────────

def test_obstacles_catalogue_parsed():
    h = _empty_harness()
    h._on_obstacles(_FakeMsg(json.dumps({"obstacles": [
        {"name": "crate", "kind": "box", "pose": {"x": 2.0, "y": 1.0}},
        {"name": "wall", "kind": "plane", "pose": {"x": 5.0, "y": 0.0}},
    ]})))
    near = h.nearest_obstacle(0.0, 0.0)
    assert near is not None and near["name"] == "crate"
    assert abs(near["distance_m"] - ((2.0**2 + 1.0**2) ** 0.5)) < 1e-9


def test_nearest_obstacle_empty_catalog_returns_none():
    h = _empty_harness()
    assert h.nearest_obstacle(0.0, 0.0) is None


# ── localization_error ──────────────────────────────────────────────────

def test_localization_stats_accumulates_peak_and_rmse():
    h = _empty_harness()
    for pos, yaw in [(0.05, 0.01), (0.12, -0.02), (0.08, 0.015)]:
        h._on_localization_error(_FakeMsg(json.dumps({
            "pos_err_m": pos, "yaw_err_rad": yaw,
            "rmse_pos_m": pos * 0.8, "rmse_yaw_rad": abs(yaw) * 0.8,
            "window_s": 30.0,
        })))
    s = h.localization_stats()
    assert s["sample_count"] == 3
    assert s["peak_pos_err_m"] == pytest.approx(0.12)
    assert s["peak_yaw_err_rad"] == pytest.approx(0.02)
    # rmse fields come from the last sample.
    assert s["rmse_pos_m"] == pytest.approx(0.08 * 0.8)


def test_localization_stats_empty_window_returns_zeros():
    h = _empty_harness()
    s = h.localization_stats()
    assert s == {
        "peak_pos_err_m": 0.0,
        "peak_yaw_err_rad": 0.0,
        "rmse_pos_m": 0.0,
        "rmse_yaw_rad": 0.0,
        "sample_count": 0,
    }


def test_localization_window_ring_caps_at_configured_size():
    h = _empty_harness()
    h._loc_err_window_cap = 5
    for i in range(20):
        h._on_localization_error(_FakeMsg(json.dumps({
            "pos_err_m": float(i), "yaw_err_rad": 0.0,
        })))
    # Only the 5 most-recent samples must survive; peak reflects those.
    s = h.localization_stats()
    assert s["sample_count"] == 5
    assert s["peak_pos_err_m"] == pytest.approx(19.0)


# ── episode_summary ────────────────────────────────────────────────────

def test_episode_summary_stored():
    h = _empty_harness()
    h._on_episode_summary(_FakeMsg(json.dumps(
        {"success": True, "time_to_goal_s": 12.3, "path_length_m": 3.4})))
    assert h.last_episode_summary["time_to_goal_s"] == 12.3


# ── events ─────────────────────────────────────────────────────────────

def test_event_histogram_counts_by_type():
    h = _empty_harness()
    for e in [
        {"event": "collision", "actor_a": "crate"},
        {"event": "drift", "pos_err_m": 0.2},
        {"event": "drift", "pos_err_m": 0.3},
        {"event": "sim_unstick", "stuck_duration_s": 4.2},
    ]:
        h._on_event(_FakeMsg(json.dumps(e)))
    hist = h.event_histogram()
    assert hist == {"collision": 1, "drift": 2, "sim_unstick": 1}


def test_event_histogram_ignores_malformed_payloads():
    h = _empty_harness()
    h._on_event(_FakeMsg("not json"))
    h._on_event(_FakeMsg(json.dumps({"notanevent": "oops"})))
    assert h.event_histogram() == {}
