"""Unit tests for mode_transitions_oracle.py (ROS-free).

Runs without a live ROS graph, so it stays in the default pytest sweep
(the HIL tests skip without SIM_API_HOST; this one always runs).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from mode_transitions_oracle import (
    ModeTransitionRecorder,
    diff_subsequence,
    is_subsequence,
)


def make_payload(mode: str) -> str:
    return json.dumps({"mode": mode, "source": "nav", "zone": "gap",
                       "operator_mode": "nav", "transitions": 0})


def test_recorder_deduplicates_consecutive_same_mode():
    r = ModeTransitionRecorder()
    r.record_transition(make_payload("corridor_nav"))
    r.record_transition(make_payload("corridor_nav"))
    r.record_transition(make_payload("corridor_nav"))
    assert r.modes_seen() == ["corridor_nav"]


def test_recorder_tracks_transition_sequence():
    r = ModeTransitionRecorder()
    for m in ["corridor_nav", "rail_approach_pend", "rail_approach_active",
              "rail_drive", "corridor_nav"]:
        r.record_transition(make_payload(m))
    assert r.modes_seen() == [
        "corridor_nav", "rail_approach_pend", "rail_approach_active",
        "rail_drive", "corridor_nav",
    ]


def test_begin_waypoint_clears_history():
    r = ModeTransitionRecorder()
    r.record_transition(make_payload("corridor_nav"))
    r.record_transition(make_payload("rail_drive"))
    r.begin_waypoint()
    assert r.modes_seen() == []
    r.record_transition(make_payload("corridor_nav"))
    assert r.modes_seen() == ["corridor_nav"]


def test_recorder_ignores_malformed_json():
    r = ModeTransitionRecorder()
    r.record_transition("not json")
    r.record_transition("")
    r.record_transition("null")
    assert r.modes_seen() == []


def test_recorder_ignores_missing_mode_field():
    r = ModeTransitionRecorder()
    r.record_transition(json.dumps({"foo": "bar"}))
    r.record_transition(json.dumps({"mode": 42}))  # wrong type
    assert r.modes_seen() == []


def test_is_subsequence_happy_path():
    assert is_subsequence(
        ["corridor_nav", "rail_approach_pend", "rail_drive"],
        ["corridor_nav", "rail_approach_pend", "rail_approach_active",
         "rail_drive", "corridor_nav"],
    )


def test_is_subsequence_rejects_wrong_order():
    assert not is_subsequence(
        ["rail_drive", "rail_approach_pend"],
        ["corridor_nav", "rail_approach_pend", "rail_drive"],
    )


def test_is_subsequence_rejects_missing_element():
    assert not is_subsequence(
        ["rail_approach_pend", "rail_drive"],
        ["corridor_nav", "rail_drive"],
    )


def test_diff_subsequence_reports_missing():
    ok, missing, extras = diff_subsequence(
        ["rail_approach_pend", "rail_drive"],
        ["corridor_nav", "rail_drive"],
    )
    assert not ok
    assert missing == ["rail_approach_pend"]


def test_diff_subsequence_reports_extras():
    ok, missing, extras = diff_subsequence(
        ["corridor_nav", "rail_drive"],
        ["corridor_nav", "blocked_handoff", "rail_approach_pend", "rail_drive"],
    )
    assert ok
    # extras are the observed items not consumed to satisfy expected.
    assert "blocked_handoff" in extras
    assert "rail_approach_pend" in extras


def test_empty_expected_is_trivially_satisfied():
    assert is_subsequence([], ["corridor_nav"])
    ok, missing, extras = diff_subsequence([], ["corridor_nav"])
    assert ok
    assert missing == []
