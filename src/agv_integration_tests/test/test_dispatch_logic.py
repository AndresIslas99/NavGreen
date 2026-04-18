"""ROS-free unit tests for the dispatch router in test_waypoint_precision.py.

Covers _dispatch_for and _derive_tag_id — they contain the branching logic
between Nav2/rail_approach/rail_drive drivers. Kept in the default pytest
sweep so regressions surface without a live ROS graph.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

# Avoid the file's `import rclpy` at module-load time: the module tries to
# import rclpy unconditionally at the top. Set SIM_API_HOST so the test
# harness skip guards don't bail when importing.
os.environ.setdefault("SIM_API_HOST", "unit-test-host")

# The logic we test lives inside a ROS-coupled module, but _dispatch_for and
# _derive_tag_id are pure. Import the top-level symbols and exercise.
from test_waypoint_precision import (  # type: ignore
    _derive_tag_id,
    _dispatch_for,
)


def test_dispatch_explicit_nav2():
    assert _dispatch_for({"dispatch": "nav2"}) == "nav2"


def test_dispatch_explicit_rail_approach():
    assert _dispatch_for({"dispatch": "rail_approach"}) == "rail_approach"


def test_dispatch_explicit_rail_drive():
    assert _dispatch_for({"dispatch": "rail_drive"}) == "rail_drive"


def test_dispatch_derived_from_last_mode_corridor():
    wp = {"expected_modes": ["corridor_nav"]}
    assert _dispatch_for(wp) == "nav2"


def test_dispatch_derived_from_last_mode_rail_drive():
    wp = {"expected_modes": ["corridor_nav", "rail_drive"]}
    assert _dispatch_for(wp) == "rail_drive"


def test_dispatch_derived_rail_approach_pend_stays_nav2():
    # rail_approach_pend is the intermediate state where Nav2 still drives
    # (coarse_approach is a Nav2 goal issued by rail_approach). So the
    # dispatcher must keep Nav2 at the wheel, not call the service.
    wp = {"expected_modes": ["corridor_nav", "rail_approach_pend"]}
    assert _dispatch_for(wp) == "nav2"


def test_dispatch_derived_rail_approach_active():
    wp = {"expected_modes": ["corridor_nav", "rail_approach_pend",
                              "rail_approach_active"]}
    assert _dispatch_for(wp) == "rail_approach"


def test_dispatch_empty_falls_back_to_nav2():
    assert _dispatch_for({}) == "nav2"
    assert _dispatch_for({"expected_modes": []}) == "nav2"


def test_dispatch_bad_value_falls_back_to_nav2():
    assert _dispatch_for({"dispatch": "moon_buggy"}) == "nav2"
    assert _dispatch_for({"dispatch": 42}) == "nav2"


def test_derive_tag_id_rear_aisle_center():
    wp = {"goal": {"x": 4.2, "y": 0.0}}
    assert _derive_tag_id(wp) == 35


def test_derive_tag_id_front_aisle_plus22():
    wp = {"goal": {"x": 7.0, "y": 2.2}}
    assert _derive_tag_id(wp) == 12


def test_derive_tag_id_rear_aisle_minus44():
    wp = {"goal": {"x": 4.0, "y": -4.4}}
    assert _derive_tag_id(wp) == 33


def test_derive_tag_id_explicit_wins():
    # Even though goal.y=0 would resolve to 35 at x=4, the explicit override
    # takes precedence.
    wp = {"tag_id": 99, "goal": {"x": 4.0, "y": 0.0}}
    assert _derive_tag_id(wp) == 99


def test_derive_tag_id_no_aisle_returns_none():
    # y=1.0 is between aisles (not within 0.35m of any center).
    wp = {"goal": {"x": 4.0, "y": 1.0}}
    assert _derive_tag_id(wp) is None


def test_derive_tag_id_no_approach_zone_returns_none():
    # x=10 is inside FRONT rail but not at an approach entry (x=7.0).
    wp = {"goal": {"x": 10.0, "y": 0.0}}
    assert _derive_tag_id(wp) is None
