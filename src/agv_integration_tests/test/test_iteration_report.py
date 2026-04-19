"""ROS-free tests for iteration_report.render_markdown.

Covers the four report shapes the Round-44 loop cares about:

  1. clean pass — every waypoint SUCCEEDED + every gate met.
  2. single regression — one rail_approach miss vs previous iteration.
  3. localization regression — rail_drive peak > 0.05 m triggers rule.
  4. full failure — COLLISION during rail_exit triggers collision rule.

The generator is pure Python; we feed synthetic dicts shaped like the
Round-44 report.json v2 and scan the markdown for the expected tokens.
"""
from __future__ import annotations

import sys
from pathlib import Path

# Make iteration_report importable without installing the package.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import iteration_report as ir  # type: ignore


def _rules():
    rules_path = Path(__file__).resolve().parent.parent / "scripts" / "iteration_analysis_rules.yaml"
    return ir._load_rules(rules_path)


def _empty_loc():
    return {
        "peak_pos_err_m": 0.0,
        "peak_yaw_err_rad": 0.0,
        "rmse_pos_m": 0.0,
        "rmse_yaw_rad": 0.0,
        "sample_count": 10,
    }


def _clean_report():
    return {
        "run_id": "precision_run_clean",
        "report_version": 2,
        "summary": {
            "sample_size": 2,
            "success_rate": 1.0,
            "p95_err_xy_m": 0.05,
            "max_err_xy_m": 0.06,
            "mean_err_xy_m": 0.04,
            "p95_err_yaw_rad": 0.01,
            "collision_count": 0,
            "status_histogram": {"SUCCEEDED": 2},
        },
        "waypoints": [
            {
                "wp_id": "wp01", "status": "SUCCEEDED", "err_xy": 0.05,
                "dispatch_used": "nav2", "localization": _empty_loc(),
                "modes_observed": ["corridor_nav"],
                "event_histogram": {},
            },
            {
                "wp_id": "wp04", "status": "SUCCEEDED", "err_xy": 0.01,
                "dispatch_used": "rail_approach", "localization": _empty_loc(),
                "modes_observed": ["corridor_nav", "rail_approach_active"],
                "event_histogram": {},
                "visible_markers_at_end": [{"id": 35, "distance_m": 0.3}],
                "tag_id": 35,
            },
        ],
    }


# ── case 1: clean pass ──────────────────────────────────────────────────

def test_clean_pass_prints_all_green():
    md = ir.render_markdown(_clean_report(), None, _rules(), iteration_n=1)
    assert "# Iteration 1 analysis" in md
    assert "| nav2 | 1 | 1 | →" in md
    assert "| rail_approach | 1 | 1 | →" in md
    # Convergence block marks all gates passing.
    assert "| collision_count | 0 | 0 | ✓ |" in md
    assert "All gates passed" in md
    assert "_All waypoints passed their bucket gates._" in md


# ── case 2: single rail_approach regression ────────────────────────────

def test_rail_approach_regression_flagged():
    prev = _clean_report()
    curr = _clean_report()
    curr["waypoints"][1]["status"] = "ABORTED"
    curr["waypoints"][1]["err_xy"] = 0.04  # > 0.02 m gate
    curr["summary"]["success_rate"] = 0.5
    md = ir.render_markdown(curr, prev, _rules(), iteration_n=2)
    # Delta indicator must show rail_approach regressed.
    assert "| rail_approach | 0 | 1 | ↓ |" in md
    # Gate failure reason surfaces.
    assert "err_xy 0.040 m > 0.02 m gate" in md
    # Convergence flags success_rate below threshold.
    assert "| success_rate | 0.500 | 0.95 | ✗ |" in md


# ── case 3: rail_drive localization regression ─────────────────────────

def test_rail_drive_loc_err_triggers_rule():
    curr = _clean_report()
    curr["waypoints"].append({
        "wp_id": "wp05", "status": "SUCCEEDED", "err_xy": 0.03,
        "dispatch_used": "rail_drive",
        "localization": {**_empty_loc(), "peak_pos_err_m": 0.12},
        "modes_observed": ["rail_drive"],
        "event_histogram": {},
    })
    md = ir.render_markdown(curr, None, _rules(), iteration_n=3)
    assert "### wp05" in md
    assert "rail_drive_lat_drift" in md
    assert "0.120 m" in md


# ── case 4: collision ──────────────────────────────────────────────────

def test_collision_event_triggers_root_cause():
    curr = _clean_report()
    curr["waypoints"].append({
        "wp_id": "wp14", "status": "COLLISION", "err_xy": None,
        "dispatch_used": "rail_exit",
        "localization": _empty_loc(),
        "modes_observed": ["rail_drive", "rail_exit", "corridor_nav"],
        "event_histogram": {"collision": 1},
        "nearest_obstacle_at_end": {
            "name": "crop_row_n", "kind": "bush",
            "distance_m": 0.12, "obstacle_x": 6.0, "obstacle_y": 2.2,
        },
        "snapshot_paths": {"fail_jpg": "/tmp/wp14_fail.jpg", "events_json": None},
    })
    curr["summary"]["collision_count"] = 1
    md = ir.render_markdown(curr, None, _rules(), iteration_n=4)
    assert "### wp14" in md
    assert "collision_any_bucket" in md
    assert "crop_row_n" in md
    # Convergence picks up the collision violation.
    assert "| collision_count | 1 | 0 | ✗ |" in md
