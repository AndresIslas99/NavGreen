#!/usr/bin/env python3
"""iteration_report — turn a precision run's report.json into a markdown.

Usage:
    python3 iteration_report.py <report.json> [prev_report.json] \
        [--rules rules.yaml] [--out OUT.md]

Reads the Round-44 v2 report(s) and emits
`iteration_N_analysis.md` alongside the current report (or `--out`).
Contents:

  1. Per-bucket verdict table (with delta vs prev).
  2. Per-waypoint diagnosis for every failure OR regression. Each diagnosis
     answers three questions by replaying rules from
     iteration_analysis_rules.yaml:
       - What went wrong?  (root-cause category)
       - What to reconsider? (approach-level recommendation)
       - Code/param to touch? (config path)
  3. Acceptance convergence block (gates with current values + pass/fail).
  4. Next-iteration recommendation one-liner.

Pure Python; no ROS dependency. Data-driven via the YAML sidecar.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Optional

try:
    import yaml
except ImportError:  # pragma: no cover
    print("iteration_report.py requires PyYAML", file=sys.stderr)
    sys.exit(2)


DEFAULT_RULES = Path(__file__).parent / "iteration_analysis_rules.yaml"

# ── report parsing helpers ─────────────────────────────────────────────

def _get_nested(d: Any, path: str) -> Any:
    """Look up 'a.b.c' in nested dicts, returning None on any miss."""
    cur = d
    for part in path.split("."):
        if isinstance(cur, dict):
            cur = cur.get(part)
        elif isinstance(cur, list):
            try:
                cur = cur[int(part)]
            except (ValueError, IndexError):
                return None
        else:
            return None
        if cur is None:
            return None
    return cur


def _cmp(value: Any, op: str, target: Any) -> bool:
    if value is None:
        return False
    try:
        if op == "gt":  return float(value) > float(target)
        if op == "gte": return float(value) >= float(target)
        if op == "lt":  return float(value) < float(target)
        if op == "lte": return float(value) <= float(target)
        if op == "eq":  return value == target
        if op == "in":  return value in target
    except (TypeError, ValueError):
        return False
    if op == "contains":
        return isinstance(value, (list, str)) and target in value
    if op == "missing":
        return isinstance(value, list) and target not in value
    return False


def _format(template: str, wp: dict) -> str:
    """Minimal {a.b.c} and {a.b:fmt} substitution for rule messages."""
    out = template
    while "{" in out and "}" in out:
        start = out.index("{")
        end = out.index("}", start)
        token = out[start + 1:end]
        if ":" in token:
            path, fmt = token.split(":", 1)
            val = _get_nested(wp, path)
            try:
                repl = format(val, fmt) if val is not None else "?"
            except (TypeError, ValueError):
                repl = str(val) if val is not None else "?"
        else:
            val = _get_nested(wp, token)
            repl = str(val) if val is not None else "?"
        out = out[:start] + repl + out[end + 1:]
    return out


# ── rule evaluation ────────────────────────────────────────────────────

def _rule_applies(rule: dict, wp: dict) -> bool:
    bucket = rule.get("bucket")
    if bucket is not None and wp.get("dispatch_used") != bucket:
        return False
    when = rule.get("when", {})
    if not isinstance(when, dict):
        return False
    for field, cond in when.items():
        if not isinstance(cond, dict):
            return False
        # Special handling for a couple of semantic checks the rules file uses.
        if field == "modes_observed.contains":
            for op, target in cond.items():
                observed = wp.get("modes_observed") or []
                if op == "contains" and target in observed:
                    continue
                if op == "missing" and target not in observed:
                    continue
                return False
            continue
        if field == "visible_markers_at_end.contains_id":
            # {"not_in_expected_tag": true} → the modes_expected's approach tag
            # MUST be visible; if not present, rule fires.
            markers = wp.get("visible_markers_at_end") or []
            observed_ids = {m.get("id") for m in markers if isinstance(m, dict)}
            # Tag lookup uses the explicit tag_id if the report carries it;
            # else we cannot evaluate and the rule is inconclusive.
            expected = wp.get("tag_id") or wp.get("expected_tag_id")
            if not cond.get("not_in_expected_tag"):
                return False
            if expected is None:
                return False
            return expected not in observed_ids
        # Generic operator matching.
        value = _get_nested(wp, field)
        ok = False
        for op, target in cond.items():
            if _cmp(value, op, target):
                ok = True
                break
        if not ok:
            return False
    return True


def _diagnose_waypoint(wp: dict, rules: list) -> list[dict]:
    """Return list of firing rules (root_cause + recommendation)."""
    hits = []
    for rule in rules:
        try:
            if _rule_applies(rule, wp):
                hits.append({
                    "id": rule["id"],
                    "category": rule.get("category", "recommendation"),
                    "message": _format(rule.get("message", ""), wp),
                })
        except Exception:
            # Rules are data-driven; a bad rule must not crash the report.
            continue
    return hits


# ── bucket summarisation ───────────────────────────────────────────────

BUCKETS = ("nav2", "rail_approach", "rail_drive", "rail_exit")


def _bucket_of(wp: dict) -> str:
    return wp.get("dispatch_used") or "nav2"


def _bucket_verdicts(report: dict, thresholds: dict) -> dict:
    out: dict = {b: {"n": 0, "passed": 0, "fails": [], "regressions": []} for b in BUCKETS}
    per_bucket = (thresholds or {}).get("per_bucket", {})
    for wp in report.get("waypoints", []):
        b = _bucket_of(wp)
        if b not in out:
            out[b] = {"n": 0, "passed": 0, "fails": [], "regressions": []}
        out[b]["n"] += 1
        status = wp.get("status")
        err_xy = wp.get("err_xy")
        loc = wp.get("localization") or {}
        bucket_rules = per_bucket.get(b, {})
        failed = False
        reasons: list[str] = []
        if status != "SUCCEEDED":
            failed = True
            reasons.append(f"status={status}")
        if b == "nav2" and err_xy is not None and err_xy > bucket_rules.get("mean_err_xy_m", 0.15):
            # Tolerant per-waypoint check; summary gates handle the aggregate.
            pass
        if b == "rail_approach" and err_xy is not None and err_xy > bucket_rules.get("lat_err_m", 0.02):
            failed = True
            reasons.append(f"err_xy {err_xy:.3f} m > {bucket_rules.get('lat_err_m', 0.02)} m gate")
        if b == "rail_drive":
            peak = loc.get("peak_pos_err_m", 0.0) or 0.0
            if peak > bucket_rules.get("peak_lat_m", 0.05):
                failed = True
                reasons.append(f"peak_pos_err {peak:.3f} m > {bucket_rules.get('peak_lat_m', 0.05)} m")
        if b == "rail_exit":
            modes = wp.get("modes_observed") or []
            if "rail_exit" not in modes:
                failed = True
                reasons.append("modes missing rail_exit")
        if failed:
            out[b]["fails"].append({"wp_id": wp.get("wp_id"), "reasons": reasons})
        else:
            out[b]["passed"] += 1
    return out


def _diff_buckets(curr: dict, prev: Optional[dict]) -> dict:
    if prev is None:
        return {b: "→" for b in BUCKETS}
    diff = {}
    prev_verdicts = _bucket_verdicts(prev, {})
    for b in BUCKETS:
        cn = curr[b]["passed"] / curr[b]["n"] if curr[b]["n"] else 0.0
        pn = prev_verdicts[b]["passed"] / prev_verdicts[b]["n"] if prev_verdicts[b]["n"] else 0.0
        if cn > pn + 1e-6:
            diff[b] = "↑"
        elif cn + 1e-6 < pn:
            diff[b] = "↓"
        else:
            diff[b] = "→"
    return diff


# ── markdown rendering ─────────────────────────────────────────────────

def render_markdown(curr: dict, prev: Optional[dict], rules_cfg: dict,
                    iteration_n: int = 1) -> str:
    thresholds = rules_cfg.get("summary_thresholds", {})
    rules = rules_cfg.get("rules", [])
    buckets = _bucket_verdicts(curr, thresholds)
    deltas = _diff_buckets(buckets, prev)

    lines: list[str] = []
    lines.append(f"# Iteration {iteration_n} analysis — {curr.get('run_id', '?')}")
    lines.append("")
    summary = curr.get("summary", {})

    # 1. Per-bucket verdict table
    lines.append("## Per-bucket verdicts")
    lines.append("")
    lines.append("| Bucket | Passed | Total | Δ vs prev |")
    lines.append("|---|---|---|---|")
    for b in BUCKETS:
        lines.append(f"| {b} | {buckets[b]['passed']} | {buckets[b]['n']} | {deltas[b]} |")
    lines.append("")

    # 2. Per-waypoint diagnosis
    any_diag = False
    diag_lines: list[str] = []
    for wp in curr.get("waypoints", []):
        b = _bucket_of(wp)
        wpid = wp.get("wp_id")
        status = wp.get("status")
        # Only diagnose failures + non-succeeded statuses.
        bucket_fails = [f for f in buckets[b]["fails"] if f["wp_id"] == wpid]
        if status == "SUCCEEDED" and not bucket_fails:
            continue
        any_diag = True
        diag_lines.append(f"### {wpid} ({b}, status={status})")
        if bucket_fails and bucket_fails[0]["reasons"]:
            diag_lines.append("- **Gate failure(s):** " + "; ".join(bucket_fails[0]["reasons"]))
        hits = _diagnose_waypoint(wp, rules)
        root = [h for h in hits if h["category"] == "root_cause"]
        reco = [h for h in hits if h["category"] == "recommendation"]
        if root:
            diag_lines.append("- **What went wrong:**")
            for h in root:
                diag_lines.append(f"  - ({h['id']}) {h['message']}")
        if reco:
            diag_lines.append("- **Recommendation:**")
            for h in reco:
                diag_lines.append(f"  - ({h['id']}) {h['message']}")
        snap = wp.get("snapshot_paths") or {}
        if snap.get("fail_jpg"):
            diag_lines.append(f"- Snapshot: `{snap['fail_jpg']}`")
        if snap.get("events_json"):
            diag_lines.append(f"- Events: `{snap['events_json']}`")
        diag_lines.append("")
    lines.append("## Per-waypoint diagnosis")
    lines.append("")
    if any_diag:
        lines.extend(diag_lines)
    else:
        lines.append("_All waypoints passed their bucket gates._")
        lines.append("")

    # 3. Acceptance convergence block
    lines.append("## Acceptance convergence")
    lines.append("")
    checks = [
        ("p95_err_xy_m",     summary.get("p95_err_xy_m"),
            thresholds.get("p95_err_xy_m", 0.10), "≤"),
        ("max_err_xy_m",     summary.get("max_err_xy_m"),
            thresholds.get("max_err_xy_m", 0.15), "≤"),
        ("p95_err_yaw_rad",  summary.get("p95_err_yaw_rad"),
            thresholds.get("p95_err_yaw_rad", 0.25), "≤"),
        ("success_rate",     summary.get("success_rate"),
            thresholds.get("min_success_rate", 0.95), "≥"),
        ("collision_count",  summary.get("collision_count"), 0, "=="),
    ]
    lines.append("| Gate | Current | Target | Pass |")
    lines.append("|---|---|---|---|")
    all_pass = True
    for name, val, target, op in checks:
        if val is None or (isinstance(val, float) and math.isnan(val)):
            display = "n/a"
            passed = False
        elif op == "≤":
            passed = float(val) <= float(target)
            display = f"{float(val):.3f}"
        elif op == "≥":
            passed = float(val) >= float(target)
            display = f"{float(val):.3f}"
        else:  # ==
            passed = val == target
            display = str(val)
        all_pass = all_pass and passed
        lines.append(f"| {name} | {display} | {target} | {'✓' if passed else '✗'} |")
    lines.append("")

    # 4. Next-iteration recommendation
    lines.append("## Next iteration")
    lines.append("")
    if all_pass and not any_diag:
        lines.append("All gates passed. Re-run once more; two clean runs → acceptance.")
    else:
        # Pick the first firing rule from any diagnosis.
        pick = None
        for wp in curr.get("waypoints", []):
            for h in _diagnose_waypoint(wp, rules):
                if h["category"] == "root_cause":
                    pick = (wp.get("wp_id"), h)
                    break
            if pick:
                break
        if pick:
            lines.append(f"Top recommendation — address **{pick[1]['id']}** "
                         f"observed on {pick[0]}: {pick[1]['message']}")
        else:
            lines.append("Review bucket failures above; apply the smallest diff "
                         "that addresses the most common gate.")
    lines.append("")
    return "\n".join(lines)


# ── CLI entry ──────────────────────────────────────────────────────────

def _load_json(path: Path) -> dict:
    with path.open() as f:
        return json.load(f)


def _load_rules(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f) or {}


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("report", type=Path, help="report.json from this iteration")
    ap.add_argument("prev", type=Path, nargs="?", default=None,
                    help="report.json from previous iteration (optional)")
    ap.add_argument("--rules", type=Path, default=DEFAULT_RULES)
    ap.add_argument("--out", type=Path, default=None,
                    help="markdown output path (default: alongside report.json)")
    ap.add_argument("--iteration", type=int, default=1)
    args = ap.parse_args(argv)

    curr = _load_json(args.report)
    prev = _load_json(args.prev) if args.prev else None
    rules_cfg = _load_rules(args.rules)
    md = render_markdown(curr, prev, rules_cfg, iteration_n=args.iteration)
    out = args.out or (args.report.parent / f"iteration_{args.iteration}_analysis.md")
    out.write_text(md)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
