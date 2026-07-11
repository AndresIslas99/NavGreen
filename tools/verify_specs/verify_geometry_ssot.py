#!/usr/bin/env python3
"""verify_geometry_ssot — Sprint A (2026-05-13 audit, CRITICAL-02-02).

Enforces that `src/agv_description/config/robot_geometry.yaml` is the
single source of truth for kinematic geometry. Two layers of checks:

BLOCKING (exit 1 on failure):
  * The SSOT file exists.
  * It declares the required keys (wheel_radius, track_width, gear_ratio).
  * `src/agv_odrive/config/odrive_params.yaml` does NOT contain any of
    those keys (geometry was moved out of odrive_params in C2).
  * `src/agv_description/launch/description.launch.py` and
    `src/agv_odrive/launch/odrive.launch.py` both reference the SSOT
    file by name (string grep — proves the launches are wired to load
    it).

WARN (exit 0 with `WARN:` lines — all.sh:69 flips RESULT to WARNING):
  * The SSOT wheel_radius matches the URDF `<xacro:arg>` default at
    `src/agv_description/urdf/agv_full.urdf.xacro` OR matches the
    geometric truth (0.0625 m). The current scaffold has SSOT=0.0781
    and URDF default=0.0625 — this WARN fires by design until
    CRITICAL-02-02 numerical fix lands.
  * The Nav2 footprint half-width (`nav2_params.yaml` footprint y-bound)
    matches `track_width / 2` within ±0.05 m. Currently it does NOT
    match (footprint sized for 0.735 m track, SSOT carries 0.960 m
    compensation). WARN fires until both are reconciled post-NVRAM-fix.

These WARN checks intentionally surface the geometry divergence so it
is visible on every commit but does not block the work that scaffolds
the SSOT. The numerical FAIL gate is enabled in a follow-up commit
after the NVRAM dump (see
`docs/calibration/odrive_nvram_dump_procedure.md`).
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:
    print("verify_geometry_ssot: python3-yaml not installed", file=sys.stderr)
    sys.exit(2)

WS_ROOT = Path(__file__).resolve().parents[2]

SSOT_FILE = WS_ROOT / "src/agv_description/config/robot_geometry.yaml"
ODRIVE_PARAMS = WS_ROOT / "src/agv_odrive/config/odrive_params.yaml"
DESC_LAUNCH = WS_ROOT / "src/agv_description/launch/description.launch.py"
ODRIVE_LAUNCH = WS_ROOT / "src/agv_odrive/launch/odrive.launch.py"
URDF_FILE = WS_ROOT / "src/agv_description/urdf/agv_full.urdf.xacro"
NAV2_PARAMS = WS_ROOT / "src/agv_navigation/config/nav2_params.yaml"

REQUIRED_KEYS = ("wheel_radius", "track_width", "gear_ratio")

# Geometric truth — caliper measurement 2026-05-13. The SSOT may
# legitimately differ from this (currently does, by the 1.25× NVRAM-bug
# compensation factor) but the URDF default must match this OR the
# SSOT, no third option.
GEOMETRIC_TRUTH_WHEEL_RADIUS = 0.0625


def _load_ssot() -> dict:
    with SSOT_FILE.open() as f:
        doc = yaml.safe_load(f) or {}
    # The flat ros2 shape: top-level `/**:` then `ros__parameters:`.
    ns = doc.get("/**", {})
    return ns.get("ros__parameters", {}) or {}


def _load_yaml(path: Path) -> dict:
    with path.open() as f:
        doc = yaml.safe_load(f) or {}
    ns = doc.get("/**", {})
    return ns.get("ros__parameters", {}) or {}


def _file_contains(path: Path, needle: str) -> bool:
    if not path.exists():
        return False
    return needle in path.read_text(encoding="utf-8", errors="ignore")


def _urdf_arg_default(path: Path, arg_name: str) -> str | None:
    """Extract the `default=` of a `<xacro:arg name=arg_name default=X/>` tag.

    Returns the raw string default (no type conversion) or None if the
    arg is not present or has no default.
    """
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8", errors="ignore")
    # Match either `<xacro:arg name="X" default="Y"/>` or with single quotes.
    pattern = (
        r"""<\s*xacro:arg\s+name\s*=\s*['"]"""
        + re.escape(arg_name)
        + r"""['"]\s+default\s*=\s*['"]([^'"]+)['"]"""
    )
    m = re.search(pattern, text)
    return m.group(1) if m else None


def _nav2_footprint_half_width(path: Path) -> float | None:
    """Parse the Nav2 footprint and return half the y-width.

    nav2_params.yaml stores the footprint as a YAML string like
        `footprint: "[[0.50, 0.37], [0.50, -0.37], [-0.30, -0.37], [-0.30, 0.37]]"`
    We extract |max y - min y| / 2.
    """
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8", errors="ignore")
    m = re.search(r"footprint:\s*\"(\[\[.+?\]\])\"", text)
    if not m:
        return None
    try:
        pts = yaml.safe_load(m.group(1))
        ys = [pt[1] for pt in pts]
        return (max(ys) - min(ys)) / 2.0
    except Exception:
        return None


def main() -> int:
    blocking: list[str] = []
    warnings: list[str] = []

    # ── BLOCKING 1: SSOT file exists ──────────────────────────────
    if not SSOT_FILE.exists():
        blocking.append(f"FAIL: SSOT file missing: {SSOT_FILE.relative_to(WS_ROOT)}")
        # Cannot continue checks that depend on the SSOT
        for msg in blocking:
            print(msg)
        return 1

    ssot = _load_ssot()

    # ── BLOCKING 2: required keys present in SSOT ─────────────────
    missing = [k for k in REQUIRED_KEYS if k not in ssot]
    if missing:
        blocking.append(
            f"FAIL: SSOT missing required keys {missing} in "
            f"{SSOT_FILE.relative_to(WS_ROOT)} (expected under /**: ros__parameters:)"
        )

    # ── BLOCKING 3: odrive_params.yaml does NOT re-declare geometry keys ──
    if ODRIVE_PARAMS.exists():
        odrive_params = _load_yaml(ODRIVE_PARAMS)
        leaked = [k for k in REQUIRED_KEYS if k in odrive_params]
        if leaked:
            blocking.append(
                f"FAIL: {ODRIVE_PARAMS.relative_to(WS_ROOT)} re-declares geometry keys "
                f"{leaked} — they belong in the SSOT only. Re-introduction would "
                f"override the SSOT silently."
            )
    else:
        blocking.append(f"FAIL: {ODRIVE_PARAMS.relative_to(WS_ROOT)} missing")

    # ── BLOCKING 4 & 5: launches reference the SSOT file by name ──
    ssot_name = SSOT_FILE.name
    if not _file_contains(DESC_LAUNCH, ssot_name):
        blocking.append(
            f"FAIL: {DESC_LAUNCH.relative_to(WS_ROOT)} does not reference "
            f"'{ssot_name}'. Description launch must load the SSOT and pass "
            f"its values to xacro as args."
        )
    if not _file_contains(ODRIVE_LAUNCH, ssot_name):
        blocking.append(
            f"FAIL: {ODRIVE_LAUNCH.relative_to(WS_ROOT)} does not reference "
            f"'{ssot_name}'. ODrive launch must load the SSOT before odrive_params.yaml."
        )

    if blocking:
        for msg in blocking:
            print(msg)
        print(f"verify_geometry_ssot: {len(blocking)} blocking failure(s)")
        return 1

    # ── WARN 1: URDF <xacro:arg> default ↔ SSOT consistency ───────
    urdf_default = _urdf_arg_default(URDF_FILE, "wheel_radius")
    ssot_radius = ssot["wheel_radius"]
    if urdf_default is None:
        warnings.append(
            f"WARN: {URDF_FILE.relative_to(WS_ROOT)} has no <xacro:arg name=\"wheel_radius\" default=\"...\"/>. "
            f"Standalone xacro testing falls back to internal defaults."
        )
    else:
        try:
            urdf_radius = float(urdf_default)
        except ValueError:
            warnings.append(
                f"WARN: {URDF_FILE.relative_to(WS_ROOT)} xacro:arg wheel_radius default "
                f"'{urdf_default}' is not numeric."
            )
        else:
            if abs(urdf_radius - ssot_radius) > 1e-4 and abs(urdf_radius - GEOMETRIC_TRUTH_WHEEL_RADIUS) > 1e-4:
                warnings.append(
                    f"WARN: URDF xacro:arg wheel_radius default {urdf_radius} matches neither "
                    f"the SSOT ({ssot_radius}) nor the geometric truth ({GEOMETRIC_TRUTH_WHEEL_RADIUS}). "
                    f"Expected one of those two values."
                )
            elif abs(urdf_radius - ssot_radius) > 1e-4:
                warnings.append(
                    f"WARN: URDF xacro:arg wheel_radius default {urdf_radius} (geometric truth) "
                    f"diverges from SSOT runtime value {ssot_radius}. Expected until CRITICAL-02-02 "
                    f"step 5 restores the SSOT to the geometric truth. See "
                    f"docs/calibration/odrive_nvram_dump_procedure.md."
                )

    # ── WARN 2: Nav2 footprint half-width ↔ SSOT track_width / 2 ──
    fp_half = _nav2_footprint_half_width(NAV2_PARAMS)
    if fp_half is None:
        warnings.append(
            f"WARN: could not parse footprint from {NAV2_PARAMS.relative_to(WS_ROOT)} — "
            f"skipping numerical check."
        )
    else:
        expected_half = ssot["track_width"] / 2.0
        if abs(fp_half - expected_half) > 0.05:
            warnings.append(
                f"WARN: Nav2 footprint half-width {fp_half:.3f} m diverges from "
                f"SSOT track_width/2 = {expected_half:.3f} m by more than 0.05 m. "
                f"With current SSOT (0.960 m track) wheels may extend beyond the "
                f"costmap footprint by {expected_half - fp_half:.3f} m. See CRITICAL-02-02 "
                f"and audit Phase 2 finding CRITICAL-02-02."
            )

    if warnings:
        for w in warnings:
            print(w)
        print(f"verify_geometry_ssot: {len(warnings)} warning(s) (structural checks passed)")
        return 0

    print("verify_geometry_ssot: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
