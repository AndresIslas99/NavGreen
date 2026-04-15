#!/usr/bin/env python3
"""verify_state_machine — sanity checks on specs/state_machine.yaml structure.

This is a WARNING-severity check: it validates the SCHEMA of the state machine
spec (required fields present, invariants have a fix plan or an enforcer) but
does not attempt to verify the invariants against the running code — that
belongs in an online runtime check (agv_healthcheck.sh).
"""

from __future__ import annotations

import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:
    print("verify_state_machine: python3-yaml not installed", file=sys.stderr)
    sys.exit(2)

WS_ROOT = Path(__file__).resolve().parents[2]
SPEC = WS_ROOT / "specs/state_machine.yaml"


def main() -> int:
    if not SPEC.exists():
        print(f"FAIL: {SPEC} does not exist")
        return 1
    with SPEC.open() as f:
        data = yaml.safe_load(f) or {}

    warnings: list[str] = []

    # Required top-level keys.
    for key in ("layers", "valid_combinations", "invariants", "transitions"):
        if key not in data:
            warnings.append(f"FAIL: missing top-level key: {key}")

    # Each layer needs an authority and switchable_live flag.
    layers = data.get("layers", {}) or {}
    for layer_name, layer_def in layers.items():
        if not isinstance(layer_def, dict):
            continue
        for req in ("variable", "source", "authority", "switchable_live"):
            if req not in layer_def:
                warnings.append(f"WARN: layer {layer_name} missing {req}")

    # Each invariant needs either an enforced_by or a current_state marker.
    for inv in data.get("invariants", []) or []:
        if "id" not in inv:
            warnings.append(f"WARN: invariant without id: {inv}")
            continue
        if "enforced_by" not in inv and "current_state" not in inv and "fix_location" not in inv:
            warnings.append(f"WARN: invariant '{inv['id']}' has no enforced_by, current_state, or fix_location")

    if warnings:
        for w in warnings:
            print(w)
        print(f"verify_state_machine: {len(warnings)} warning(s)")
        return 0

    print("verify_state_machine: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
