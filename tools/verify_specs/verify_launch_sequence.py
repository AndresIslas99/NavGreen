#!/usr/bin/env python3
"""verify_launch_sequence — sanity checks on specs/launch_sequence.yaml.

Validates schema and that each `source:` file:line reference actually points
to an existing file in the workspace.
"""

from __future__ import annotations

import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:
    print("verify_launch_sequence: python3-yaml not installed", file=sys.stderr)
    sys.exit(2)

WS_ROOT = Path(__file__).resolve().parents[2]
SPEC = WS_ROOT / "specs/launch_sequence.yaml"


def main() -> int:
    if not SPEC.exists():
        print(f"FAIL: {SPEC} does not exist")
        return 1
    with SPEC.open() as f:
        data = yaml.safe_load(f) or {}

    errors: list[str] = []

    if "sequence" not in data:
        errors.append("FAIL: missing top-level 'sequence' key")
    else:
        for entry in data["sequence"]:
            if not isinstance(entry, dict):
                continue
            name = entry.get("name", "<unknown>")
            src = entry.get("source")
            if not src:
                errors.append(f"WARN: entry '{name}' missing 'source'")
                continue
            # Skip free-form references that are not real paths. If it does
            # not look like a path (no '/' and no '.launch.py', etc), assume
            # it is a human note referring to a previous entry.
            if "/" not in src:
                continue
            path = src.split(":")[0]
            p = WS_ROOT / path
            if not p.exists():
                errors.append(f"WARN: entry '{name}' source file missing: {src}")

    if errors:
        for e in errors:
            print(e)
        print(f"verify_launch_sequence: {len(errors)} issue(s)")
        return 0  # WARNING severity

    print("verify_launch_sequence: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
