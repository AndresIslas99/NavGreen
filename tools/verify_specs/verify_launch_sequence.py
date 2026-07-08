#!/usr/bin/env python3
"""verify_launch_sequence — checks on specs/launch_sequence.yaml.

`source:` convention: "<path>" or "<path> (<anchor>)" or "<path>:<line>".
- The path must exist in the workspace (FAIL — a spec pointing at a deleted
  launch file is structural drift and blocks the suite via all.sh).
- If an anchor is given, it must appear as a literal substring of the file
  (WARN — anchors replace brittle line numbers; a vanished anchor means the
  entry needs a refresh but the file still exists).
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


def parse_source(src: str) -> tuple[str, str | None]:
    """Split "<path> (<anchor>)" / "<path>:<line>" into (path, anchor)."""
    anchor = None
    if " (" in src and src.rstrip().endswith(")"):
        path_part, anchor_part = src.split(" (", 1)
        anchor = anchor_part.rstrip()[:-1].strip() or None
        src = path_part
    return src.split(":")[0].strip(), anchor


def main() -> int:
    if not SPEC.exists():
        print(f"FAIL: {SPEC} does not exist")
        return 1
    with SPEC.open() as f:
        data = yaml.safe_load(f) or {}

    failures: list[str] = []
    warnings: list[str] = []

    if "sequence" not in data:
        failures.append("FAIL: missing top-level 'sequence' key")
    else:
        for entry in data["sequence"]:
            if not isinstance(entry, dict):
                continue
            name = entry.get("name", "<unknown>")
            src = entry.get("source")
            if not src:
                warnings.append(f"WARN: entry '{name}' missing 'source'")
                continue
            # Free-form references without a path are human notes.
            if "/" not in src:
                continue
            path, anchor = parse_source(src)
            p = WS_ROOT / path
            if not p.exists():
                failures.append(f"FAIL: entry '{name}' source file missing: {src}")
                continue
            if anchor:
                try:
                    text = p.read_text(errors="ignore")
                except OSError:
                    text = ""
                if anchor not in text:
                    warnings.append(f"WARN: entry '{name}' anchor '{anchor}' not found in {path}")

    for line in failures + warnings:
        print(line)

    if failures:
        print(f"verify_launch_sequence: {len(failures)} failure(s), {len(warnings)} warning(s)")
        return 1
    if warnings:
        print(f"verify_launch_sequence: {len(warnings)} warning(s)")
        return 0

    print("verify_launch_sequence: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
