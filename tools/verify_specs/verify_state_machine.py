#!/usr/bin/env python3
"""verify_state_machine — checks on specs/state_machine.yaml.

Two classes of checks:

1. Structural (FAIL → exit 1): required top-level keys must exist, and the
   runtime mode-arbiter layer declared in the spec must match the actual
   `enum class Mode` in src/agv_mode_arbiter (Phase 2 FSM). A gutted or
   drifted spec must never pass green — all.sh treats FAIL lines and a
   non-zero exit as blocking.

2. Soft schema warnings (WARN → exit 0): missing optional per-layer or
   per-invariant fields. These do not block.

Behavioral verification of the invariants against a running system belongs
in an online runtime check (agv_healthcheck.sh), not here.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:
    print("verify_state_machine: python3-yaml not installed", file=sys.stderr)
    sys.exit(2)

WS_ROOT = Path(__file__).resolve().parents[2]
SPEC = WS_ROOT / "specs/state_machine.yaml"
MODE_FSM_HPP = WS_ROOT / "src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp"

# Matches the body of `enum class Mode { ... };` — comments allowed inside.
ENUM_RE = re.compile(r"enum\s+class\s+Mode\s*(?::\s*\w+\s*)?\{(?P<body>.*?)\};", re.DOTALL)
IDENT_RE = re.compile(r"^\s*([A-Z][A-Z0-9_]*)\s*(?:=\s*[^,]+)?,?\s*(?://.*)?$")


def parse_mode_enum(path: Path) -> list[str] | None:
    """Return the enumerator names of `enum class Mode` in mode_fsm.hpp."""
    if not path.exists():
        return None
    m = ENUM_RE.search(path.read_text(errors="ignore"))
    if not m:
        return None
    states: list[str] = []
    for line in m.group("body").splitlines():
        line = line.strip()
        if not line or line.startswith("//"):
            continue
        im = IDENT_RE.match(line)
        if im:
            states.append(im.group(1))
    return states


def main() -> int:
    if not SPEC.exists():
        print(f"FAIL: {SPEC} does not exist")
        return 1
    with SPEC.open() as f:
        data = yaml.safe_load(f) or {}

    failures: list[str] = []
    warnings: list[str] = []

    # ── Structural: required top-level keys ──────────────────────────────
    for key in ("layers", "valid_combinations", "invariants", "transitions"):
        if key not in data:
            failures.append(f"FAIL: missing top-level key: {key}")

    layers = data.get("layers", {}) or {}

    # ── Structural: runtime arbiter layer must mirror mode_fsm.hpp ───────
    # The Phase-2 mode arbiter owns /agv/cmd_vel in production; its FSM
    # states are part of the answer to "what is the current mode?" and the
    # spec must track the C++ enum exactly.
    code_states = parse_mode_enum(MODE_FSM_HPP)
    arbiter_layer = layers.get("layer_5_runtime_arbiter")
    if code_states is None:
        failures.append(
            f"FAIL: cannot parse 'enum class Mode' from {MODE_FSM_HPP.relative_to(WS_ROOT)} "
            "(file missing or enum renamed) — update this verifier and the spec together"
        )
    elif not isinstance(arbiter_layer, dict):
        failures.append(
            "FAIL: layers.layer_5_runtime_arbiter missing from state_machine.yaml but "
            f"src/agv_mode_arbiter/mode_fsm.hpp defines FSM states: {', '.join(code_states)}"
        )
    else:
        spec_states = list((arbiter_layer.get("values") or {}).keys())
        if [s.upper() for s in spec_states] != code_states:
            failures.append(
                "FAIL: layer_5_runtime_arbiter.values does not match enum class Mode in "
                f"{MODE_FSM_HPP.relative_to(WS_ROOT)}: spec={spec_states} code={code_states}"
            )

    # ── Soft schema warnings ──────────────────────────────────────────────
    for layer_name, layer_def in layers.items():
        if not isinstance(layer_def, dict):
            continue
        for req in ("variable", "source", "authority", "switchable_live"):
            if req not in layer_def:
                warnings.append(f"WARN: layer {layer_name} missing {req}")

    for inv in data.get("invariants", []) or []:
        if "id" not in inv:
            warnings.append(f"WARN: invariant without id: {inv}")
            continue
        if "enforced_by" not in inv and "current_state" not in inv and "fix_location" not in inv:
            warnings.append(f"WARN: invariant '{inv['id']}' has no enforced_by, current_state, or fix_location")

    for line in failures + warnings:
        print(line)

    if failures:
        print(f"verify_state_machine: {len(failures)} failure(s), {len(warnings)} warning(s)")
        return 1
    if warnings:
        print(f"verify_state_machine: {len(warnings)} warning(s)")
        return 0

    print("verify_state_machine: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
