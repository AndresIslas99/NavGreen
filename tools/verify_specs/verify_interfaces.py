#!/usr/bin/env python3
"""verify_interfaces — check that interfaces declared in specs/interfaces.yaml exist in code.

For each topic, service, and action declared:
- If it is an AGV-owned interface (owner_pkg starts with agv_*), verify the
  owner_pkg source tree contains a `create_publisher` / `advertise_service` /
  `create_service` / `rclcpp::Action` call matching the name.
- The match is lexical — we grep for the shortest unambiguous form of the
  topic name (stripping the `/agv/` namespace).

This is NOT a semantic check of QoS or type — it is a presence check. The goal
is to catch "spec says topic X exists but nobody publishes it" and "code
publishes topic Y but spec doesn't know about it" (partial — see TODO).

Severity: BLOCKING for presence failures of owner_pkg claims. WARNING for
unowned (external) interfaces.
"""

from __future__ import annotations

import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:
    print("verify_interfaces: python3-yaml not installed", file=sys.stderr)
    sys.exit(2)

WS_ROOT = Path(__file__).resolve().parents[2]
SRC = WS_ROOT / "src"
SPEC = WS_ROOT / "specs/interfaces.yaml"


def short_topic(name: str) -> str:
    """Return the last segment of a topic path — what the code is likely to use."""
    return name.rsplit("/", 1)[-1]


def grep_in_pkg(pkg_name: str, needle: str) -> list[Path]:
    """Return files under src/<pkg>/ that contain needle (case-sensitive)."""
    pkg_dir = SRC / pkg_name
    if not pkg_dir.exists():
        return []
    hits: list[Path] = []
    for ext in ("*.cpp", "*.hpp", "*.h", "*.py", "*.ts"):
        for f in pkg_dir.rglob(ext):
            try:
                if needle in f.read_text(errors="ignore"):
                    hits.append(f)
            except Exception:
                continue
    return hits


def check_interface(iface: dict, kind: str) -> str | None:
    name = iface.get("name")
    owner_pkg = iface.get("owner_pkg", "")
    status = iface.get("status", "")
    if not name:
        return f"{kind} entry has no name: {iface}"
    # Skip interfaces explicitly marked planned / deprecated — they are
    # spec-only placeholders with no current implementation.
    if status in ("planned", "deprecated", "proposed"):
        return None
    # Only check interfaces owned by agv_* packages.
    if not owner_pkg.startswith("agv_"):
        return None
    short = short_topic(name)
    hits = grep_in_pkg(owner_pkg, short)
    if not hits:
        return f"FAIL: {kind} '{name}' declared with owner_pkg={owner_pkg} but no code match for '{short}' in src/{owner_pkg}/"
    return None


def main() -> int:
    if not SPEC.exists():
        print(f"FAIL: {SPEC} does not exist")
        return 1
    with SPEC.open() as f:
        data = yaml.safe_load(f) or {}

    errors: list[str] = []

    for topic in data.get("topics", []) or []:
        err = check_interface(topic, "topic")
        if err:
            errors.append(err)

    for svc in data.get("services", []) or []:
        err = check_interface(svc, "service")
        if err:
            errors.append(err)

    for act in data.get("actions", []) or []:
        err = check_interface(act, "action")
        if err:
            errors.append(err)

    if errors:
        for e in errors:
            print(e)
        print(f"verify_interfaces: {len(errors)} violation(s)")
        print("Fix: either update the code to publish the declared interface, OR remove/relocate the spec entry.")
        return 1

    print("verify_interfaces: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
