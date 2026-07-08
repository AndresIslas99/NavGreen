#!/usr/bin/env python3
"""verify_interfaces — check specs/interfaces.yaml against the code.

Checks (all BLOCKING — a FAIL line fails the suite):

1. Owner presence: every topic/service/action owned by a workspace package
   (src/<pkg> or fleet/<pkg>) must reference the interface name as a QUOTED
   STRING LITERAL in that package (or, for relative names remapped at
   launch, in a launch file). A mention in a comment or a substring inside
   an unrelated identifier does NOT count — the old raw-substring grep let
   both pass.

2. Declared-consumer presence: every subscriber/caller/publisher entry that
   names a workspace package must likewise reference the interface name as
   a quoted literal in that package (or a launch remap). This catches
   "spec says X subscribes but the code subscribes a different topic".

3. Type presence: for workspace-owned, implemented interfaces the declared
   message/service/action class name must appear in the owner package.

4. Reverse pass: every ABSOLUTE `/agv/...` name passed as a string literal
   to a publisher/subscription/service/client/action creation call in
   src/ or fleet/ must be declared in the spec (ROS parameter/action
   plumbing endpoints are exempt). Relative names resolved via node
   namespaces cannot be attributed statically and are out of scope.

Entries with status planned/deprecated/proposed skip checks 1 and 3 (they
are spec-only placeholders with no owner-side implementation) but their
declared consumers ARE checked — a planned topic may already have real
subscribers waiting for it.

This is still a static, lexical check — it does not verify QoS depth or
runtime remapping. It exists to catch name/type drift and undeclared
interfaces, which were the failure modes observed in the 2026-04/07 audits.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:
    print("verify_interfaces: python3-yaml not installed", file=sys.stderr)
    sys.exit(2)

WS_ROOT = Path(__file__).resolve().parents[2]
SRC = WS_ROOT / "src"
FLEET = WS_ROOT / "fleet"
SPEC = WS_ROOT / "specs/interfaces.yaml"

# *.yaml included because topic names legitimately live in config files
# (the workspace rule is "all configuration from YAML or environment").
# The reverse pass is unaffected: it additionally requires a creation call
# next to the literal, which YAML never contains.
SOURCE_EXTS = ("*.cpp", "*.hpp", "*.h", "*.py", "*.ts", "*.yaml")
SKIP_DIR_PARTS = {"node_modules", "dist", "build", "install", "__pycache__", ".git"}

NAMESPACE_PREFIX = "/agv/"

CREATION_KEYWORDS = re.compile(
    r"create_publisher|create_subscription|create_generic_subscription|create_service"
    r"|create_client|create_server|createPublisher|createSubscription|createService"
    r"|createClient|ActionClient|ActionServer|rclcpp_action"
)

# ROS plumbing endpoints that every node exposes implicitly — not contract.
INFRA_SUFFIXES = (
    "/set_parameters", "/set_parameters_atomically", "/get_parameters",
    "/get_parameter_types", "/describe_parameters", "/list_parameters",
)


def pkg_dir(pkg_name: str) -> Path | None:
    for base in (SRC, FLEET):
        d = base / pkg_name
        if d.is_dir():
            return d
    return None


def iter_source_files(root: Path):
    for ext in SOURCE_EXTS:
        for f in root.rglob(ext):
            if any(part in SKIP_DIR_PARTS for part in f.parts):
                continue
            yield f


def read(f: Path) -> str:
    try:
        return f.read_text(errors="ignore")
    except OSError:
        return ""


def literal_re(needle: str) -> re.Pattern:
    """Quoted-string-literal match: `"...prefix/needle"` or `"needle"`.

    The needle must end at the closing quote so `wheel_odom` does not match
    inside `wheel_odom_validated`. Template/f-string prefixes such as
    `/${NAMESPACE}/` or f`/{NS}/` are covered by the permissive prefix class.
    """
    return re.compile(r"[\"'`](?:[^\"'`\n]*/)?" + re.escape(needle) + r"[\"'`]")


def candidates_for(name: str) -> list[str]:
    """Name forms code may plausibly quote for a declared interface."""
    cands = [name]
    if name.startswith(NAMESPACE_PREFIX):
        cands.append(name[len(NAMESPACE_PREFIX):])
    elif name.startswith("/"):
        cands.append(name[1:])
    return cands


def find_literal_in_dir(root: Path, name: str) -> bool:
    pats = [literal_re(c) for c in candidates_for(name)]
    for f in iter_source_files(root):
        text = read(f)
        if any(p.search(text) for p in pats):
            return True
    return False


_launch_cache: str | None = None


def launch_corpus() -> str:
    """Concatenated text of all launch files — used as a remap fallback."""
    global _launch_cache
    if _launch_cache is None:
        parts: list[str] = []
        for pkg in sorted(SRC.iterdir()):
            ld = pkg / "launch"
            if ld.is_dir():
                for f in ld.glob("*.py"):
                    parts.append(read(f))
        _launch_cache = "\n".join(parts)
    return _launch_cache


def found_in_pkg_or_launch(pkg_name: str, name: str) -> tuple[bool, str]:
    d = pkg_dir(pkg_name)
    if d is None:
        return False, f"package dir not found (src/{pkg_name} or fleet/{pkg_name})"
    if find_literal_in_dir(d, name):
        return True, ""
    corpus = launch_corpus()
    if any(literal_re(c).search(corpus) for c in candidates_for(name)):
        return True, ""
    return False, f"no quoted literal for '{name}' in {d.relative_to(WS_ROOT)} nor in any launch file"


def is_workspace_pkg(pkg_field: str) -> str | None:
    """Return the bare package name if the pkg field names a workspace pkg."""
    if not isinstance(pkg_field, str):
        return None
    bare = pkg_field.split(" ", 1)[0].strip()
    if pkg_dir(bare) is not None:
        return bare
    return None


def type_class_name(type_field: str) -> str | None:
    if not isinstance(type_field, str) or "/" not in type_field:
        return None
    return type_field.rsplit("/", 1)[-1]


def check_interface(iface: dict, kind: str, errors: list[str]) -> None:
    name = iface.get("name")
    if not name:
        errors.append(f"FAIL: {kind} entry has no name: {iface}")
        return
    status = iface.get("status", "")
    owner_pkg = is_workspace_pkg(iface.get("owner_pkg", ""))

    # 1 + 3: owner presence and type presence (implemented, workspace-owned).
    if owner_pkg and status not in ("planned", "deprecated", "proposed"):
        ok, why = found_in_pkg_or_launch(owner_pkg, name)
        if not ok:
            errors.append(f"FAIL: {kind} '{name}' owner_pkg={owner_pkg}: {why}")
        else:
            cls = type_class_name(iface.get("type", ""))
            d = pkg_dir(owner_pkg)
            if cls and d is not None:
                if not any(cls in read(f) for f in iter_source_files(d)):
                    errors.append(
                        f"FAIL: {kind} '{name}' declares type '{iface.get('type')}' but class "
                        f"'{cls}' does not appear in {d.relative_to(WS_ROOT)}"
                    )

    # 2: declared consumers/producers that name workspace packages.
    for list_key in ("subscribers", "callers", "publishers", "alternative_publishers"):
        for entry in iface.get(list_key, []) or []:
            if not isinstance(entry, dict):
                continue
            pkg = is_workspace_pkg(entry.get("pkg", ""))
            if pkg is None:
                continue
            ok, why = found_in_pkg_or_launch(pkg, name)
            if not ok:
                errors.append(f"FAIL: {kind} '{name}' {list_key} entry pkg={pkg}: {why}")


ABS_LITERAL_RE = re.compile(
    r"[\"'`]"
    r"(?:/agv/|/\$\{[A-Za-z_]+\}/|/\{[A-Za-z_]+\}/)"
    r"(?P<rest>[A-Za-z0-9_/]+)"
    r"[\"'`]"
)


def reverse_pass(declared: set[str], errors: list[str]) -> None:
    """Flag absolute /agv/* literals used in creation calls but undeclared."""
    found: dict[str, set[str]] = {}
    for base in (SRC, FLEET):
        if not base.is_dir():
            continue
        for f in iter_source_files(base):
            text = read(f)
            for m in ABS_LITERAL_RE.finditer(text):
                ctx = text[max(0, m.start() - 240):m.start()]
                if not CREATION_KEYWORDS.search(ctx):
                    continue
                name = "/agv/" + m.group("rest")
                if "/_action/" in name or name.endswith(INFRA_SUFFIXES):
                    continue
                rel = f.relative_to(WS_ROOT)
                found.setdefault(name, set()).add(str(rel))
    for name in sorted(found):
        if name not in declared:
            refs = ", ".join(sorted(found[name])[:3])
            errors.append(
                f"FAIL: code creates interface on '{name}' ({refs}) but specs/interfaces.yaml "
                "does not declare it — add an entry or remove the interface"
            )


def main() -> int:
    if not SPEC.exists():
        print(f"FAIL: {SPEC} does not exist")
        return 1
    with SPEC.open() as f:
        data = yaml.safe_load(f) or {}

    errors: list[str] = []
    declared: set[str] = set()

    for kind, key in (("topic", "topics"), ("service", "services"), ("action", "actions")):
        for iface in data.get(key, []) or []:
            if isinstance(iface, dict) and iface.get("name"):
                declared.add(iface["name"])
    for kind, key in (("topic", "topics"), ("service", "services"), ("action", "actions")):
        for iface in data.get(key, []) or []:
            if isinstance(iface, dict):
                check_interface(iface, kind, errors)

    reverse_pass(declared, errors)

    if errors:
        for e in errors:
            print(e)
        print(f"verify_interfaces: {len(errors)} violation(s)")
        print("Fix: update the code to match the declared interface, OR update/add the spec entry.")
        return 1

    print("verify_interfaces: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
