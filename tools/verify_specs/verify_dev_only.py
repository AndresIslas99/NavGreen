#!/usr/bin/env python3
"""verify_dev_only — Rule 0 enforcement.

Every Python ROS 2 node launched by a production launch file must belong to
a package whose TASK.yaml declares `dev_only: true` (interim dev tooling) —
otherwise it must be ported to C++17.

A node counts as Python if EITHER:
  - its `executable` ends in `.py` (script installed by any package), OR
  - its owning package is an ament_python package (entry-point console
    scripts have no `.py` suffix, so the suffix test alone is blind to them).

Node(...) calls are parsed with a balanced-parenthesis scan and
order-independent kwarg extraction, so `Node(condition=IfCondition(x),
executable=..., package=...)` is not silently skipped.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:
    print("verify_dev_only: python3-yaml not installed", file=sys.stderr)
    sys.exit(2)

WS_ROOT = Path(__file__).resolve().parents[2]
SRC = WS_ROOT / "src"

# Launch files reachable from agv_start.sh in production modes (real,
# mapping, hil_full). agv_hil_full.launch.py (AGV_MODE=hil) is HIL-only and
# excluded. The cuVSLAM stack (agv_slam) is an external deploy-time package
# not present in this workspace, so it has no launch file to scan here.
PRODUCTION_LAUNCH_PATTERNS = [
    "agv_full.launch.py",
    "agv_mapping.launch.py",
]


def read_task_yaml(pkg_dir: Path) -> tuple[dict | None, str | None]:
    """Return (data, error). A parse error is returned, never swallowed."""
    task_file = pkg_dir / "TASK.yaml"
    if not task_file.exists():
        return None, None
    try:
        with task_file.open() as f:
            return (yaml.safe_load(f) or {}), None
    except Exception as exc:  # noqa: BLE001 — report any parse failure loudly
        return None, f"FAIL: cannot parse {task_file.relative_to(WS_ROOT)}: {exc}"


def collect_dev_only_packages() -> tuple[set[str], list[str]]:
    out: set[str] = set()
    errors: list[str] = []
    for pkg_dir in sorted(SRC.iterdir()):
        if not pkg_dir.is_dir():
            continue
        data, err = read_task_yaml(pkg_dir)
        if err:
            errors.append(err)
            continue
        if data and data.get("dev_only") is True:
            out.add(pkg_dir.name)
    return out, errors


def is_ament_python_pkg(pkg_name: str) -> bool:
    """True if src/<pkg> is an ament_python package (entry-point nodes)."""
    pkg_dir = SRC / pkg_name
    if not pkg_dir.is_dir():
        return False
    if (pkg_dir / "setup.py").exists() or (pkg_dir / "setup.cfg").exists():
        return True
    pkg_xml = pkg_dir / "package.xml"
    if pkg_xml.exists():
        try:
            if "ament_python" in pkg_xml.read_text(errors="ignore"):
                return True
        except OSError:
            pass
    return False


def find_production_launches() -> tuple[list[Path], list[str]]:
    launches: list[Path] = []
    matched: set[str] = set()
    errors: list[str] = []
    for pkg_dir in sorted(SRC.iterdir()):
        launch_dir = pkg_dir / "launch"
        if not launch_dir.is_dir():
            continue
        for lf in sorted(launch_dir.iterdir()):
            if any(lf.name == p for p in PRODUCTION_LAUNCH_PATTERNS):
                launches.append(lf)
                matched.add(lf.name)
    for pattern in PRODUCTION_LAUNCH_PATTERNS:
        if pattern not in matched:
            errors.append(
                f"FAIL: production launch pattern '{pattern}' matches no file under src/*/launch/ "
                "— update PRODUCTION_LAUNCH_PATTERNS in verify_dev_only.py"
            )
    return launches, errors


NODE_CALL_RE = re.compile(r"\b(?:Node|ComposableNode)\s*\(")
KWARG_STR_RE = {
    "package": re.compile(r"\bpackage\s*=\s*['\"](?P<v>[^'\"]+)['\"]"),
    "executable": re.compile(r"\bexecutable\s*=\s*['\"](?P<v>[^'\"]+)['\"]"),
    "plugin": re.compile(r"\bplugin\s*=\s*['\"](?P<v>[^'\"]+)['\"]"),
}


def extract_call_bodies(text: str) -> list[str]:
    """Return the balanced-paren argument text of every Node(...) call."""
    bodies: list[str] = []
    for m in NODE_CALL_RE.finditer(text):
        depth = 1
        i = m.end()
        start = i
        in_str: str | None = None
        while i < len(text) and depth > 0:
            c = text[i]
            if in_str:
                if c == "\\":
                    i += 2
                    continue
                if c == in_str:
                    in_str = None
            elif c in "'\"":
                in_str = c
            elif c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
            i += 1
        if depth == 0:
            bodies.append(text[start:i - 1])
    return bodies


def find_python_executables(launch_file: Path) -> list[tuple[str, str, str]]:
    """Return (package, executable, reason) for Python-looking nodes.

    Nodes whose package/executable are non-literal (LaunchConfiguration etc.)
    cannot be resolved statically and are skipped — production launches use
    literals for both today.
    """
    text = launch_file.read_text(errors="ignore")
    out: list[tuple[str, str, str]] = []
    for body in extract_call_bodies(text):
        pm = KWARG_STR_RE["package"].search(body)
        em = KWARG_STR_RE["executable"].search(body)
        if not pm or not em:
            continue
        pkg, exe = pm.group("v"), em.group("v")
        if exe.endswith(".py"):
            out.append((pkg, exe, "executable ends in .py"))
        elif is_ament_python_pkg(pkg):
            out.append((pkg, exe, f"package {pkg} is ament_python (entry-point node)"))
    return out


def main() -> int:
    dev_only_pkgs, task_errors = collect_dev_only_packages()
    launches, launch_errors = find_production_launches()
    violations: list[str] = list(task_errors) + list(launch_errors)

    for lf in launches:
        for pkg, exe, reason in find_python_executables(lf):
            if pkg not in dev_only_pkgs:
                violations.append(
                    f"FAIL: {lf.relative_to(WS_ROOT)} launches Python node {exe} from package {pkg} "
                    f"({reason}) which does NOT have dev_only: true in TASK.yaml"
                )

    if violations:
        for v in violations:
            print(v)
        print(f"verify_dev_only: {len(violations)} violation(s)")
        print("Fix: add `dev_only: true` to src/<pkg>/TASK.yaml OR port the script to C++17.")
        return 1

    print("verify_dev_only: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
