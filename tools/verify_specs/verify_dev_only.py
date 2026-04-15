#!/usr/bin/env python3
"""verify_dev_only — Rule 0 enforcement.

Every `.py` file that is launched as a ROS 2 node in a production launch file
must have `dev_only: true` in its package's TASK.yaml.

Scans launch files for `Node(...executable=...).py...)` or direct
`executable="something.py"` references.
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

# Launch files under this package are considered "production" if they are
# reachable from agv_start.sh. This list is conservative — if a launch file
# is not used in production, operators can mark it in a future exclusion file.
PRODUCTION_LAUNCH_PATTERNS = [
    "agv_full.launch.py",
    "agv_slam.launch.py",
    "agv_mapping.launch.py",
    # agv_hil_full.launch.py is HIL, excluded from production
    # agv_teleop.launch.py is commissioning
]

# A `.py` in these packages is exempt (dev tooling, not runtime).
EXEMPT_PACKAGES = set()  # populated from TASK.yaml dev_only flags


def read_task_yaml(pkg_dir: Path) -> dict | None:
    task_file = pkg_dir / "TASK.yaml"
    if not task_file.exists():
        return None
    try:
        with task_file.open() as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return None


def collect_dev_only_packages() -> set[str]:
    out = set()
    for pkg_dir in SRC.iterdir():
        if not pkg_dir.is_dir():
            continue
        data = read_task_yaml(pkg_dir)
        if data and data.get("dev_only") is True:
            out.add(pkg_dir.name)
    return out


def find_production_launches() -> list[Path]:
    launches: list[Path] = []
    for pkg_dir in SRC.iterdir():
        launch_dir = pkg_dir / "launch"
        if not launch_dir.is_dir():
            continue
        for lf in launch_dir.iterdir():
            if any(lf.name == p for p in PRODUCTION_LAUNCH_PATTERNS):
                launches.append(lf)
    return launches


# Match `Node(package='pkg_name', executable='something.py', ...)` or
# equivalent `executable="something.py"`.
NODE_RE = re.compile(
    r"Node\s*\([^)]*?"
    r"package\s*=\s*['\"](?P<pkg>[^'\"]+)['\"]"
    r"[^)]*?"
    r"executable\s*=\s*['\"](?P<exe>[^'\"]+)['\"]",
    re.DOTALL,
)


def find_python_executables(launch_file: Path) -> list[tuple[str, str]]:
    """Return list of (package, executable) pairs that look like Python scripts."""
    text = launch_file.read_text()
    out: list[tuple[str, str]] = []
    for m in NODE_RE.finditer(text):
        pkg = m.group("pkg")
        exe = m.group("exe")
        if exe.endswith(".py"):
            out.append((pkg, exe))
    return out


def main() -> int:
    dev_only_pkgs = collect_dev_only_packages()
    launches = find_production_launches()
    violations = []

    for lf in launches:
        py_execs = find_python_executables(lf)
        for pkg, exe in py_execs:
            if pkg not in dev_only_pkgs:
                violations.append((lf, pkg, exe))

    if violations:
        for lf, pkg, exe in violations:
            print(f"FAIL: {lf} launches Python node {exe} from package {pkg} "
                  f"which does NOT have dev_only: true in TASK.yaml")
        print(f"verify_dev_only: {len(violations)} violation(s)")
        print("Fix: add `dev_only: true` to src/<pkg>/TASK.yaml OR port the script to C++17.")
        return 1

    print("verify_dev_only: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
