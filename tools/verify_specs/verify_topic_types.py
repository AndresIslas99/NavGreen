#!/usr/bin/env python3
"""verify_topic_types.py — BLOCKING

Catches the bug class that has hit the AGV three times already:
a `create_subscription<T>` or `create_publisher<T>` declared with the
wrong ROS message type, silently dropped by DDS, never wired in
practice. Original 2026-04-13 audit bug #1 (safety_supervisor),
CRITICAL-11-A-01 (mode_arbiter), G4/Section-0 (rail_driver). The fourth
occurrence won't happen if this check is wired into the suite.

Approach
--------
1. Build the canonical type map from `specs/interfaces.yaml`
   ({topic_name: canonical_type}).
2. For each `.cpp` file under `src/`:
   - Resolve `declare_parameter<std::string>("P", "DEFAULT")`
     and `declare_parameter("P", "DEFAULT")` calls into param→default.
   - Resolve `auto v = get_parameter("P").as_string()` into var→param.
   - Match every `create_subscription<TYPE>(X, ...)` and
     `create_publisher<TYPE>(X, ...)` call, where X is either a literal
     string or a variable resolved via the above.
3. For each discovered (topic, type) declaration, cross-check against
   the canonical type in `specs/interfaces.yaml`. Mismatches are
   BLOCKING.

Limitations (intentional v1):
- TypeScript / rclnodejs not parsed yet (rail_driver-class bugs are
  all C++; agv_ui_backend uses rclnodejs.createSubscription with the
  IDL string, which the runtime catches anyway).
- Macro-generated subscriptions and pure-template types not handled.
- Topics passed via shared header constants not handled. Add cases as
  they appear; the file enumerates exactly which patterns work today.

Exemptions
----------
A declaration that intentionally disagrees with the spec (e.g., a
documented dual-type side-channel) can be exempted by placing
`verify_topic_types: allow(<reason>)` in a comment on the create_* line
or on either of the two lines above it. The reason is mandatory —
a bare `allow` without parentheses is NOT honored.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import yaml

WS_ROOT = Path(__file__).resolve().parent.parent.parent
SPEC_PATH = WS_ROOT / "specs" / "interfaces.yaml"
SRC_ROOT = WS_ROOT / "src"

# ─────────────────────────────────────────────────────────────────────────────
# 1. Canonical type map from spec
# ─────────────────────────────────────────────────────────────────────────────

def load_canonical_types() -> dict[str, str]:
    """Parse specs/interfaces.yaml → {topic_name: type}.

    The spec is the SSOT for message type per topic. The note at
    interfaces.yaml line 404 documents that this field was wrong in
    pre-2026-04-13 versions and explicitly calls out the audit context.
    """
    with open(SPEC_PATH) as f:
        spec = yaml.safe_load(f)
    out: dict[str, str] = {}
    for topic in spec.get("topics", []):
        name = topic.get("name")
        typ = topic.get("type")
        if name and typ:
            out[name] = typ
    return out

# ─────────────────────────────────────────────────────────────────────────────
# 2. C++ source parsing
# ─────────────────────────────────────────────────────────────────────────────

# `declare_parameter<std::string>("P", "DEFAULT")`
# `declare_parameter("P", "DEFAULT")`  (type inferred from second arg)
# `declare_parameter<std::string>("P", std::string("DEFAULT"))`
_DECLARE_PARAM_RE = re.compile(
    r"declare_parameter"
    r"(?:<\s*std::string\s*>)?"
    r"\s*\(\s*\"([^\"]+)\"\s*,\s*"
    r"(?:std::string\s*\(\s*)?"
    r"\"([^\"]+)\""
)

# `auto VAR = ... get_parameter("P").as_string()`
# `const auto VAR = ...`
# `std::string VAR = ...`
_GET_PARAM_STRING_RE = re.compile(
    r"(?:auto|const\s+auto|std::string)\s+(\w+)\s*=\s*"
    r"(?:this->)?\s*get_parameter\s*\(\s*\"([^\"]+)\"\s*\)"
    r"\s*\.\s*as_string\s*\(\s*\)"
)

# `create_subscription<TYPE>(EXPR, ...)`
# `create_publisher<TYPE>(EXPR, ...)`
# TYPE is a C++ identifier path (e.g., `nav2_msgs::msg::CollisionMonitorState`).
# EXPR can be a literal "/topic" or a bare identifier.
# `// verify_topic_types: allow(reason)` — inline exemption with a reason.
_ALLOW_RE = re.compile(r"verify_topic_types:\s*allow\(.+\)")

_CREATE_RE = re.compile(
    r"create_(subscription|publisher)\s*<\s*([\w:]+)\s*>\s*\(\s*"
    r"(\"[^\"]+\"|[\w_]+)\s*,"
)

# ROS C++ message types use `pkg::msg::Type`. Convert to spec form
# `pkg/msg/Type` for comparison.
def cpp_type_to_spec(t: str) -> str:
    return t.replace("::", "/")


def parse_cpp_file(path: Path) -> list[tuple[str, str, str, str, int, bool]]:
    """Return list of (topic_name, spec_type, kind, file, line) discovered.

    Topic name can be either a literal (`"/agv/foo"` → `/agv/foo`) or a
    parameter-default string (resolved via declare_parameter → get_parameter
    → variable name → create_*).
    """
    try:
        text = path.read_text()
    except UnicodeDecodeError:
        return []
    lines = text.splitlines()

    # Step A: param defaults
    param_defaults: dict[str, str] = {}
    for m in _DECLARE_PARAM_RE.finditer(text):
        param_defaults[m.group(1)] = m.group(2)

    # Step B: var → param mapping
    var_to_param: dict[str, str] = {}
    for m in _GET_PARAM_STRING_RE.finditer(text):
        var_to_param[m.group(1)] = m.group(2)

    # Step C: create_* calls
    out: list[tuple[str, str, str, str, int, bool]] = []
    for m in _CREATE_RE.finditer(text):
        kind = m.group(1)              # "subscription" or "publisher"
        cpp_type = m.group(2)
        expr = m.group(3)

        if expr.startswith('"'):
            topic = expr.strip('"')
        else:
            # bare identifier — resolve to param default
            param = var_to_param.get(expr)
            if not param:
                continue                # cannot resolve this declaration
            default = param_defaults.get(param)
            if not default:
                continue
            topic = default

        if not topic.startswith("/"):
            # rclcpp prepends the namespace at runtime; in spec topics
            # start with `/agv/...`. Add the default ns to make matching
            # work for the namespace-relative form.
            topic = "/agv/" + topic.lstrip("/")

        # Find line number
        line = text.count("\n", 0, m.start()) + 1
        # Exemption pragma: look at the call line and the two lines above.
        window = lines[max(0, line - 3):line]
        allowed = any(_ALLOW_RE.search(w) for w in window)
        out.append((topic, cpp_type_to_spec(cpp_type), kind,
                    str(path.relative_to(WS_ROOT)), line, allowed))
    return out

# ─────────────────────────────────────────────────────────────────────────────
# 3. Verifier
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    canonical = load_canonical_types()

    # Only scan AGV-owned packages. External pkgs (isaac_ros_*, zed-ros2-*,
    # nvblox*, slam_toolbox, etc.) live under src/ but are not in scope for
    # this verifier — the spec covers only the AGV-internal contract.
    def is_in_scope(path: Path) -> bool:
        try:
            parts = path.relative_to(SRC_ROOT).parts
        except ValueError:
            return False
        if not parts:
            return False
        return parts[0].startswith("agv_")

    decls: list[tuple[str, str, str, str, int, bool]] = []
    for cpp in SRC_ROOT.rglob("*.cpp"):
        if not is_in_scope(cpp):
            continue
        if "/build/" in str(cpp) or "/install/" in str(cpp):
            continue
        decls.extend(parse_cpp_file(cpp))
    for hpp in SRC_ROOT.rglob("*.hpp"):
        if not is_in_scope(hpp):
            continue
        if "/build/" in str(hpp) or "/install/" in str(hpp):
            continue
        decls.extend(parse_cpp_file(hpp))

    errors: list[str] = []
    gaps: list[str] = []      # topics not in spec — coverage info, NOT a warning
    counted = 0

    exempted = 0
    for topic, code_type, kind, file, line, allowed in decls:
        spec_type = canonical.get(topic)
        if spec_type is None:
            gaps.append(
                f"  {file}:{line} {kind}<{code_type}>({topic})"
            )
            continue
        counted += 1
        if code_type != spec_type and allowed:
            exempted += 1
            continue
        if code_type != spec_type:
            errors.append(
                f"FAIL: {file}:{line} {kind}<{code_type}>({topic})\n"
                f"      spec says: {spec_type}\n"
                f"      Same bug class as 2026-04-13 audit bug #1, "
                f"CRITICAL-11-A-01 (mode_arbiter), and G4/Section-0 "
                f"(rail_driver). DDS will drop messages silently."
            )

    print(f"verify_topic_types: scanned {len(decls)} create_* declarations "
          f"in AGV packages, {counted} cross-checked against spec, "
          f"{len(errors)} type mismatch(es), {exempted} exempted by pragma.")
    print(f"  (informational: {len(gaps)} topic-declaration(s) not in "
          f"specs/interfaces.yaml — coverage gap, run with VERBOSE=1 to "
          f"list. Not a verifier warning.)")
    if errors == [] and len(gaps) and __import__("os").environ.get("VERBOSE"):
        print("VERBOSE — gaps follow:")
        for g in gaps:
            print(g)

    if errors:
        print()
        for e in errors:
            print(e)
        print()
        print("BLOCKING: at least one declaration's message type disagrees "
              "with the canonical type in specs/interfaces.yaml. Either fix "
              "the source code or update the spec — they must agree.")
        return 1

    print("verify_topic_types: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
