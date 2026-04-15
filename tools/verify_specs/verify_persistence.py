#!/usr/bin/env python3
"""verify_persistence — check that writers and readers in specs/persistence.yaml exist in code.

For each artifact:
- Resolve the writer.code_ref path prefix (file, no line number).
- Verify the file exists under src/.
- (Weak check — we do not verify the line actually contains a write operation.
  verify_persistence is WARNING severity because it's approximate.)
"""

from __future__ import annotations

import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:
    print("verify_persistence: python3-yaml not installed", file=sys.stderr)
    sys.exit(2)

WS_ROOT = Path(__file__).resolve().parents[2]
SPEC = WS_ROOT / "specs/persistence.yaml"


def normalize_ref(ref: str) -> str:
    """Return the bare path from a code_ref string.

    code_ref values are free-form strings that may include:
    - `path:line` — strip the line suffix
    - `path:line:line` — strip to the first colon
    - `path::function::path` — only the prefix is a real path
    - `path (human note)` — strip the parenthetical
    """
    # Strip any parenthetical note.
    ref = ref.split(" (", 1)[0]
    # Take the substring before the first :: (C++-style scope).
    ref = ref.split("::", 1)[0]
    # Take the substring before the first : (line number).
    ref = ref.split(":", 1)[0]
    return ref.strip()


def _iter_writers(art: dict):
    """Yield individual writer dicts from an artifact entry.

    Some artifacts have a single `writer:` mapping, others have
    `writers:` (plural) which is a list of mappings, and some of those list
    items may be strings (free-form alternative publishers). We only yield
    dict-typed entries with a possible `code_ref`.
    """
    if isinstance(art.get("writer"), dict):
        yield art["writer"]
    for w in art.get("writers", []) or []:
        if isinstance(w, dict):
            yield w


def check_artifact(art: dict, name: str) -> list[str]:
    errors: list[str] = []
    if not isinstance(art, dict):
        return errors
    for writer in _iter_writers(art):
        code_ref = writer.get("code_ref")
        if not code_ref:
            continue
        path = WS_ROOT / normalize_ref(code_ref)
        if not path.exists():
            errors.append(f"WARN: artifact '{name}' writer.code_ref points to missing file: {code_ref}")
    for reader in art.get("readers", []) or []:
        if not isinstance(reader, dict):
            continue
        rref = reader.get("code_ref")
        if rref:
            rpath = WS_ROOT / normalize_ref(rref)
            if not rpath.exists():
                errors.append(f"WARN: artifact '{name}' reader.code_ref points to missing file: {rref}")
    return errors


def main() -> int:
    if not SPEC.exists():
        print(f"FAIL: {SPEC} does not exist")
        return 1
    with SPEC.open() as f:
        data = yaml.safe_load(f) or {}

    warnings: list[str] = []

    for category in ("per_map_artifacts", "global_artifacts", "session_artifacts_to_migrate"):
        for name, art in (data.get(category, {}) or {}).items():
            warnings.extend(check_artifact(art, f"{category}.{name}"))

    if warnings:
        for w in warnings:
            print(w)
        print(f"verify_persistence: {len(warnings)} warning(s)")
        # WARNING severity — do not block commit
        return 0

    print("verify_persistence: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
