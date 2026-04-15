#!/bin/bash
# verify_canonical_sources — the 5 canonical source files must exist, be non-empty, and have headers.

set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

CANONICAL=(
  "specs/project.yaml"
  "specs/interfaces.yaml"
  "specs/acceptance.yaml"
  "specs/state_machine.yaml"
  "specs/persistence.yaml"
  "specs/launch_sequence.yaml"
  "specs/README.md"
  "agents/registry.yaml"
  "policies/engineering_rules.md"
  "CLAUDE.md"
)

violations=0

for f in "${CANONICAL[@]}"; do
  if [ ! -f "$f" ]; then
    echo "FAIL: missing canonical source: $f"
    violations=$((violations + 1))
    continue
  fi
  if [ ! -s "$f" ]; then
    echo "FAIL: empty canonical source: $f"
    violations=$((violations + 1))
    continue
  fi
  # YAML files should have a spec_version header.
  if [[ "$f" == *.yaml ]]; then
    if ! grep -q '^spec_version:' "$f"; then
      echo "FAIL: $f missing spec_version header"
      violations=$((violations + 1))
    fi
  fi
done

if [ "$violations" -gt 0 ]; then
  echo "verify_canonical_sources: $violations violation(s)"
  exit 1
fi

echo "verify_canonical_sources: OK"
exit 0
