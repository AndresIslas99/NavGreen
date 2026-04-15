#!/bin/bash
# install_git_hook — install the pre-commit hook that runs tools/verify_specs/all.sh
#
# Run from the workspace root:  bash tools/verify_specs/install_git_hook.sh
#
# The hook is idempotent. Re-running is safe.

set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

if [ ! -d .git ]; then
  echo "FAIL: not in a git repository root"
  exit 1
fi

HOOK=".git/hooks/pre-commit"

cat > "$HOOK" <<'HOOK_EOF'
#!/bin/bash
# AGV pre-commit hook — runs tools/verify_specs/all.sh
#
# This hook is installed by tools/verify_specs/install_git_hook.sh.
# Blocks commits that violate BLOCKING specs. WARNING findings are printed
# but do not block.
#
# Bypass with:  git commit --no-verify   (NOT RECOMMENDED)

WS_ROOT="$(git rev-parse --show-toplevel)"
cd "$WS_ROOT"

if [ ! -x tools/verify_specs/all.sh ]; then
  echo "pre-commit: tools/verify_specs/all.sh not found or not executable"
  echo "pre-commit: skipping verify (is the repo fully set up?)"
  exit 0
fi

bash tools/verify_specs/all.sh
exit $?
HOOK_EOF

chmod +x "$HOOK"
echo "installed: $HOOK"
echo
echo "The hook will now run tools/verify_specs/all.sh before every commit."
echo "Bypass (emergency only): git commit --no-verify"
