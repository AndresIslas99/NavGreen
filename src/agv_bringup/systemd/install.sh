#!/bin/bash
# Installs / refreshes the agv systemd units into /etc/systemd/system/.
# Run once with sudo after pulling a change that touches anything in this
# directory:
#
#   sudo src/agv_bringup/systemd/install.sh
#
# Safe to re-run — it copies, reloads, and restarts each unit idempotently.
# The existing drop-in /etc/systemd/system/agv.service.d/override.conf (if any)
# is left untouched so bench overrides persist across installs.

set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "ERROR: must run as root (sudo)" >&2
    exit 1
fi

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST_DIR="/etc/systemd/system"

UNITS=(
    agv.service
    agv-healthcheck.service
    agv-watchdog.service
)

for unit in "${UNITS[@]}"; do
    src="${SRC_DIR}/${unit}"
    dst="${DST_DIR}/${unit}"
    if [ ! -f "$src" ]; then
        echo "WARN: $src not found — skipping"
        continue
    fi
    install -m 0644 "$src" "$dst"
    echo "installed: $dst"
done

echo "---"
systemctl daemon-reload

# Enable but do not (re)start yet — the caller decides when to reboot or
# restart. Enabling is idempotent.
for unit in "${UNITS[@]}"; do
    systemctl enable "$unit" >/dev/null 2>&1 || true
done

echo "---"
echo "Next steps:"
echo "  sudo systemctl restart agv.service"
echo "  journalctl -fu agv.service -u agv-healthcheck.service -u agv-watchdog.service"
