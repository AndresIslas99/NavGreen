#!/bin/bash
# setup_production_network.sh — One-time network configuration for production Jetson
#
# Disables WiFi (persists across reboots) and configures a static IP on Ethernet
# so the AGV is always reachable at a known address for the dashboard and diagnostics.
#
# Usage:
#   sudo ./setup_production_network.sh [IP] [GATEWAY] [DNS]
#
# Defaults:
#   IP:      192.168.1.100/24
#   GATEWAY: 192.168.1.1
#   DNS:     8.8.8.8
#
# To re-enable WiFi temporarily (e.g., for firmware updates):
#   sudo nmcli radio wifi on
#   sudo nmcli device wifi connect "SSID" password "PASS"
#   # When done:
#   sudo nmcli radio wifi off

set -euo pipefail

IP="${1:-192.168.1.100/24}"
GATEWAY="${2:-192.168.1.1}"
DNS="${3:-8.8.8.8}"
IFACE="eno1"

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Run with sudo"
    exit 1
fi

echo "=== AGV Production Network Setup ==="
echo "  Interface: $IFACE"
echo "  IP:        $IP"
echo "  Gateway:   $GATEWAY"
echo "  DNS:       $DNS"
echo ""

# Step 1: Disable WiFi radio (persists across reboots)
echo "[1/4] Disabling WiFi radio..."
nmcli radio wifi off
echo "  WiFi disabled."

# Step 2: Configure static IP on Ethernet
echo "[2/4] Configuring static IP on $IFACE..."
nmcli connection modify "$IFACE" \
    ipv4.method manual \
    ipv4.addresses "$IP" \
    ipv4.gateway "$GATEWAY" \
    ipv4.dns "$DNS"
echo "  Static IP configured."

# Step 3: Set Ethernet route priority (lower metric = higher priority)
echo "[3/4] Setting Ethernet route priority..."
nmcli connection modify "$IFACE" ipv4.route-metric 100
nmcli connection modify "$IFACE" ipv6.route-metric 100
echo "  Route metric set to 100 (highest priority)."

# Step 4: Apply changes
echo "[4/4] Applying configuration..."
nmcli connection up "$IFACE"
echo "  Connection restarted."

echo ""
echo "=== Done ==="
echo "  WiFi:     OFF"
echo "  Ethernet: $IP on $IFACE"
echo ""
echo "Verify with:"
echo "  nmcli device status"
echo "  ip addr show $IFACE"
echo "  ping -c 1 $GATEWAY"
