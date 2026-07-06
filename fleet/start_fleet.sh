#!/bin/bash
# AGV Fleet Infrastructure — Start/Stop/Status
# Usage: ./start_fleet.sh [start|stop|status]

set -e

FLEET_DIR="$(cd "$(dirname "$0")" && pwd)"
VDA_MQTT_BROKER="${VDA_MQTT_BROKER:-mqtt://localhost:1883}"
# 8091 is taken by agv_image_server on the same Jetson (specs/project.yaml).
FLEET_PORT="${FLEET_PORT:-8092}"
VDA_MANUFACTURER="${VDA_MANUFACTURER:-agv-greenhouse}"
VDA_SERIAL_NUMBER="${VDA_SERIAL_NUMBER:-agv-001}"
AGV_NAMESPACE="${AGV_NAMESPACE:-agv}"
# Optional security knobs (empty = disabled; see fleet/README.md):
#   FLEET_API_TOKEN     shared secret for the fleet REST/WebSocket API
#   VDA_MQTT_USERNAME / VDA_MQTT_PASSWORD  broker credentials
FLEET_API_TOKEN="${FLEET_API_TOKEN:-}"
VDA_MQTT_USERNAME="${VDA_MQTT_USERNAME:-}"
VDA_MQTT_PASSWORD="${VDA_MQTT_PASSWORD:-}"

PID_DIR="/tmp/agv_fleet_pids"
mkdir -p "$PID_DIR"

start() {
    echo "Starting AGV fleet infrastructure..."

    # 1. Start Mosquitto
    echo "[1/3] Starting Mosquitto MQTT broker..."
    cd "$FLEET_DIR"
    docker compose up -d mosquitto
    echo "  Waiting for MQTT..."
    for i in $(seq 1 10); do
        if docker compose exec -T mosquitto mosquitto_sub -t '$SYS/broker/version' -C 1 -W 1 2>/dev/null; then
            echo "  MQTT ready."
            break
        fi
        sleep 1
    done

    # 2. Start Fleet Manager
    echo "[2/3] Starting Fleet Manager on port $FLEET_PORT..."
    cd "$FLEET_DIR/agv_fleet_manager"
    VDA_MQTT_BROKER="$VDA_MQTT_BROKER" FLEET_PORT="$FLEET_PORT" \
        FLEET_API_TOKEN="$FLEET_API_TOKEN" \
        VDA_MQTT_USERNAME="$VDA_MQTT_USERNAME" VDA_MQTT_PASSWORD="$VDA_MQTT_PASSWORD" \
        node dist/index.js &
    echo $! > "$PID_DIR/fleet_manager.pid"
    echo "  Fleet Manager PID: $(cat "$PID_DIR/fleet_manager.pid")"

    # 3. Start VDA 5050 Adapter
    echo "[3/3] Starting VDA 5050 Adapter ($VDA_MANUFACTURER/$VDA_SERIAL_NUMBER)..."
    cd "$FLEET_DIR/agv_vda5050_adapter"
    # Note: when broker auth is enabled, the adapter defaults its username to
    # VDA_SERIAL_NUMBER (per the ACL) — do not export a fleet-wide
    # VDA_MQTT_USERNAME here unless every service should share it.
    VDA_MQTT_BROKER="$VDA_MQTT_BROKER" VDA_MANUFACTURER="$VDA_MANUFACTURER" \
        VDA_SERIAL_NUMBER="$VDA_SERIAL_NUMBER" AGV_NAMESPACE="$AGV_NAMESPACE" \
        VDA_MQTT_USERNAME="$VDA_MQTT_USERNAME" VDA_MQTT_PASSWORD="$VDA_MQTT_PASSWORD" \
        node dist/index.js &
    echo $! > "$PID_DIR/vda5050_adapter.pid"
    echo "  VDA Adapter PID: $(cat "$PID_DIR/vda5050_adapter.pid")"

    echo ""
    echo "Fleet infrastructure running."
    echo "  MQTT: localhost:1883"
    echo "  Fleet Manager: http://localhost:$FLEET_PORT"
    echo "  VDA Adapter: $VDA_MANUFACTURER/$VDA_SERIAL_NUMBER"
}

stop() {
    echo "Stopping AGV fleet infrastructure..."

    if [ -f "$PID_DIR/vda5050_adapter.pid" ]; then
        kill "$(cat "$PID_DIR/vda5050_adapter.pid")" 2>/dev/null || true
        rm -f "$PID_DIR/vda5050_adapter.pid"
        echo "  VDA Adapter stopped."
    fi

    if [ -f "$PID_DIR/fleet_manager.pid" ]; then
        kill "$(cat "$PID_DIR/fleet_manager.pid")" 2>/dev/null || true
        rm -f "$PID_DIR/fleet_manager.pid"
        echo "  Fleet Manager stopped."
    fi

    cd "$FLEET_DIR"
    docker compose stop mosquitto 2>/dev/null || true
    echo "  Mosquitto stopped."

    echo "Fleet infrastructure stopped."
}

status() {
    echo "AGV Fleet Status:"

    # Mosquitto
    if docker compose -f "$FLEET_DIR/docker-compose.yaml" ps mosquitto 2>/dev/null | grep -q "running"; then
        echo "  Mosquitto: RUNNING"
    else
        echo "  Mosquitto: STOPPED"
    fi

    # Fleet Manager
    if [ -f "$PID_DIR/fleet_manager.pid" ] && kill -0 "$(cat "$PID_DIR/fleet_manager.pid")" 2>/dev/null; then
        echo "  Fleet Manager: RUNNING (PID $(cat "$PID_DIR/fleet_manager.pid"))"
    else
        echo "  Fleet Manager: STOPPED"
    fi

    # VDA Adapter
    if [ -f "$PID_DIR/vda5050_adapter.pid" ] && kill -0 "$(cat "$PID_DIR/vda5050_adapter.pid")" 2>/dev/null; then
        echo "  VDA Adapter: RUNNING (PID $(cat "$PID_DIR/vda5050_adapter.pid"))"
    else
        echo "  VDA Adapter: STOPPED"
    fi
}

case "${1:-start}" in
    start)  start ;;
    stop)   stop ;;
    status) status ;;
    restart) stop; sleep 2; start ;;
    *)
        echo "Usage: $0 [start|stop|status|restart]"
        exit 1
        ;;
esac
