/**
 * WebSocket hook for fleet manager connection.
 * Receives multi-robot positions from the fleet manager service.
 */

import { useEffect, useRef, useState, useCallback } from 'react'

export interface FleetRobot {
  id: string
  position: { x: number; y: number; theta: number }
  driving: boolean
  connectionState: string
  operatingMode: string
  errorCount: number
  orderId: string
  batteryCharge?: number
}

export interface FleetKPIs {
  total: number
  online: number
  driving: number
  idle: number
  errors: number
  avgBattery: number
  utilization: number
}

const FLEET_RECONNECT_BASE = 2000
const FLEET_RECONNECT_MAX = 10000

export function useFleetSocket() {
  const wsRef = useRef<WebSocket | null>(null)
  const [fleetConnected, setFleetConnected] = useState(false)
  const [robots, setRobots] = useState<FleetRobot[]>([])
  const [selectedRobot, setSelectedRobot] = useState<string | null>(null)

  useEffect(() => {
    let delay = FLEET_RECONNECT_BASE
    let timer: ReturnType<typeof setTimeout>
    let alive = true

    // Fleet manager runs on port 8091
    const fleetHost = location.hostname
    const fleetPort = '8091'

    function connect() {
      if (!alive) return
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      const ws = new WebSocket(`${proto}://${fleetHost}:${fleetPort}/ws/fleet`)
      wsRef.current = ws

      ws.onopen = () => {
        setFleetConnected(true)
        delay = FLEET_RECONNECT_BASE
      }

      ws.onclose = () => {
        setFleetConnected(false)
        wsRef.current = null
        if (alive) {
          timer = setTimeout(connect, delay)
          delay = Math.min(delay * 1.5, FLEET_RECONNECT_MAX)
        }
      }

      ws.onerror = () => ws.close()

      ws.onmessage = (ev) => {
        try {
          const msg = JSON.parse(ev.data)
          if (msg.type === 'fleet_state' || msg.type === 'fleet_update') {
            setRobots(msg.robots || [])
          }
        } catch { /* ignore */ }
      }
    }

    connect()
    return () => {
      alive = false
      clearTimeout(timer)
      wsRef.current?.close()
    }
  }, [])

  const selectRobot = useCallback((id: string | null) => {
    setSelectedRobot(id)
  }, [])

  return { fleetConnected, robots, selectedRobot, selectRobot }
}
