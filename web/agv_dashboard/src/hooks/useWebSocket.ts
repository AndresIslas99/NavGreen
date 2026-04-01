import { useEffect, useRef, useCallback, useState } from 'react'
import type { WsMessage, RobotStatus, PathPoint, MapUpdate, LogEntry } from '../api/types'

const RECONNECT_BASE = 500
const RECONNECT_MAX = 5000

export function useWebSocket() {
  const wsRef = useRef<WebSocket | null>(null)
  const [connected, setConnected] = useState(false)
  const [status, setStatus] = useState<RobotStatus | null>(null)
  const [path, setPath] = useState<PathPoint[]>([])
  const [scanPoints, setScanPoints] = useState<PathPoint[]>([])
  const [mapData, setMapData] = useState<MapUpdate | null>(null)
  const [accMapData, setAccMapData] = useState<MapUpdate | null>(null)
  const [events, setEvents] = useState<LogEntry[]>([])

  // Load event history on first connect
  const historyLoaded = useRef(false)

  useEffect(() => {
    let delay = RECONNECT_BASE
    let timer: ReturnType<typeof setTimeout>
    let alive = true

    function connect() {
      if (!alive) return
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      const ws = new WebSocket(`${proto}://${location.host}/ws/control`)
      wsRef.current = ws

      ws.onopen = () => {
        setConnected(true)
        delay = RECONNECT_BASE
        // Load event history on first connect
        if (!historyLoaded.current) {
          historyLoaded.current = true
          fetch('/api/events?limit=200')
            .then(r => r.json())
            .then((evts: LogEntry[]) => setEvents(evts))
            .catch(() => {})
        }
      }

      ws.onclose = () => {
        setConnected(false)
        wsRef.current = null
        if (alive) {
          timer = setTimeout(connect, delay)
          delay = Math.min(delay * 1.5, RECONNECT_MAX)
        }
      }

      ws.onerror = () => ws.close()

      ws.onmessage = (ev) => {
        try {
          const msg: WsMessage = JSON.parse(ev.data)
          if (msg.type === 'status') {
            setStatus(msg as unknown as RobotStatus)
          } else if (msg.type === 'path') {
            setPath((msg as { type: 'path'; points: PathPoint[] }).points)
          } else if (msg.type === 'scan') {
            setScanPoints((msg as { type: 'scan'; points: PathPoint[] }).points)
          } else if (msg.type === 'map_update') {
            setMapData(msg as unknown as MapUpdate)
          } else if (msg.type === 'acc_map') {
            setAccMapData(msg as unknown as MapUpdate)
          } else if (msg.type === 'event') {
            const entry = msg as unknown as LogEntry
            setEvents(prev => [entry, ...prev].slice(0, 500))
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

  const send = useCallback((data: Record<string, unknown>) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(data))
    }
  }, [])

  return { connected, status, path, scanPoints, mapData, accMapData, events, send }
}
