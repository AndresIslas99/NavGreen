import { useEffect, useRef, useCallback, useState } from 'react'
import type { WsMessage, RobotStatus, PathPoint, MapUpdate, LogEntry } from '../api/types'

const RECONNECT_BASE = 500
const RECONNECT_MAX = 5000
const SCAN_FLUSH_MS = 500 // Debounce scan/path state updates to ~2Hz

export function useWebSocket() {
  const wsRef = useRef<WebSocket | null>(null)
  const [connected, setConnected] = useState(false)
  const [status, setStatus] = useState<RobotStatus | null>(null)
  const [path, setPath] = useState<PathPoint[]>([])
  const [scanPoints, setScanPoints] = useState<PathPoint[]>([])
  const [mapData, setMapData] = useState<MapUpdate | null>(null)
  const [accMapData, setAccMapData] = useState<MapUpdate | null>(null)
  const [events, setEvents] = useState<LogEntry[]>([])

  // High-frequency data stored in refs, flushed to state on debounce timer
  const scanBuf = useRef<PathPoint[]>([])
  const pathBuf = useRef<PathPoint[]>([])
  const scanDirty = useRef(false)
  const pathDirty = useRef(false)

  // Load event history on first connect
  const historyLoaded = useRef(false)

  // Debounce flush timer for scan/path
  useEffect(() => {
    const timer = setInterval(() => {
      if (scanDirty.current) {
        scanDirty.current = false
        setScanPoints(scanBuf.current)
      }
      if (pathDirty.current) {
        pathDirty.current = false
        setPath(pathBuf.current)
      }
    }, SCAN_FLUSH_MS)
    return () => clearInterval(timer)
  }, [])

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
            // Buffer path, flush on debounce timer
            pathBuf.current = (msg as { type: 'path'; points: PathPoint[] }).points
            pathDirty.current = true
          } else if (msg.type === 'scan') {
            // Buffer scan, flush on debounce timer
            scanBuf.current = (msg as { type: 'scan'; points: PathPoint[] }).points
            scanDirty.current = true
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
