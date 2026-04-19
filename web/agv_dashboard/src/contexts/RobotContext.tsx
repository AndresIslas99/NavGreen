/**
 * RobotContext — centralized state from useWebSocket.
 * Eliminates prop-drilling of status, path, scanPoints, etc. through 3+ levels.
 *
 * Usage:
 *   <RobotProvider><App /></RobotProvider>   (in main.tsx)
 *   const { status, send } = useRobot()      (in any component)
 */

import { createContext, useContext } from 'react'
import { useWebSocket } from '../hooks/useWebSocket'
import type { RobotStatus, MapUpdate, PathPoint, LogEntry } from '../api/types'

export interface RobotContextValue {
  connected: boolean
  status: RobotStatus | null
  path: PathPoint[]
  scanPoints: PathPoint[]
  mapData: MapUpdate | null
  accMapData: MapUpdate | null
  events: LogEntry[]
  send: (data: Record<string, unknown>) => void
}

const RobotContext = createContext<RobotContextValue | null>(null)

export function RobotProvider({ children }: { children: React.ReactNode }) {
  const ws = useWebSocket()
  return <RobotContext.Provider value={ws}>{children}</RobotContext.Provider>
}

export function useRobot(): RobotContextValue {
  const ctx = useContext(RobotContext)
  if (!ctx) throw new Error('useRobot must be used within RobotProvider')
  return ctx
}
