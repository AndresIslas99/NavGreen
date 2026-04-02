/**
 * Fleet sidebar overlay — shows all robots with status indicators.
 * Appears when fleet manager is connected.
 */

import type { FleetRobot } from '../hooks/useFleetSocket'

interface Props {
  robots: FleetRobot[]
  selectedRobot: string | null
  onSelectRobot: (id: string | null) => void
  connected: boolean
}

const STATE_COLORS: Record<string, string> = {
  ONLINE: 'var(--normal)',
  OFFLINE: 'var(--dim)',
  CONNECTIONBROKEN: 'var(--red)',
}

export function FleetOverlay({ robots, selectedRobot, onSelectRobot, connected }: Props) {
  if (!connected || robots.length === 0) return null

  return (
    <div className="fleet-overlay">
      <div className="fleet-header">
        <span className="fleet-title">Fleet ({robots.length})</span>
        <span className="fleet-conn-dot" style={{ background: connected ? 'var(--normal)' : 'var(--red)' }} />
      </div>
      <div className="fleet-list">
        {robots.map(robot => (
          <div
            key={robot.id}
            className={`fleet-robot-row ${selectedRobot === robot.id ? 'fleet-robot-selected' : ''}`}
            onClick={() => onSelectRobot(selectedRobot === robot.id ? null : robot.id)}
          >
            <span
              className="fleet-robot-dot"
              style={{ background: STATE_COLORS[robot.connectionState] || 'var(--dim)' }}
            />
            <div className="fleet-robot-info">
              <span className="fleet-robot-name">{robot.id.split('/').pop()}</span>
              <span className="fleet-robot-detail">
                {robot.driving ? 'Driving' : robot.operatingMode}
                {robot.errorCount > 0 && <span className="fleet-error-badge">{robot.errorCount}</span>}
              </span>
            </div>
            {robot.batteryCharge !== undefined && robot.batteryCharge >= 0 && (
              <span className="fleet-robot-battery">{Math.round(robot.batteryCharge)}%</span>
            )}
          </div>
        ))}
      </div>
    </div>
  )
}
