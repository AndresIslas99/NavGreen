import type { RobotStatus, RobotState } from '../api/types'

interface Props {
  status: RobotStatus | null
  state: RobotState
  connected: boolean
  onEStop: (active: boolean) => void
  onNavCancel: () => void
}

// ISA-101: Gray for normal. Color only for abnormalities.
const STATE_LABELS: Record<RobotState, string> = {
  offline: 'OFFLINE',
  idle: 'IDLE',
  ready: 'READY',
  mapping: 'MAPPING',
  navigating: 'NAVIGATING',
  executing_mission: 'MISSION',
  blocked: 'BLOCKED',
  e_stop: 'E-STOP',
  fault: 'FAULT',
}

// ISA-101 color discipline:
// Normal states = no color (gray/dark bg)
// Active operations = subtle blue
// Warnings = amber/orange
// Critical = red
function stateBadgeStyle(state: RobotState): React.CSSProperties {
  switch (state) {
    case 'e_stop':
      return { background: 'var(--red)', color: '#fff' }
    case 'fault':
      return { background: 'var(--red)', color: '#fff' }
    case 'blocked':
      return { background: 'var(--orange)', color: '#000' }
    case 'navigating':
    case 'executing_mission':
    case 'mapping':
      return { background: 'transparent', color: 'var(--blue)', border: '1px solid var(--blue)' }
    case 'ready':
      return { background: 'var(--normal-bg)', color: 'var(--text)' }
    default:
      return { background: 'var(--normal-bg)', color: 'var(--dim)' }
  }
}

export function TopBar({ status, state, connected, onEStop, onNavCancel }: Props) {
  const s = status
  const navActive = s?.nav_state?.active || false
  const mp = s?.mission_progress

  // Connection quality
  const connColor = connected ? 'var(--normal)' : 'var(--red)'
  const connText = connected ? 'OK' : 'LOST'

  return (
    <>
      <header className="top-bar">
        <span className="brand">AGV Control</span>

        <span className="state-badge" style={stateBadgeStyle(state)}>
          {STATE_LABELS[state]}
        </span>

        {s && (
          <div className="top-metrics">
            <span className="metric">
              <span className="metric-value">{s.velocity.linear.toFixed(2)}</span>
              <span className="metric-unit">m/s</span>
            </span>
            <span className="metric-sep">|</span>
            <span className="metric">
              <span className="metric-value">{s.wheel_odom_hz}</span>
              <span className="metric-unit">Hz</span>
            </span>
            <span className="metric-sep">|</span>
            <span className="metric">
              <span className="metric-value">({s.pose.x.toFixed(1)}, {s.pose.y.toFixed(1)})</span>
            </span>
          </div>
        )}

        <div className="top-actions">
          <span className="conn-dot" style={{ color: connColor }}>
            <span className="conn-circle" style={{ background: connColor }} />
            {connText}
          </span>

          {navActive && (
            <button className="top-btn cancel-btn" onClick={onNavCancel}>Cancel</button>
          )}

          <button
            className="estop-top"
            onClick={() => onEStop(!s?.e_stop)}
          >
            {s?.e_stop ? 'CLEAR' : 'E-STOP'}
          </button>
        </div>
      </header>

      {/* Mission progress bar (OTTO-style at-a-glance) */}
      {mp && (mp.status === 'running' || mp.status === 'paused') && (
        <div className="mission-bar">
          <div className="mission-bar-fill"
            style={{ width: `${((mp.current_node + 1) / mp.total_nodes) * 100}%` }} />
          <span className="mission-bar-text">
            {mp.mission_name} — Node {mp.current_node + 1}/{mp.total_nodes}
            {s?.nav_state?.distance_remaining ? ` — ${s.nav_state.distance_remaining.toFixed(1)}m` : ''}
          </span>
        </div>
      )}
    </>
  )
}
