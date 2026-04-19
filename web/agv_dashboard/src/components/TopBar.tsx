import type { RobotStatus, RobotState } from '../api/types'

interface Props {
  status: RobotStatus | null
  state: RobotState
  connected: boolean
  onEStop: (active: boolean) => void
  onNavCancel: () => void
  username?: string
  userRole?: string
  onLogout?: () => void
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

function slamColor(tracking: string): string {
  if (tracking === 'good') return 'ok'
  if (tracking === 'low' || tracking === 'medium') return 'warn'
  if (tracking === 'unknown') return 'unknown'
  return 'warn'
}

// collision_monitor pill — gray when idle/normal, color when something is wrong
// IDLE: no nav in progress, chain is alive but hasn't published state yet
// OK: chain published DO_NOTHING during active nav
// SLOWDOWN: amber (obstacle in slowdown_zone)
// STOP/STALE/OFFLINE: red (obstacle in stop_zone OR chain actually dead)
type Cm = RobotStatus['collision_monitor']
function safetyLabel(cm: Cm | undefined): string {
  if (!cm) return '…'
  if (cm.action === 'IDLE') return 'IDLE'
  if (cm.action === 'OFFLINE') return '⛔ OFFLINE'
  if (cm.action === 'STALE') return '⚠ STALE'
  if (cm.action === 'STOP') return '⛔ STOP'
  if (cm.action === 'SLOWDOWN') return '⚠ SLOW'
  if (cm.action === 'OK') return 'OK'
  return cm.action
}
function safetyBadgeStyle(cm: Cm | undefined): React.CSSProperties {
  if (!cm) return { background: 'var(--normal-bg)', color: 'var(--dim)' }
  switch (cm.action) {
    case 'OFFLINE':
    case 'STALE':
    case 'STOP':
      return { background: 'var(--red)', color: '#fff', fontWeight: 700 }
    case 'SLOWDOWN':
      return { background: 'var(--orange)', color: '#000', fontWeight: 700 }
    case 'IDLE':
    case 'OK':
      return { background: 'var(--normal-bg)', color: 'var(--dim)' }
    default:
      return { background: 'var(--normal-bg)', color: 'var(--dim)' }
  }
}
function collisionTooltip(cm: Cm | undefined): string {
  if (!cm) return 'Collision monitor: status unknown'
  const ageStr = cm.age_s != null ? `${cm.age_s}s ago` : 'never'
  const polyStr = cm.polygon ? `polygon: ${cm.polygon}` : ''
  return `Safety chain: ${cm.action}${polyStr ? ' · ' + polyStr : ''} · last update: ${ageStr}`
}

// Auto-localization orchestrator pill
type Loc = RobotStatus['localization']
function localizationLabel(loc: Loc | undefined): string {
  if (!loc || loc.action === 'UNKNOWN') return '…'
  if (loc.action === 'INITIALIZING') return '⟳ INIT'
  if (loc.action === 'LOCALIZED') return '✓ OK'
  if (loc.action === 'DEGRADED') return '⚠ DEG'
  if (loc.action === 'FAILED') return '⛔ FAIL'
  return loc.action
}
function localizationBadgeStyle(loc: Loc | undefined): React.CSSProperties {
  if (!loc) return { background: 'var(--normal-bg)', color: 'var(--dim)' }
  switch (loc.action) {
    case 'FAILED':
      return { background: 'var(--red)', color: '#fff', fontWeight: 700 }
    case 'DEGRADED':
      return { background: 'var(--orange)', color: '#000', fontWeight: 700 }
    case 'INITIALIZING':
      return { background: 'transparent', color: 'var(--blue)', border: '1px solid var(--blue)' }
    case 'LOCALIZED':
      return { background: 'var(--normal-bg)', color: 'var(--dim)' }
    default:
      return { background: 'var(--normal-bg)', color: 'var(--dim)' }
  }
}
function localizationTooltip(loc: Loc | undefined): string {
  if (!loc) return 'Localization: unknown'
  return `Localization: ${loc.action}${loc.map ? ' · map: ' + loc.map : ''}${loc.detail ? ' · ' + loc.detail : ''}`
}

export function TopBar({ status, state, connected, onEStop, onNavCancel, username, userRole, onLogout }: Props) {
  const s = status
  const navActive = s?.nav_state?.active || false
  const mp = s?.mission_progress

  // Connection quality
  const connColor = connected ? 'var(--normal)' : 'var(--red)'
  const connText = connected ? 'OK' : 'LOST'

  const mapName = s?.current_map_name ?? null

  return (
    <>
      <header className="top-bar">
        <span className="brand">AGV Control</span>

        <span
          className="map-header-pill"
          title={mapName ? `Active map: ${mapName}` : 'No map loaded — mapping-first mode'}
          style={{
            display: 'inline-flex',
            alignItems: 'center',
            gap: '6px',
            padding: '4px 10px',
            marginLeft: '8px',
            borderRadius: '4px',
            background: mapName ? 'var(--blue)' : 'var(--normal-bg)',
            color: mapName ? '#fff' : 'var(--dim)',
            fontWeight: 700,
            fontSize: '13px',
            letterSpacing: '0.5px',
            border: mapName ? 'none' : '1px dashed var(--dim)',
          }}
        >
          <span style={{ opacity: 0.7, fontWeight: 500, fontSize: '11px' }}>MAP</span>
          <span>{mapName ?? 'mapping…'}</span>
        </span>

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
              <span className="metric-label">SLAM</span>
              <span className={`metric-value slam-badge slam-${slamColor(s.slam_tracking)}`}>
                {s.slam_tracking || 'N/A'}
              </span>
            </span>
            <span className="metric-sep">|</span>
            <span className="metric">
              <span className="metric-label">BAT</span>
              <span className="metric-value" style={{
                color: s.battery_pct != null && s.battery_pct >= 0
                  ? s.battery_pct < 20 ? 'var(--red)' : s.battery_pct < 40 ? 'var(--orange)' : 'var(--dim)'
                  : 'var(--dim)'
              }}>
                {s.battery_pct != null && s.battery_pct >= 0 ? `${Math.round(s.battery_pct)}%` : 'N/A'}
              </span>
            </span>
            <span className="metric-sep">|</span>
            <span className="metric">
              <span className="metric-value">({s.pose.x.toFixed(1)}, {s.pose.y.toFixed(1)})</span>
            </span>
            <span className="metric-sep">|</span>
            <span className="metric" title={collisionTooltip(s.collision_monitor)}>
              <span className="metric-label">SAFETY</span>
              <span className="metric-value safety-badge" style={safetyBadgeStyle(s.collision_monitor)}>
                {safetyLabel(s.collision_monitor)}
              </span>
            </span>
            <span className="metric-sep">|</span>
            <span className="metric" title={localizationTooltip(s.localization)}>
              <span className="metric-label">LOC</span>
              <span className="metric-value safety-badge" style={localizationBadgeStyle(s.localization)}>
                {localizationLabel(s.localization)}
              </span>
            </span>
          </div>
        )}

        <div className="top-actions">
          {username && (
            <>
              <span className="user-badge">
                {username}<span className="user-role">{userRole}</span>
              </span>
              {onLogout && <button className="logout-btn" onClick={onLogout}>Logout</button>}
            </>
          )}
          <span className="conn-dot" style={{ color: connColor }}>
            <span className="conn-circle" style={{ background: connColor }} />
            {connText}
          </span>

          {navActive && (
            <button className="top-btn cancel-btn" onClick={onNavCancel}>Cancel</button>
          )}

          <button
            className={`estop-top ${s?.e_stop ? 'engaged' : ''}`}
            onClick={() => onEStop(!s?.e_stop)}
            aria-label={s?.e_stop ? 'Clear emergency stop' : 'Activate emergency stop'}
            title={s?.e_stop ? 'Click to clear E-Stop' : 'EMERGENCY STOP — click to halt'}
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
