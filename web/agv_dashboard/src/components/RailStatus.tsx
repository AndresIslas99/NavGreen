import type { RobotStatus } from '../api/types'

type Rail = RobotStatus['rail_state']

const FRESH_S = 3.0  // a state is "fresh" if the publisher has a tick within this many seconds

function nowS(): number {
  return Date.now() / 1000
}

function isStale(updated: number | undefined): boolean {
  if (!updated || updated <= 0) return true
  return (nowS() - updated) > FRESH_S
}

// ── Arbiter FSM pill ────────────────────────────────────────────────
// Blue border when actively routing (approach/drive/exit). Amber on
// blocked_handoff. Gray otherwise. Dim when stale / publisher offline.
const ARBITER_LABELS: Record<string, string> = {
  corridor_nav: 'CORRIDOR',
  rail_approach_pend: 'APPROACH·P',
  rail_approach: 'APPROACH',
  rail_drive: 'RAIL·DRV',
  rail_exit: 'RAIL·EXIT',
  blocked_handoff: '⚠ HANDOFF',
  teleop: 'TELEOP',
  idle: 'IDLE',
  unknown: '…',
}

function arbiterLabel(mode: string): string {
  return ARBITER_LABELS[mode] ?? mode.toUpperCase()
}

function arbiterStyle(ma: Rail['mode_arbiter']): React.CSSProperties {
  if (isStale(ma.updated)) return { background: 'var(--normal-bg)', color: 'var(--dim)', opacity: 0.5 }
  switch (ma.mode) {
    case 'blocked_handoff':
      return { background: 'var(--orange)', color: '#000', fontWeight: 700 }
    case 'rail_approach':
    case 'rail_approach_pend':
    case 'rail_drive':
    case 'rail_exit':
      return { background: 'transparent', color: 'var(--blue)', border: '1px solid var(--blue)' }
    case 'corridor_nav':
    case 'teleop':
    case 'idle':
      return { background: 'var(--normal-bg)', color: 'var(--dim)' }
    default:
      return { background: 'var(--normal-bg)', color: 'var(--dim)' }
  }
}

function arbiterTooltip(ma: Rail['mode_arbiter']): string {
  if (isStale(ma.updated)) return 'Mode arbiter: offline (no recent publish)'
  const age = Math.round((nowS() - ma.updated) * 10) / 10
  return `Arbiter: ${ma.mode} · src=${ma.source} · zone=${ma.zone} · op=${ma.operator_mode} · tx=${ma.transitions} · ${age}s ago`
}

// ── Zone classifier pill ────────────────────────────────────────────
// Highlights rail_aisle_* (inside a rail, active Rail-P3 gates) and
// rail_approach_* (lined up with a tag). Gap is normal. Unknown is dim.
function zoneLabel(z: Rail['zone']): string {
  if (!z.zone || z.zone === 'unknown') return '…'
  if (z.zone.startsWith('rail_aisle')) return `AISLE ${z.section}`
  if (z.zone.startsWith('rail_approach')) return `APPR·${z.section}${z.approach_tag_id >= 0 ? ` #${z.approach_tag_id}` : ''}`
  if (z.zone === 'gap') return 'GAP'
  return z.zone.toUpperCase()
}

function zoneStyle(z: Rail['zone']): React.CSSProperties {
  if (isStale(z.updated)) return { background: 'var(--normal-bg)', color: 'var(--dim)', opacity: 0.5 }
  if (z.zone.startsWith('rail_aisle') || z.zone.startsWith('rail_approach')) {
    return { background: 'transparent', color: 'var(--blue)', border: '1px solid var(--blue)' }
  }
  return { background: 'var(--normal-bg)', color: 'var(--dim)' }
}

function zoneTooltip(z: Rail['zone']): string {
  if (isStale(z.updated)) return 'Zone detector: offline'
  const lat = z.rail_offset_lat != null ? `${z.rail_offset_lat.toFixed(2)}m` : 'n/a'
  const yaw = z.rail_yaw_error != null ? `${(z.rail_yaw_error * 180 / Math.PI).toFixed(1)}°` : 'n/a'
  return `Zone: ${z.zone}/${z.section} · lat_off=${lat} · yaw_err=${yaw} · conf=${z.confidence.toFixed(2)}`
}

// ── Rail driver pill ────────────────────────────────────────────────
// Driving = blue. Reached = gray/neutral (just finished). blocked_* = amber.
// canceled = dim. idle = dim. Red never — rail_driver doesn't have a
// critical/fault state; collision_stop rides the SAFETY pill.
const DRV_LABELS: Record<string, string> = {
  idle: 'IDLE',
  driving: 'DRIVING',
  reached: 'REACHED',
  blocked_wait: '⚠ WAIT',
  blocked_misaligned: '⚠ MISALIGN',
  blocked_lateral: '⚠ LAT',
  canceled: 'CANCELED',
}

function driverLabel(d: Rail['rail_driver']): string {
  return DRV_LABELS[d.state] ?? d.state.toUpperCase()
}

function driverStyle(d: Rail['rail_driver']): React.CSSProperties {
  if (isStale(d.updated)) return { background: 'var(--normal-bg)', color: 'var(--dim)', opacity: 0.5 }
  switch (d.state) {
    case 'driving':
      return { background: 'transparent', color: 'var(--blue)', border: '1px solid var(--blue)' }
    case 'blocked_wait':
    case 'blocked_misaligned':
    case 'blocked_lateral':
      return { background: 'var(--orange)', color: '#000', fontWeight: 700 }
    default:
      return { background: 'var(--normal-bg)', color: 'var(--dim)' }
  }
}

function driverTooltip(d: Rail['rail_driver']): string {
  if (isStale(d.updated)) return 'Rail driver: offline'
  const colStr = d.collision_stop ? ' · COLLISION' : ''
  return `Rail driver: ${d.state} · v=${d.linear_x.toFixed(2)}m/s · remain=${d.remaining_m.toFixed(2)}m · in_rail=${d.in_rail_zone}${colStr}`
}

interface Props {
  rail: Rail | undefined
}

export function RailStatus({ rail }: Props) {
  if (!rail) return null
  const ma = rail.mode_arbiter
  const z = rail.zone
  const d = rail.rail_driver
  return (
    <>
      <span className="metric-sep">|</span>
      <span className="metric" title={arbiterTooltip(ma)}>
        <span className="metric-label">ARB</span>
        <span className="metric-value safety-badge" style={arbiterStyle(ma)}>
          {arbiterLabel(ma.mode)}
        </span>
      </span>
      <span className="metric-sep">|</span>
      <span className="metric" title={zoneTooltip(z)}>
        <span className="metric-label">ZONE</span>
        <span className="metric-value safety-badge" style={zoneStyle(z)}>
          {zoneLabel(z)}
        </span>
      </span>
      <span className="metric-sep">|</span>
      <span className="metric" title={driverTooltip(d)}>
        <span className="metric-label">RAIL</span>
        <span className="metric-value safety-badge" style={driverStyle(d)}>
          {driverLabel(d)}
        </span>
      </span>
    </>
  )
}
