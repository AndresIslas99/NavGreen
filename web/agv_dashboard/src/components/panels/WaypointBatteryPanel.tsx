import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import type { RobotStatus } from '../../api/types'

// 20-waypoint validation battery — 5 aisles × 4 stages (front_entry, front_mid,
// rear_entry, rear_mid). Operator dispatches each goal manually; the panel
// captures err_xy as soon as the nav action transitions from active to a
// terminal state. CSV export at the end of the session.

interface WpTarget {
  id: string
  aisle_y: number
  stage: 'front_entry' | 'front_mid' | 'rear_entry' | 'rear_mid'
  x: number
  y: number
  theta: number
}

const AISLES = [-4.4, -2.2, 0, 2.2, 4.4] as const

// Rough approximations aligned with world geometry — GAP 3.5..7.5, front = x>7.5,
// rear = x<3.5. Target points are at 0.5m and 2.5m into each rail from the gap.
const WP_TEMPLATE: Array<Omit<WpTarget, 'id' | 'aisle_y'>> = [
  { stage: 'front_entry', x: 8.0, y: 0, theta: 0 },
  { stage: 'front_mid',   x: 10.0, y: 0, theta: 0 },
  { stage: 'rear_entry',  x: 3.0, y: 0, theta: Math.PI },
  { stage: 'rear_mid',    x: 1.0, y: 0, theta: Math.PI },
]

function buildBattery(): WpTarget[] {
  const out: WpTarget[] = []
  for (const y of AISLES) {
    for (const tpl of WP_TEMPLATE) {
      out.push({
        id: `${tpl.stage}_y${y.toFixed(1)}`,
        aisle_y: y,
        stage: tpl.stage,
        x: tpl.x,
        y,
        theta: tpl.theta,
      })
    }
  }
  return out
}

type RunStatus = 'pending' | 'running' | 'done' | 'aborted'

interface WpResult {
  status: RunStatus
  started_at: number | null
  ended_at: number | null
  err_xy: number | null
  final_pose: { x: number; y: number; theta: number } | null
  nav_outcome: string | null
}

interface Props {
  status: RobotStatus | null
  onSendGoal: (x: number, y: number, theta: number) => void
  onCancel: () => void
}

function pad(n: number, w = 2): string {
  return String(n).padStart(w, '0')
}

function csvEscape(v: string | number | null): string {
  if (v === null) return ''
  const s = String(v)
  return /[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s
}

export function WaypointBatteryPanel({ status, onSendGoal, onCancel }: Props) {
  const battery = useMemo(() => buildBattery(), [])
  const [results, setResults] = useState<Record<string, WpResult>>(() => {
    const r: Record<string, WpResult> = {}
    for (const wp of battery) {
      r[wp.id] = {
        status: 'pending', started_at: null, ended_at: null,
        err_xy: null, final_pose: null, nav_outcome: null,
      }
    }
    return r
  })

  // Track the currently running waypoint id so status transitions can be matched
  const runningRef = useRef<string | null>(null)
  const prevNavActiveRef = useRef(false)

  // Watch for nav_state transitions from active → inactive to capture result
  useEffect(() => {
    const active = status?.nav_state?.active ?? false
    const prev = prevNavActiveRef.current
    prevNavActiveRef.current = active

    if (!prev || active) return  // only react to the falling edge
    const runId = runningRef.current
    if (!runId) return

    const wp = battery.find(b => b.id === runId)
    if (!wp || !status?.pose) {
      runningRef.current = null
      return
    }
    const dx = status.pose.x - wp.x
    const dy = status.pose.y - wp.y
    const err = Math.sqrt(dx * dx + dy * dy)
    const outcome = status.nav_state?.status ?? 'unknown'
    const done: RunStatus = outcome === 'succeeded' ? 'done' : 'aborted'

    setResults(prevR => ({
      ...prevR,
      [runId]: {
        ...prevR[runId],
        status: done,
        ended_at: Date.now() / 1000,
        err_xy: err,
        final_pose: { ...status.pose },
        nav_outcome: outcome,
      },
    }))
    runningRef.current = null
  }, [status, battery])

  const handleSend = useCallback((wp: WpTarget) => {
    runningRef.current = wp.id
    setResults(prevR => ({
      ...prevR,
      [wp.id]: {
        ...prevR[wp.id],
        status: 'running',
        started_at: Date.now() / 1000,
        ended_at: null,
        err_xy: null,
        final_pose: null,
        nav_outcome: null,
      },
    }))
    onSendGoal(wp.x, wp.y, wp.theta)
  }, [onSendGoal])

  const handleReset = useCallback(() => {
    runningRef.current = null
    setResults(() => {
      const r: Record<string, WpResult> = {}
      for (const wp of battery) {
        r[wp.id] = {
          status: 'pending', started_at: null, ended_at: null,
          err_xy: null, final_pose: null, nav_outcome: null,
        }
      }
      return r
    })
  }, [battery])

  const handleExportCsv = useCallback(() => {
    const header = 'id,aisle_y,stage,target_x,target_y,target_theta,status,err_xy,nav_outcome,duration_s,final_x,final_y,final_theta,started_at,ended_at'
    const lines = [header]
    for (const wp of battery) {
      const r = results[wp.id]
      const dur = r.started_at && r.ended_at ? (r.ended_at - r.started_at).toFixed(2) : ''
      lines.push([
        wp.id, wp.aisle_y, wp.stage, wp.x, wp.y, wp.theta.toFixed(3),
        r.status,
        r.err_xy != null ? r.err_xy.toFixed(4) : '',
        r.nav_outcome,
        dur,
        r.final_pose?.x.toFixed(3) ?? '',
        r.final_pose?.y.toFixed(3) ?? '',
        r.final_pose?.theta.toFixed(3) ?? '',
        r.started_at ?? '',
        r.ended_at ?? '',
      ].map(csvEscape).join(','))
    }
    const blob = new Blob([lines.join('\n')], { type: 'text/csv' })
    const d = new Date()
    const stamp = `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}`
    const a = document.createElement('a')
    a.href = URL.createObjectURL(blob)
    a.download = `waypoint_battery_${stamp}.csv`
    a.click()
    URL.revokeObjectURL(a.href)
  }, [battery, results])

  const doneCount = battery.filter(wp => results[wp.id].status === 'done').length
  const abortedCount = battery.filter(wp => results[wp.id].status === 'aborted').length
  const doneErrors = battery
    .filter(wp => results[wp.id].status === 'done' && results[wp.id].err_xy != null)
    .map(wp => results[wp.id].err_xy as number)
  const avgErr = doneErrors.length ? (doneErrors.reduce((a, b) => a + b, 0) / doneErrors.length) : null
  const maxErr = doneErrors.length ? Math.max(...doneErrors) : null
  const p95Err = doneErrors.length
    ? doneErrors.slice().sort((a, b) => a - b)[Math.floor(doneErrors.length * 0.95)] ?? null
    : null

  return (
    <div className="context-panel">
      <div className="panel-section">
        <div className="section-title">Batería de waypoints</div>
        <div style={{ fontSize: '12px', color: 'var(--text-secondary)', marginBottom: '6px' }}>
          5 pasillos × 4 etapas = 20 validaciones. Envía nav_goal; el backend
          dispara rail_approach automáticamente cuando el destino mapea a una
          etiqueta rail_start.
        </div>
        <div className="btn-row">
          <button className="full-width" onClick={handleReset}>Resetear</button>
          <button className="full-width" onClick={handleExportCsv}>Exportar CSV</button>
          <button className="full-width" onClick={onCancel}>Cancelar nav</button>
        </div>
      </div>

      <div className="panel-section">
        <div className="section-title">
          Progreso — {doneCount}/{battery.length} completados
          {abortedCount > 0 ? ` · ${abortedCount} abortados` : ''}
        </div>
        {doneErrors.length > 0 && (
          <div style={{ fontSize: '12px', color: 'var(--text-secondary)' }}>
            err_xy — prom {avgErr!.toFixed(3)} m · p95 {p95Err!.toFixed(3)} m · máx {maxErr!.toFixed(3)} m
          </div>
        )}
      </div>

      <div className="panel-section">
        <div className="section-title">Batería</div>
        <div style={{ maxHeight: '50vh', overflowY: 'auto', fontSize: '12px' }}>
          <table style={{ width: '100%', borderCollapse: 'collapse' }}>
            <thead>
              <tr style={{ textAlign: 'left', color: 'var(--text-secondary)' }}>
                <th>etapa</th>
                <th>y</th>
                <th>err</th>
                <th>estado</th>
                <th></th>
              </tr>
            </thead>
            <tbody>
              {battery.map(wp => {
                const r = results[wp.id]
                const statusColor =
                  r.status === 'done' ? 'var(--accent)' :
                  r.status === 'aborted' ? 'var(--crit)' :
                  r.status === 'running' ? 'var(--info)' : 'var(--text-secondary)'
                const statusLabel =
                  r.status === 'done' ? 'completado' :
                  r.status === 'aborted' ? 'abortado' :
                  r.status === 'running' ? 'en curso' : 'pendiente'
                return (
                  <tr key={wp.id} style={{ borderTop: '1px solid var(--border)' }}>
                    <td>{wp.stage}</td>
                    <td>{wp.aisle_y.toFixed(1)}</td>
                    <td>{r.err_xy != null ? r.err_xy.toFixed(3) : '—'}</td>
                    <td style={{ color: statusColor }}>{statusLabel}</td>
                    <td>
                      <button
                        className="small-btn"
                        disabled={r.status === 'running' || runningRef.current !== null}
                        onClick={() => handleSend(wp)}
                      >
                        Enviar
                      </button>
                    </td>
                  </tr>
                )
              })}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  )
}
