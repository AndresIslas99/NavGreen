/**
 * System Health Panel — Sub-fase 1.1.c.
 *
 * Modal triggered from the TopBar "Health" button. Polls
 * /api/health/components, /api/health/verifiers, /api/health/events
 * every 3 seconds while open. Grouped by `section` (Sensors,
 * Localization, Navigation, Services, Network).
 *
 * Designed to work even when the ROS bridge is offline — the systemd /
 * network / process checks keep returning data, so the operator always
 * has SOMETHING actionable on screen.
 */

import { useEffect, useState } from 'react'
import { apiUrl, getToken } from '../api/client'

type ComponentStatus = 'green' | 'amber' | 'red' | 'idle' | 'unknown'

interface ComponentSample {
  id: string
  name: string
  section: string
  critical: boolean
  status: ComponentStatus
  detail: string
  last_seen_ms_ago: number | null
}

interface VerifierDef {
  id: string
  name: string
  script: string
  blocking: boolean
}

interface HealthEvent {
  ts: number
  type: 'component_status' | 'verifier_run'
  id: string
  payload: Record<string, any>
}

interface VerifierResult {
  verifier: string
  exit_code: number
  duration_ms: number
  result: 'pass' | 'fail'
  stdout: string
  stderr: string
}

interface Props {
  open: boolean
  onClose: () => void
}

const STATUS_GLYPH: Record<ComponentStatus, string> = {
  green: '●',
  amber: '◐',
  red: '○',
  idle: '◯',
  unknown: '?',
}
const STATUS_COLOR: Record<ComponentStatus, string> = {
  green: 'var(--normal, #2ecc71)',
  amber: 'var(--orange, #f5a623)',
  red: 'var(--red, #ff453a)',
  idle: 'var(--dim, #888)',
  unknown: 'var(--dim, #888)',
}

async function authedJson<T>(url: string, init?: RequestInit): Promise<T> {
  const token = getToken()
  const r = await fetch(apiUrl(url), {
    ...init,
    headers: {
      'Content-Type': 'application/json',
      ...(token ? { Authorization: `Bearer ${token}` } : {}),
      ...(init?.headers || {}),
    },
  })
  if (!r.ok) {
    throw new Error(`HTTP ${r.status}: ${await r.text()}`)
  }
  return r.json()
}

export function HealthPanel({ open, onClose }: Props) {
  const [components, setComponents] = useState<ComponentSample[]>([])
  const [verifiers, setVerifiers] = useState<VerifierDef[]>([])
  const [events, setEvents] = useState<HealthEvent[]>([])
  const [verifierOut, setVerifierOut] = useState<VerifierResult | null>(null)
  const [runningVerifier, setRunningVerifier] = useState<string | null>(null)
  const [err, setErr] = useState<string | null>(null)

  useEffect(() => {
    if (!open) return
    let alive = true
    async function tick() {
      try {
        const [c, v, e] = await Promise.all([
          authedJson<{ components: ComponentSample[] }>('/api/health/components'),
          authedJson<{ verifiers: VerifierDef[] }>('/api/health/verifiers'),
          authedJson<{ events: HealthEvent[] }>('/api/health/events?lines=30'),
        ])
        if (!alive) return
        setComponents(c.components)
        setVerifiers(v.verifiers)
        setEvents(e.events)
        setErr(null)
      } catch (ex: any) {
        if (alive) setErr(String(ex?.message ?? ex))
      }
    }
    tick()
    const t = setInterval(tick, 3000)
    return () => { alive = false; clearInterval(t) }
  }, [open])

  if (!open) return null

  async function runVerifier(id: string) {
    setRunningVerifier(id)
    setVerifierOut(null)
    try {
      const r = await authedJson<VerifierResult>(`/api/health/verifiers/${id}/run`, { method: 'POST' })
      setVerifierOut(r)
    } catch (ex: any) {
      setVerifierOut({
        verifier: id, exit_code: -1, duration_ms: 0, result: 'fail',
        stdout: '', stderr: String(ex?.message ?? ex),
      })
    } finally {
      setRunningVerifier(null)
    }
  }

  // Group components by section
  const bySection = new Map<string, ComponentSample[]>()
  for (const c of components) {
    if (!bySection.has(c.section)) bySection.set(c.section, [])
    bySection.get(c.section)!.push(c)
  }
  const sectionOrder = ['Sensors', 'Localization', 'Navigation', 'Services', 'Network']
  const sections = [
    ...sectionOrder.filter(s => bySection.has(s)),
    ...[...bySection.keys()].filter(s => !sectionOrder.includes(s)),
  ]

  // Overall: red if any critical red, amber if any amber, else green
  const anyCriticalRed = components.some(c => c.critical && c.status === 'red')
  const anyAmber = components.some(c => c.status === 'amber')
  const overall: ComponentStatus = anyCriticalRed ? 'red' : anyAmber ? 'amber' : components.length === 0 ? 'unknown' : 'green'

  return (
    <div className="modal-overlay" onClick={onClose}>
      <div
        className="health-panel"
        onClick={e => e.stopPropagation()}
        style={{
          background: '#1a1a2e', border: '1px solid #2a2a3e', borderRadius: 8,
          padding: 20, width: '90%', maxWidth: 720, maxHeight: '85vh',
          overflowY: 'auto', color: 'var(--text, #e0e0e0)',
        }}
      >
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline' }}>
          <h3 style={{ margin: 0 }}>System Health</h3>
          <button onClick={onClose}>Close</button>
        </div>

        <div style={{ margin: '12px 0', fontSize: 14 }}>
          Overall:&nbsp;
          <span style={{ color: STATUS_COLOR[overall], fontSize: 18 }}>{STATUS_GLYPH[overall]}</span>
          &nbsp;<strong>{overall.toUpperCase()}</strong>
        </div>

        {err && <div style={{ color: 'var(--red, #ff453a)', marginBottom: 12 }}>backend error: {err}</div>}

        {sections.map(section => (
          <div key={section} style={{ marginBottom: 16 }}>
            <div style={{ fontSize: 13, fontWeight: 600, marginBottom: 6, opacity: 0.85 }}>{section}</div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
              {bySection.get(section)!.map(c => (
                <div key={c.id} style={{
                  display: 'flex', alignItems: 'baseline', gap: 8,
                  padding: '4px 0', fontSize: 13,
                }}>
                  <span style={{ color: STATUS_COLOR[c.status], fontSize: 16, width: 16, textAlign: 'center' }}>{STATUS_GLYPH[c.status]}</span>
                  <span style={{ width: 180 }}>{c.name}</span>
                  <span style={{ opacity: 0.7, flex: 1 }}>{c.detail}</span>
                  {c.critical && c.status === 'red' && (
                    <span style={{ color: 'var(--red, #ff453a)', fontSize: 11 }}>CRIT</span>
                  )}
                </div>
              ))}
            </div>
          </div>
        ))}

        <div style={{ marginTop: 20, marginBottom: 12 }}>
          <div style={{ fontSize: 13, fontWeight: 600, marginBottom: 6, opacity: 0.85 }}>Pre-flight Verifiers</div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
            {verifiers.map(v => (
              <div key={v.id} style={{ display: 'flex', alignItems: 'center', gap: 8, padding: '4px 0', fontSize: 13 }}>
                <button
                  onClick={() => runVerifier(v.id)}
                  disabled={runningVerifier !== null}
                  style={{ width: 64 }}
                >
                  {runningVerifier === v.id ? '...' : 'Run'}
                </button>
                <span style={{ width: 240 }}>{v.name}</span>
                <span style={{ opacity: 0.6, fontSize: 11 }}>{v.script}</span>
              </div>
            ))}
          </div>
          {verifierOut && (
            <div style={{
              marginTop: 12, padding: 8,
              background: '#0f0f1e', border: '1px solid #2a2a3e', borderRadius: 4,
              fontSize: 12, fontFamily: 'monospace',
            }}>
              <div style={{ marginBottom: 4 }}>
                <strong>{verifierOut.verifier}</strong>:&nbsp;
                <span style={{ color: verifierOut.result === 'pass' ? STATUS_COLOR.green : STATUS_COLOR.red }}>
                  {verifierOut.result.toUpperCase()}
                </span>
                &nbsp;(exit {verifierOut.exit_code}, {verifierOut.duration_ms}ms)
              </div>
              {verifierOut.stdout && <pre style={{ margin: 0, whiteSpace: 'pre-wrap', maxHeight: 200, overflowY: 'auto' }}>{verifierOut.stdout}</pre>}
              {verifierOut.stderr && <pre style={{ margin: 0, whiteSpace: 'pre-wrap', color: 'var(--red, #ff453a)', maxHeight: 100, overflowY: 'auto' }}>{verifierOut.stderr}</pre>}
            </div>
          )}
        </div>

        <div style={{ marginTop: 20 }}>
          <div style={{ fontSize: 13, fontWeight: 600, marginBottom: 6, opacity: 0.85 }}>Recent Events</div>
          {events.length === 0 ? (
            <div style={{ fontSize: 12, opacity: 0.5 }}>no events recorded yet</div>
          ) : (
            <div style={{
              fontFamily: 'monospace', fontSize: 11,
              background: '#0f0f1e', border: '1px solid #2a2a3e', borderRadius: 4,
              padding: 8, maxHeight: 160, overflowY: 'auto',
            }}>
              {events.map((e, i) => (
                <div key={i}>
                  {new Date(e.ts * 1000).toLocaleTimeString()} &nbsp;
                  <strong>{e.type}</strong>&nbsp;
                  <span style={{ opacity: 0.8 }}>{e.id}</span>&nbsp;
                  {JSON.stringify(e.payload)}
                </div>
              ))}
            </div>
          )}
        </div>
      </div>
    </div>
  )
}
