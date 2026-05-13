import { useEffect, useState, useCallback, useRef } from 'react'
import type { DefinedTag, AprilTagState, TagType } from '../../api/types'
import { apiUrl, getToken } from '../../api/client'

// Sub-fase 1.2 — operator-facing schema. Mirrors apriltag_manager.ts.
type LayoutRole = 'charging' | 'rail_entry' | 'central_aisle_beacon' | 'handoff' | 'other'
interface LayoutTagPreview {
  id: number
  role: LayoutRole
  rail_id?: string
  label?: string
  pose: { x: number; y: number; z: number; yaw_deg: number }
  size?: number
}
interface ValidationErr { index: number; id?: number; field: string; message: string }

interface ProbeStatus {
  localization_state: string
  localization_detail: string
  current_detection: {
    tag_id: number
    decision_margin: number
    range_m: number
    pose_in_map: { x: number; y: number; z: number; yaw_rad: number }
    last_seen_ms_ago: number | null
  } | null
}

function authedFetch(url: string, init?: RequestInit): Promise<Response> {
  const token = getToken()
  return fetch(apiUrl(url), {
    ...init,
    headers: {
      ...(init?.headers || {}),
      ...(token ? { Authorization: `Bearer ${token}` } : {}),
    },
  })
}

export function AprilTagsPanel() {
  const [state, setState] = useState<AprilTagState | null>(null)
  const [showAddForm, setShowAddForm] = useState(false)
  const [editingId, setEditingId] = useState<number | null>(null)
  const [form, setForm] = useState({ label: '', description: '', type: 'wall' as TagType, x: 0, y: 0, yaw: 0 })

  // ── Sub-fase 1.2 Layout Loader state ──
  const [layoutPreview, setLayoutPreview] = useState<{ tags: LayoutTagPreview[]; errors: ValidationErr[]; yamlText: string } | null>(null)
  const [layoutBusy, setLayoutBusy] = useState(false)
  const [layoutMsg, setLayoutMsg] = useState<string | null>(null)
  const fileInputRef = useRef<HTMLInputElement>(null)

  // ── Sub-fase 1.2 Probe state ──
  const [probeOpen, setProbeOpen] = useState(false)
  const [probeStatus, setProbeStatus] = useState<ProbeStatus | null>(null)
  const [probeRole, setProbeRole] = useState<LayoutRole>('rail_entry')
  const [probeRailId, setProbeRailId] = useState('')
  const [probeBusy, setProbeBusy] = useState(false)
  const [probeMsg, setProbeMsg] = useState<string | null>(null)

  const fetchState = useCallback(async () => {
    try {
      const r = await fetch(apiUrl('/api/apriltags'))
      if (r.ok) setState(await r.json())
    } catch { /* ignore */ }
  }, [])

  // Poll every 2s for state updates (defined tags + assignments + pending)
  useEffect(() => {
    fetchState()
    const id = setInterval(fetchState, 2000)
    return () => clearInterval(id)
  }, [fetchState])

  const resetForm = () => {
    setForm({ label: '', description: '', type: 'wall', x: 0, y: 0, yaw: 0 })
    setShowAddForm(false)
    setEditingId(null)
  }

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    if (!form.label.trim()) return
    // Convert yaw from degrees (input) to radians (storage)
    const yawRad = (form.yaw * Math.PI) / 180
    const body = { ...form, yaw: yawRad }
    try {
      if (editingId !== null) {
        await fetch(apiUrl(`/api/apriltags/defined/${editingId}`), {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        })
      } else {
        await fetch(apiUrl('/api/apriltags/defined'), {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        })
      }
      resetForm()
      fetchState()
    } catch { /* ignore */ }
  }

  const handleEdit = (tag: DefinedTag) => {
    setEditingId(tag.id)
    setForm({
      label: tag.label,
      description: tag.description,
      type: tag.type,
      x: tag.x,
      y: tag.y,
      yaw: (tag.yaw * 180) / Math.PI,  // radians → degrees for display
    })
    setShowAddForm(true)
  }

  const handleNavigate = async (tag: DefinedTag) => {
    try {
      const r = await fetch(apiUrl(`/api/apriltags/${tag.id}/navigate`), { method: 'POST' })
      if (!r.ok) {
        const err = await r.json().catch(() => ({}))
        alert(`Failed to send goal: ${err.error || 'unknown'}`)
      }
    } catch (e: any) {
      alert(`Network error: ${e?.message}`)
    }
  }

  /**
   * Pure-alignment path: skips Nav2 entirely and goes directly to
   * fine-servoing with the AprilTag detection. Use when the tag is
   * already physically in front of the robot. Works with localization
   * in any state (DEGRADED, FAILED, no map loaded).
   */
  const handleAlign = async (tag: DefinedTag) => {
    const hwId = getHardwareForDefined(tag.id)
    if (hwId === null) {
      alert(`"${tag.label}" doesn't have a hardware ID assigned. ` +
            `Assign one from the Pending Detections panel first.`)
      return
    }
    try {
      const r = await fetch(apiUrl(`/api/apriltags/${hwId}/align`), { method: 'POST' })
      if (!r.ok) {
        const err = await r.json().catch(() => ({}))
        alert(`Align failed: ${err.error || 'unknown'}`)
      }
    } catch (e: any) {
      alert(`Network error: ${e?.message}`)
    }
  }

  const handleDelete = async (id: number) => {
    if (!confirm('Delete this tag? Hardware assignment (if any) will also be removed.')) return
    await fetch(apiUrl(`/api/apriltags/defined/${id}`), { method: 'DELETE' })
    fetchState()
  }

  const handleUnassign = async (hardware_id: number) => {
    await fetch(apiUrl(`/api/apriltags/assignment/${hardware_id}`), { method: 'DELETE' })
    fetchState()
  }

  const getHardwareForDefined = (definedId: number): number | null => {
    if (!state) return null
    for (const [hwId, defId] of Object.entries(state.hardware_assignments)) {
      if (defId === definedId) return parseInt(hwId, 10)
    }
    return null
  }

  // ── Sub-fase 1.2 Layout Loader handlers ──

  const handleLayoutFile = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0]
    if (!file) return
    const reader = new FileReader()
    reader.onload = async () => {
      const yamlText = String(reader.result ?? '')
      setLayoutBusy(true)
      setLayoutMsg(null)
      try {
        const r = await authedFetch('/api/tags/layout/validate', {
          method: 'POST',
          headers: { 'Content-Type': 'text/yaml' },
          body: yamlText,
        })
        const data = await r.json()
        if (r.ok && data.valid) {
          setLayoutPreview({ tags: data.tags, errors: [], yamlText })
        } else {
          setLayoutPreview({ tags: [], errors: data.errors || [], yamlText })
        }
      } catch (ex: any) {
        setLayoutMsg(`Network: ${ex?.message}`)
      } finally {
        setLayoutBusy(false)
      }
    }
    reader.readAsText(file)
  }

  const handleLayoutApply = async () => {
    if (!layoutPreview || layoutPreview.errors.length > 0) return
    if (!confirm(`Apply ${layoutPreview.tags.length} tags? This will REPLACE all current tags + assignments.`)) return
    setLayoutBusy(true)
    setLayoutMsg(null)
    try {
      const r = await authedFetch('/api/tags/layout/apply', {
        method: 'POST',
        headers: { 'Content-Type': 'text/yaml' },
        body: layoutPreview.yamlText,
      })
      const data = await r.json()
      if (r.ok && data.applied) {
        setLayoutMsg(`Applied ${data.tag_count} tags (replaced=${data.replaced}, runtime_reloaded=${data.runtime_reloaded})`)
        setLayoutPreview(null)
        if (fileInputRef.current) fileInputRef.current.value = ''
        fetchState()
      } else {
        setLayoutMsg(`Apply failed: ${data.error || JSON.stringify(data.errors)}`)
      }
    } catch (ex: any) {
      setLayoutMsg(`Network: ${ex?.message}`)
    } finally {
      setLayoutBusy(false)
    }
  }

  const handleLayoutDownloadCurrent = async () => {
    const r = await authedFetch('/api/tags/layout/current')
    if (!r.ok) { alert('Failed to fetch current layout'); return }
    const text = await r.text()
    const blob = new Blob([text], { type: 'text/yaml' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `tag_layout_${new Date().toISOString().slice(0, 10)}.yaml`
    a.click()
    URL.revokeObjectURL(url)
  }

  const handleLayoutDownloadExample = async () => {
    const r = await authedFetch('/api/tags/layout/example')
    if (!r.ok) { alert('Failed to fetch example layout'); return }
    const text = await r.text()
    const blob = new Blob([text], { type: 'text/yaml' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = 'sample_layout.yaml'
    a.click()
    URL.revokeObjectURL(url)
  }

  // ── Sub-fase 1.2 Probe handlers ──

  useEffect(() => {
    if (!probeOpen) return
    let alive = true
    async function tick() {
      try {
        const r = await authedFetch('/api/tags/probe/status')
        if (r.ok) {
          const data: ProbeStatus = await r.json()
          if (alive) setProbeStatus(data)
        }
      } catch { /* ignore */ }
    }
    tick()
    const t = setInterval(tick, 500)
    return () => { alive = false; clearInterval(t) }
  }, [probeOpen])

  const handleProbeSave = async () => {
    if (!probeStatus?.current_detection) return
    setProbeBusy(true)
    setProbeMsg(null)
    try {
      const r = await authedFetch('/api/tags/probe/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          tag_id: probeStatus.current_detection.tag_id,
          role: probeRole,
          rail_id: probeRole === 'rail_entry' ? probeRailId : undefined,
        }),
      })
      const data = await r.json()
      if (r.ok && (data.added || data.updated)) {
        setProbeMsg(`Tag ${probeStatus.current_detection.tag_id} ${data.added ? 'added' : 'updated'} (total: ${data.total_tags})`)
        fetchState()
      } else {
        setProbeMsg(`Save failed: ${data.error || 'unknown'}`)
      }
    } catch (ex: any) {
      setProbeMsg(`Network: ${ex?.message}`)
    } finally {
      setProbeBusy(false)
    }
  }

  const probeReady = probeStatus !== null
    && (probeStatus.localization_state === 'LOCALIZED' || probeStatus.localization_state === 'DEGRADED')
    && probeStatus.current_detection !== null
    && probeStatus.current_detection.last_seen_ms_ago !== null
    && probeStatus.current_detection.last_seen_ms_ago < 2000

  return (
    <div className="panel">
      <div className="panel-header">
        <h3>AprilTags</h3>
      </div>

      <div className="panel-section">
        <div className="section-title">
          Defined Tags
          {!showAddForm && (
            <button className="btn-small" onClick={() => setShowAddForm(true)}>+ Add</button>
          )}
        </div>

        {showAddForm && (
          <form onSubmit={handleSubmit} className="apriltag-form">
            <input
              type="text" placeholder="Label (e.g. Entrada principal)"
              value={form.label}
              onChange={e => setForm({ ...form, label: e.target.value })}
              required
            />
            <input
              type="text" placeholder="Description (optional)"
              value={form.description}
              onChange={e => setForm({ ...form, description: e.target.value })}
            />
            <label className="form-field">
              Type
              <select value={form.type}
                onChange={e => setForm({ ...form, type: e.target.value as TagType })}>
                <option value="wall">Wall (vertical — drift correction)</option>
                <option value="rail_start">Rail start (horizontal — precision approach)</option>
              </select>
            </label>
            <div className="form-row">
              <label>X (m)<input type="number" step="0.01" value={form.x}
                onChange={e => setForm({ ...form, x: parseFloat(e.target.value) || 0 })} /></label>
              <label>Y (m)<input type="number" step="0.01" value={form.y}
                onChange={e => setForm({ ...form, y: parseFloat(e.target.value) || 0 })} /></label>
              <label>Yaw (°)<input type="number" step="1" value={form.yaw}
                onChange={e => setForm({ ...form, yaw: parseFloat(e.target.value) || 0 })} /></label>
            </div>
            <div className="form-actions">
              <button type="submit" className="btn-primary">{editingId !== null ? 'Update' : 'Create'}</button>
              <button type="button" onClick={resetForm}>Cancel</button>
            </div>
          </form>
        )}

        {state && state.defined_tags.length === 0 && !showAddForm && (
          <p className="dim">No tags defined yet. Click "+ Add" to define your first tag.</p>
        )}

        {state && state.defined_tags.map(tag => {
          const hwId = getHardwareForDefined(tag.id)
          const typeIcon = tag.type === 'rail_start' ? '⏸' : '🟦'
          const typeLabel = tag.type === 'rail_start' ? 'Rail start' : 'Wall'
          return (
            <div key={tag.id} className={`apriltag-item apriltag-${tag.type}`}>
              <div className="apriltag-header">
                <strong>{typeIcon} #{tag.id} {tag.label}</strong>
                <div className="apriltag-actions">
                  <button className="btn-small" onClick={() => handleNavigate(tag)} title="Send AGV via Nav2 (requires map + LOCALIZED)">
                    Send AGV
                  </button>
                  <button
                    className="btn-small"
                    onClick={() => handleAlign(tag)}
                    title="Align with this tag using fine-servoing only — skips Nav2. Tag must be visible and a hardware ID assigned."
                    disabled={hwId === null}
                  >
                    Align
                  </button>
                  <button className="btn-small" onClick={() => handleEdit(tag)}>Edit</button>
                  <button className="btn-small btn-danger" onClick={() => handleDelete(tag.id)}>Delete</button>
                </div>
              </div>
              <div className="apriltag-type-badge">{typeLabel}</div>
              {tag.description && <div className="dim">{tag.description}</div>}
              <div className="apriltag-coords">
                ({tag.x.toFixed(2)}, {tag.y.toFixed(2)}) yaw={(tag.yaw * 180 / Math.PI).toFixed(0)}°
              </div>
              <div className="apriltag-hw">
                Hardware: {hwId !== null ? (
                  <>
                    <span className="badge">ID {hwId}</span>
                    <button className="btn-small" onClick={() => handleUnassign(hwId)}>Unassign</button>
                  </>
                ) : (
                  <span className="dim">not assigned</span>
                )}
              </div>
            </div>
          )
        })}
      </div>

      {/* ── Sub-fase 1.2 Layout Import / Export ── */}
      <div className="panel-section">
        <div className="section-title">Layout (YAML)</div>
        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', marginBottom: 8 }}>
          <input
            ref={fileInputRef}
            type="file"
            accept=".yaml,.yml"
            onChange={handleLayoutFile}
            style={{ flex: 1, minWidth: 200 }}
          />
          <button className="btn-small" onClick={handleLayoutDownloadCurrent}>Download current</button>
          <button className="btn-small" onClick={handleLayoutDownloadExample}>Download example</button>
        </div>

        {layoutBusy && <div className="dim">Working…</div>}
        {layoutMsg && <div style={{ fontSize: 12, opacity: 0.9 }}>{layoutMsg}</div>}

        {layoutPreview && layoutPreview.errors.length > 0 && (
          <div style={{ marginTop: 8, padding: 8, background: '#3a1a1a', border: '1px solid var(--red, #ff453a)', borderRadius: 4 }}>
            <div style={{ fontWeight: 600, marginBottom: 4 }}>Validation errors ({layoutPreview.errors.length}):</div>
            <ul style={{ margin: 0, paddingLeft: 18, fontSize: 12 }}>
              {layoutPreview.errors.slice(0, 10).map((e, i) => (
                <li key={i}>
                  {e.id !== undefined ? `tag id=${e.id}` : `tag #${e.index}`} — <code>{e.field}</code>: {e.message}
                </li>
              ))}
              {layoutPreview.errors.length > 10 && <li>… +{layoutPreview.errors.length - 10} more</li>}
            </ul>
          </div>
        )}

        {layoutPreview && layoutPreview.errors.length === 0 && layoutPreview.tags.length > 0 && (
          <>
            <div style={{ marginTop: 8, marginBottom: 6, fontSize: 12 }}>
              <strong>{layoutPreview.tags.length}</strong> tags ready to apply.
              <span style={{ marginLeft: 8, opacity: 0.7 }}>(Apply REPLACES all current tags.)</span>
            </div>
            <div style={{ maxHeight: 180, overflowY: 'auto', fontFamily: 'monospace', fontSize: 11, background: '#0f0f1e', border: '1px solid #2a2a3e', padding: 6 }}>
              <div style={{ opacity: 0.6, marginBottom: 4 }}>
                id   role                    x       y       z      yaw   rail_id
              </div>
              {layoutPreview.tags.map(t => (
                <div key={t.id}>
                  {String(t.id).padEnd(5)}{t.role.padEnd(24)}{t.pose.x.toFixed(2).padStart(7)}{t.pose.y.toFixed(2).padStart(8)}{t.pose.z.toFixed(2).padStart(7)}{t.pose.yaw_deg.toFixed(1).padStart(7)}°  {t.rail_id ?? ''}
                </div>
              ))}
            </div>
            <div style={{ display: 'flex', gap: 8, marginTop: 8 }}>
              <button className="btn-primary" disabled={layoutBusy} onClick={handleLayoutApply}>
                Apply layout
              </button>
              <button onClick={() => { setLayoutPreview(null); setLayoutMsg(null); if (fileInputRef.current) fileInputRef.current.value = '' }}>
                Cancel
              </button>
            </div>
          </>
        )}
      </div>

      {/* ── Sub-fase 1.2 Robot Probe ── */}
      <div className="panel-section">
        <div className="section-title">Robot Probe (in-situ)</div>
        <p className="dim" style={{ fontSize: 12, margin: '4px 0' }}>
          Drive the robot in front of a physical AprilTag while localized.
          The probe captures the tag's pose in map frame from the current
          marker_correction estimate and saves it to the layout.
        </p>
        <button className="btn-small" onClick={() => { setProbeOpen(true); setProbeMsg(null) }}>Open probe…</button>
        {probeOpen && (
          <div className="modal-overlay" onClick={() => setProbeOpen(false)}>
            <div className="modal-body" onClick={e => e.stopPropagation()} style={{ maxWidth: 480 }}>
              <h3 style={{ margin: '0 0 8px 0' }}>Probe Tag in-situ</h3>
              {!probeStatus && <div className="dim">Loading…</div>}
              {probeStatus && (
                <>
                  <div style={{ marginBottom: 6 }}>
                    Localization:&nbsp;
                    <strong style={{ color: probeStatus.localization_state === 'LOCALIZED' ? 'var(--normal, #2ecc71)' : probeStatus.localization_state === 'DEGRADED' ? 'var(--orange, #f5a623)' : 'var(--red, #ff453a)' }}>
                      {probeStatus.localization_state}
                    </strong>
                    {probeStatus.localization_detail && <span className="dim"> · {probeStatus.localization_detail}</span>}
                  </div>

                  {probeStatus.current_detection ? (
                    <div style={{ padding: 8, background: '#0f0f1e', border: '1px solid #2a2a3e', borderRadius: 4, marginBottom: 8 }}>
                      <div>Tag <strong>#{probeStatus.current_detection.tag_id}</strong>{' '}
                        <span className="dim">(last seen {probeStatus.current_detection.last_seen_ms_ago}ms ago)</span>
                      </div>
                      <div style={{ fontFamily: 'monospace', fontSize: 12, marginTop: 4 }}>
                        pose in map: x={probeStatus.current_detection.pose_in_map.x.toFixed(3)} m,
                        y={probeStatus.current_detection.pose_in_map.y.toFixed(3)} m,
                        z={probeStatus.current_detection.pose_in_map.z.toFixed(3)} m,
                        yaw={(probeStatus.current_detection.pose_in_map.yaw_rad * 180 / Math.PI).toFixed(1)}°
                      </div>
                    </div>
                  ) : (
                    <div className="dim" style={{ marginBottom: 8 }}>Waiting for tag detection… (point camera at a physical tag)</div>
                  )}

                  <div style={{ display: 'flex', gap: 8, alignItems: 'baseline', marginBottom: 8 }}>
                    <label style={{ width: 60 }}>Role:</label>
                    <select
                      value={probeRole}
                      onChange={e => setProbeRole(e.target.value as LayoutRole)}
                    >
                      <option value="charging">charging</option>
                      <option value="rail_entry">rail_entry</option>
                      <option value="central_aisle_beacon">central_aisle_beacon</option>
                      <option value="handoff">handoff</option>
                      <option value="other">other</option>
                    </select>
                  </div>
                  {probeRole === 'rail_entry' && (
                    <div style={{ display: 'flex', gap: 8, alignItems: 'baseline', marginBottom: 8 }}>
                      <label style={{ width: 60 }}>Rail ID:</label>
                      <input
                        type="text"
                        placeholder="e.g. rail_1_north"
                        value={probeRailId}
                        onChange={e => setProbeRailId(e.target.value)}
                        style={{ flex: 1 }}
                      />
                    </div>
                  )}

                  {probeMsg && <div style={{ fontSize: 12, marginBottom: 8 }}>{probeMsg}</div>}

                  <div style={{ display: 'flex', gap: 8, justifyContent: 'flex-end' }}>
                    <button onClick={() => setProbeOpen(false)}>Close</button>
                    <button
                      className="btn-primary"
                      disabled={!probeReady || probeBusy || (probeRole === 'rail_entry' && !probeRailId.trim())}
                      onClick={handleProbeSave}
                    >
                      {probeBusy ? '…' : 'Confirm & save'}
                    </button>
                  </div>
                </>
              )}
            </div>
          </div>
        )}
      </div>

      {state && state.pending_detections.length > 0 && (
        <div className="panel-section">
          <div className="section-title">Pending Detections</div>
          <p className="dim">Hardware AprilTags detected but not yet assigned to any defined tag.</p>
          {state.pending_detections.map(d => (
            <div key={d.hardware_id} className="apriltag-pending">
              Hardware ID {d.hardware_id} (seen {Math.round(Date.now() / 1000 - d.first_seen)}s ago)
            </div>
          ))}
          <p className="dim">A modal will appear automatically to assign each new detection.</p>
        </div>
      )}
    </div>
  )
}
