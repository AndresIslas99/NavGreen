import { useEffect, useState, useCallback } from 'react'
import type { DefinedTag, AprilTagState, TagType } from '../../api/types'
import { apiUrl } from '../../api/client'

export function AprilTagsPanel() {
  const [state, setState] = useState<AprilTagState | null>(null)
  const [showAddForm, setShowAddForm] = useState(false)
  const [editingId, setEditingId] = useState<number | null>(null)
  const [form, setForm] = useState({ label: '', description: '', type: 'wall' as TagType, x: 0, y: 0, yaw: 0 })

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
