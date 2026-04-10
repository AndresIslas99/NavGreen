import { useEffect, useState, useCallback } from 'react'
import type { DefinedTag, AprilTagState } from '../../api/types'

export function AprilTagsPanel() {
  const [state, setState] = useState<AprilTagState | null>(null)
  const [showAddForm, setShowAddForm] = useState(false)
  const [editingId, setEditingId] = useState<number | null>(null)
  const [form, setForm] = useState({ label: '', description: '', x: 0, y: 0, yaw: 0 })

  const fetchState = useCallback(async () => {
    try {
      const r = await fetch('/api/apriltags')
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
    setForm({ label: '', description: '', x: 0, y: 0, yaw: 0 })
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
        await fetch(`/api/apriltags/defined/${editingId}`, {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        })
      } else {
        await fetch('/api/apriltags/defined', {
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
      x: tag.x,
      y: tag.y,
      yaw: (tag.yaw * 180) / Math.PI,  // radians → degrees for display
    })
    setShowAddForm(true)
  }

  const handleDelete = async (id: number) => {
    if (!confirm('Delete this tag? Hardware assignment (if any) will also be removed.')) return
    await fetch(`/api/apriltags/defined/${id}`, { method: 'DELETE' })
    fetchState()
  }

  const handleUnassign = async (hardware_id: number) => {
    await fetch(`/api/apriltags/assignment/${hardware_id}`, { method: 'DELETE' })
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
          return (
            <div key={tag.id} className="apriltag-item">
              <div className="apriltag-header">
                <strong>#{tag.id} {tag.label}</strong>
                <div className="apriltag-actions">
                  <button className="btn-small" onClick={() => handleEdit(tag)}>Edit</button>
                  <button className="btn-small btn-danger" onClick={() => handleDelete(tag.id)}>Delete</button>
                </div>
              </div>
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
