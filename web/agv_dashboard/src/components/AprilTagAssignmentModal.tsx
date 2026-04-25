import { useEffect, useState } from 'react'
import type { DefinedTag } from '../api/types'
import { apiUrl } from '../api/client'

interface Props {
  hardwareId: number
  onClose: () => void
}

export function AprilTagAssignmentModal({ hardwareId, onClose }: Props) {
  const [definedTags, setDefinedTags] = useState<DefinedTag[]>([])
  const [selectedId, setSelectedId] = useState<number | null>(null)
  const [submitting, setSubmitting] = useState(false)
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    fetch(apiUrl('/api/apriltags'))
      .then(r => r.json())
      .then(s => setDefinedTags(s.defined_tags || []))
      .catch(() => setError('Failed to load defined tags'))
  }, [])

  const handleAssign = async () => {
    if (selectedId === null) {
      setError('Select a defined tag')
      return
    }
    setSubmitting(true)
    try {
      const r = await fetch(apiUrl('/api/apriltags/assign'), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ hardware_id: hardwareId, defined_id: selectedId }),
      })
      if (r.ok) {
        onClose()
      } else {
        const data = await r.json().catch(() => ({}))
        setError(data.error || 'Assignment failed')
      }
    } catch (e: any) {
      setError(e?.message || 'Network error')
    } finally {
      setSubmitting(false)
    }
  }

  const handleDismiss = async () => {
    setSubmitting(true)
    try {
      await fetch(apiUrl(`/api/apriltags/dismiss/${hardwareId}`), { method: 'POST' })
      onClose()
    } catch { /* ignore */ } finally {
      setSubmitting(false)
    }
  }

  return (
    <div className="modal-overlay">
      <div className="modal-content">
        <h2>AprilTag Detected</h2>
        <p>The robot detected a hardware AprilTag with ID <strong>{hardwareId}</strong>.</p>

        {definedTags.length === 0 ? (
          <>
            <p className="error-text">
              No defined tags available. You need to define at least one tag in the AprilTags panel
              before you can assign detections.
            </p>
            <div className="modal-actions">
              <button onClick={handleDismiss} disabled={submitting}>Dismiss for now</button>
            </div>
          </>
        ) : (
          <>
            <p>Which of your defined tags is this?</p>
            <select
              value={selectedId ?? ''}
              onChange={e => setSelectedId(parseInt(e.target.value, 10) || null)}
              disabled={submitting}
              className="modal-select"
            >
              <option value="">— Select a defined tag —</option>
              {definedTags.map(t => (
                <option key={t.id} value={t.id}>
                  #{t.id} {t.label} ({t.x.toFixed(1)}, {t.y.toFixed(1)})
                </option>
              ))}
            </select>

            {error && <p className="error-text">{error}</p>}

            <div className="modal-actions">
              <button
                onClick={handleAssign}
                disabled={submitting || selectedId === null}
                className="btn-primary"
              >
                Assign
              </button>
              <button onClick={handleDismiss} disabled={submitting}>Dismiss for now</button>
            </div>
          </>
        )}
      </div>
    </div>
  )
}
