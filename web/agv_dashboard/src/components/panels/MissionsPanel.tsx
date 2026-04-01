import { useState, useEffect } from 'react'
import type { Mission, Waypoint, AllowedActions, MissionProgress } from '../../api/types'
import * as api from '../../api/client'

interface Props {
  actions: AllowedActions
  navActive: boolean
  missionProgress: MissionProgress | null
  pendingWaypoints: Waypoint[]
  capturingWaypoints: boolean
  onStartCapture: () => void
  onClearWaypoints: () => void
}

export function MissionsPanel({
  actions, navActive, missionProgress, pendingWaypoints, capturingWaypoints,
  onStartCapture, onClearWaypoints,
}: Props) {
  const [missions, setMissions] = useState<Mission[]>([])
  const [missionName, setMissionName] = useState('')

  const refresh = () => api.listMissions().then(setMissions).catch(() => {})
  useEffect(() => { refresh() }, [])

  const handleSave = async () => {
    if (!missionName.trim() || pendingWaypoints.length === 0) return
    await api.createMission({
      name: missionName.trim(),
      waypoints: pendingWaypoints,
    })
    setMissionName('')
    onClearWaypoints()
    refresh()
  }

  const handleExecute = async (m: Mission) => {
    await api.executeMission(m.id)
  }

  const handleDelete = async (m: Mission) => {
    await api.deleteMission(m.id)
    refresh()
  }

  const handlePause = () => fetch('/api/missions/pause', { method: 'POST' })
  const handleResume = () => fetch('/api/missions/resume', { method: 'POST' })

  const mp = missionProgress
  const isRunning = mp && mp.status === 'running'

  return (
    <div className="context-panel">
      {/* Mission progress (if executing) */}
      {mp && mp.status !== 'completed' && (
        <div className="panel-section">
          <div className="section-title">Active Mission</div>
          <div className="mission-active">
            <span className="mission-name">{mp.mission_name}</span>
            <span className="mission-meta">
              Node {mp.current_node + 1} / {mp.total_nodes} — {mp.status}
            </span>
            <div className="progress-bar">
              <div
                className="progress-fill"
                style={{ width: `${((mp.current_node + 1) / mp.total_nodes) * 100}%` }}
              />
            </div>
            <div className="btn-row">
              {isRunning && (
                <button className="small" onClick={handlePause}>Pause</button>
              )}
              {mp.status === 'paused' && (
                <button className="small action-btn" onClick={handleResume}>Resume</button>
              )}
              {(isRunning || mp.status === 'paused') && (
                <button className="small stop-btn" onClick={() => fetch('/api/nav/cancel', { method: 'POST' })}>
                  Cancel
                </button>
              )}
            </div>
          </div>
        </div>
      )}

      {/* Create mission */}
      <div className="panel-section">
        <div className="section-title">Create Mission</div>
        {!capturingWaypoints ? (
          <button className="full-width" onClick={onStartCapture} disabled={!actions.canSendGoal}>
            + Capture Waypoints
          </button>
        ) : (
          <>
            <input
              type="text"
              placeholder="Mission name"
              value={missionName}
              onChange={e => setMissionName(e.target.value)}
              className="full-width"
            />
            <p className="capture-status">{pendingWaypoints.length} waypoints — click map to add</p>
            <div className="btn-row">
              <button onClick={handleSave} disabled={!missionName.trim() || pendingWaypoints.length === 0}>
                Save
              </button>
              <button onClick={onClearWaypoints} className="secondary">Cancel</button>
            </div>
          </>
        )}
      </div>

      {/* Mission list */}
      <div className="panel-section">
        <div className="section-title">Missions</div>
        {missions.length === 0 && <p className="dim">No missions saved</p>}
        {missions.map(m => (
          <div key={m.id} className="mission-row">
            <div className="mission-info">
              <span className="mission-name">{m.name}</span>
              <span className="mission-meta">
                {(m.nodes || m.waypoints || []).length} nodes
              </span>
            </div>
            <div className="btn-row">
              <button
                className="small action-btn"
                onClick={() => handleExecute(m)}
                disabled={!actions.canExecuteMission || navActive}
              >
                Run
              </button>
              <button className="small secondary" onClick={() => handleDelete(m)}>Del</button>
            </div>
          </div>
        ))}
      </div>
    </div>
  )
}
