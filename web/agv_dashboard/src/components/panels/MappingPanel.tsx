import { useState, useEffect } from 'react'
import type { MapInfo, RobotState, AllowedActions } from '../../api/types'
import { Joystick } from '../Joystick'
import * as api from '../../api/client'

interface Props {
  state: RobotState
  actions: AllowedActions
  motorsArmed: boolean
  onModeChange: (mode: string) => void
  onRecording: (action: 'start' | 'stop') => void
  onCmdVel: (linear: number, angular: number) => void
}

export function MappingPanel({ state, actions, motorsArmed, onModeChange, onRecording, onCmdVel }: Props) {
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [saveName, setSaveName] = useState('')
  const [selectedMap, setSelectedMap] = useState('')
  const [busy, setBusy] = useState(false)
  const [msg, setMsg] = useState('')

  const refresh = () => api.listMaps().then(setMaps).catch(() => {})
  useEffect(() => { refresh() }, [])

  const handleStartMapping = () => {
    onModeChange('mapping')
    onRecording('start')
  }

  const handleStopMapping = () => {
    onRecording('stop')
    onModeChange('teleop')
  }

  const handleSave = async () => {
    if (!saveName.trim()) return
    setBusy(true)
    setMsg('')
    const r = await api.saveMap(saveName.trim()) as { success?: boolean }
    setBusy(false)
    setMsg(r.success ? 'Saved' : 'Failed')
    if (r.success) { setSaveName(''); refresh() }
    setTimeout(() => setMsg(''), 3000)
  }

  const handleLoad = async () => {
    if (!selectedMap) return
    setBusy(true)
    setMsg('')
    const r = await api.loadMap(selectedMap) as { success?: boolean }
    setBusy(false)
    setMsg(r.success ? 'Loaded' : 'Failed')
    setTimeout(() => setMsg(''), 3000)
  }

  return (
    <div className="context-panel">
      <div className="panel-section">
        <div className="section-title">Mapping Control</div>
        {state !== 'mapping' ? (
          <button
            className="full-width action-btn"
            disabled={!actions.canStartMapping}
            onClick={handleStartMapping}
          >
            Start Mapping
          </button>
        ) : (
          <button className="full-width stop-btn" onClick={handleStopMapping}>
            Stop Mapping
          </button>
        )}
        {state === 'mapping' && <div className="recording-indicator">Mapping active</div>}
        {state === 'mapping' && (
          <button className="full-width secondary" style={{ marginTop: 6 }}
            onClick={() => fetch('/api/acc_map', { method: 'DELETE' })}>
            Clear Scan Map
          </button>
        )}
      </div>

      {/* Joystick — always available in mapping mode */}
      <div className="panel-section">
        <div className="section-title">Drive</div>
        <Joystick
          enabled={motorsArmed && (state === 'mapping' || state === 'ready')}
          maxLinear={0.5}
          maxAngular={1.0}
          onMove={onCmdVel}
        />
        {!motorsArmed && <p className="dim">Arm motors first (Recovery panel)</p>}
      </div>

      <div className="panel-section">
        <div className="section-title">Save Map</div>
        <div className="input-row">
          <input
            type="text"
            placeholder="Map name"
            value={saveName}
            onChange={e => setSaveName(e.target.value)}
          />
          <button onClick={handleSave} disabled={!saveName.trim() || busy || !actions.canSaveMap}>
            Save
          </button>
        </div>
      </div>

      <div className="panel-section">
        <div className="section-title">Load Map</div>
        <div className="input-row">
          <select value={selectedMap} onChange={e => setSelectedMap(e.target.value)}>
            <option value="">Select map...</option>
            {maps.map(m => <option key={m.name} value={m.name}>{m.name}</option>)}
          </select>
          <button onClick={handleLoad} disabled={!selectedMap || busy || !actions.canLoadMap}>Load</button>
          <button onClick={refresh} title="Refresh" className="icon-btn">↻</button>
        </div>
        {msg && <span className="toolbar-msg">{msg}</span>}
      </div>
    </div>
  )
}
