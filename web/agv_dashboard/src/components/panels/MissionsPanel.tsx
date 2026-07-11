import { useState, useEffect } from 'react'
import type { Mission, Waypoint, AllowedActions, MissionProgress, MissionNode, DefinedTag, WaypointAction } from '../../api/types'
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

// Plantillas de misión — patrón de recorrido sobre los waypoints capturados.
const TEMPLATES = [
  { label: 'Patrulla (A-B-C)', short: 'Patrulla', type: 'linear' },
  { label: 'Bucle (A-B-C-A)',  short: 'Bucle',    type: 'loop' },
  { label: 'Vaivén (A-B-A-B)', short: 'Vaivén',   type: 'shuttle' },
]

export function MissionsPanel({
  actions, navActive, missionProgress, pendingWaypoints, capturingWaypoints,
  onStartCapture, onClearWaypoints,
}: Props) {
  const [missions, setMissions] = useState<Mission[]>([])
  const [missionName, setMissionName] = useState('')
  const [editingNode, setEditingNode] = useState<number | null>(null)
  const [nodeAction, setNodeAction] = useState<WaypointAction>('none')
  const [nodePauseSec, setNodePauseSec] = useState(3)
  const [selectedTemplate, setSelectedTemplate] = useState('linear')
  const [expandedMission, setExpandedMission] = useState<string | null>(null)
  const [definedTags, setDefinedTags] = useState<DefinedTag[]>([])

  const refresh = () => api.listMissions().then(setMissions).catch(() => {})
  useEffect(() => { refresh() }, [])

  // Fetch defined AprilTags so waypoints can snap to them
  useEffect(() => {
    fetch(api.apiUrl('/api/apriltags'))
      .then(r => r.json())
      .then(s => setDefinedTags(s.defined_tags || []))
      .catch(() => {})
    const id = setInterval(() => {
      fetch(api.apiUrl('/api/apriltags')).then(r => r.json()).then(s => setDefinedTags(s.defined_tags || [])).catch(() => {})
    }, 5000)
    return () => clearInterval(id)
  }, [])

  // Find nearest defined tag within snap radius
  const findNearestTag = (x: number, y: number, radiusM = 0.5): DefinedTag | null => {
    let nearest: DefinedTag | null = null
    let nearestDist = Infinity
    for (const tag of definedTags) {
      const dx = tag.x - x
      const dy = tag.y - y
      const dist = Math.sqrt(dx * dx + dy * dy)
      if (dist < radiusM && dist < nearestDist) {
        nearest = tag
        nearestDist = dist
      }
    }
    return nearest
  }

  // Build nodes from waypoints with node actions
  const [nodeConfigs, setNodeConfigs] = useState<Map<number, { action: WaypointAction; pause_sec: number; apriltag_id?: number }>>(new Map())

  const handleSave = async () => {
    if (!missionName.trim() || pendingWaypoints.length === 0) return

    // Build nodes with configured actions; auto-snap to nearby AprilTag if no manual override
    const nodes: MissionNode[] = pendingWaypoints.map((wp, i) => {
      const config = nodeConfigs.get(i)
      const snapped = config?.apriltag_id !== undefined
        ? definedTags.find(t => t.id === config!.apriltag_id) || null
        : findNearestTag(wp.x, wp.y)
      return {
        id: `n${i}`,
        type: 'waypoint',
        // If snapped, use exact tag coordinates (operator click was approximate)
        x: snapped ? snapped.x : wp.x,
        y: snapped ? snapped.y : wp.y,
        theta: snapped ? snapped.yaw : (wp.theta || 0),
        action: (config?.action || 'none') as WaypointAction,
        pause_sec: config?.pause_sec || 3,
        ...(snapped ? { apriltag_id: snapped.id } : {}),
      }
    })

    // Apply template pattern
    let finalNodes = nodes
    if (selectedTemplate === 'loop' && nodes.length > 1) {
      finalNodes = [...nodes, { ...nodes[0], id: `n${nodes.length}` }]
    } else if (selectedTemplate === 'shuttle' && nodes.length >= 2) {
      const reversed = [...nodes].reverse().slice(1)
      finalNodes = [...nodes, ...reversed.map((n, i) => ({ ...n, id: `n${nodes.length + i}` }))]
    }

    await api.createMission({
      name: missionName.trim(),
      nodes: finalNodes,
      edges: [],
      waypoints: pendingWaypoints,
    })
    setMissionName('')
    setNodeConfigs(new Map())
    onClearWaypoints()
    refresh()
  }

  const [nodeApriltagId, setNodeApriltagId] = useState<number | null>(null)

  const handleEditNode = (idx: number) => {
    setEditingNode(idx)
    const config = nodeConfigs.get(idx)
    setNodeAction((config?.action || 'none') as WaypointAction)
    setNodePauseSec(config?.pause_sec || 3)
    // Default snap to nearest tag if not already configured
    if (config?.apriltag_id !== undefined) {
      setNodeApriltagId(config.apriltag_id)
    } else {
      const wp = pendingWaypoints[idx]
      const nearest = wp ? findNearestTag(wp.x, wp.y) : null
      setNodeApriltagId(nearest ? nearest.id : null)
    }
  }

  const handleSaveNodeEdit = () => {
    if (editingNode === null) return
    const newConfigs = new Map(nodeConfigs)
    newConfigs.set(editingNode, {
      action: nodeAction,
      pause_sec: nodePauseSec,
      ...(nodeApriltagId !== null ? { apriltag_id: nodeApriltagId } : {}),
    })
    setNodeConfigs(newConfigs)
    setEditingNode(null)
    setNodeApriltagId(null)
  }

  const [actionError, setActionError] = useState('')

  const handleExecute = async (m: Mission) => {
    setActionError('')
    try { await api.executeMission(m.id) }
    catch { setActionError('No se pudo iniciar la misión') }
  }

  const handleDelete = async (m: Mission) => {
    await api.deleteMission(m.id)
    refresh()
  }

  const handlePause = async () => {
    setActionError('')
    try { await api.pauseMission() }
    catch { setActionError('No se pudo pausar') }
  }

  const handleResume = async () => {
    setActionError('')
    try { await api.resumeMission() }
    catch { setActionError('No se pudo reanudar') }
  }

  const handleCancel = async () => {
    setActionError('')
    try { await api.cancelGoal() }
    catch { setActionError('No se pudo cancelar') }
  }

  const mp = missionProgress
  const isRunning = mp && mp.status === 'running'

  return (
    <div className="context-panel">
      {/* Mission progress (if executing) */}
      {mp && mp.status !== 'completed' && (
        <div className="panel-section">
          <div className="section-title">Misión activa</div>
          <div className="mission-active">
            <span className="mission-name">{mp.mission_name}</span>
            <span className="mission-meta">
              Nodo {mp.current_node + 1} / {mp.total_nodes} — {mp.status}
            </span>
            <div className="progress-bar">
              <div
                className="progress-fill"
                style={{ width: `${((mp.current_node + 1) / mp.total_nodes) * 100}%` }}
              />
            </div>
            <div className="btn-row">
              {isRunning && (
                <button className="small" onClick={handlePause}>Pausar</button>
              )}
              {mp.status === 'paused' && (
                <button className="small action-btn" onClick={handleResume}>Reanudar</button>
              )}
              {(isRunning || mp.status === 'paused') && (
                <button className="small stop-btn" onClick={handleCancel}>Cancelar</button>
              )}
              {actionError && <span className="mission-error">{actionError}</span>}
            </div>
          </div>
        </div>
      )}

      {/* Create mission */}
      <div className="panel-section">
        <div className="section-title">Crear misión</div>
        {!capturingWaypoints ? (
          <button className="full-width" onClick={onStartCapture} disabled={!actions.canSendGoal}>
            + Capturar waypoints
          </button>
        ) : (
          <>
            <input
              type="text"
              placeholder="Nombre de la misión"
              value={missionName}
              onChange={e => setMissionName(e.target.value)}
              className="full-width"
            />

            {/* Template selector */}
            <div className="template-row">
              {TEMPLATES.map(t => (
                <button
                  key={t.type}
                  className={`period-btn ${selectedTemplate === t.type ? 'period-btn-active' : ''}`}
                  onClick={() => setSelectedTemplate(t.type)}
                  title={t.label}
                >
                  {t.short}
                </button>
              ))}
            </div>

            {/* Waypoint list with edit */}
            <div className="waypoint-list">
              {pendingWaypoints.map((wp, i) => (
                <div key={i} className="waypoint-row">
                  <span className="wp-idx">{i + 1}</span>
                  <span className="wp-coord">({wp.x.toFixed(1)}, {wp.y.toFixed(1)})</span>
                  {nodeConfigs.get(i)?.action && nodeConfigs.get(i)?.action !== 'none' && (
                    <span className="wp-action-badge">
                      {nodeConfigs.get(i)?.action}
                    </span>
                  )}
                  <button className="wp-edit-btn" onClick={() => handleEditNode(i)}>...</button>
                </div>
              ))}
            </div>

            {/* Node edit dialog */}
            {editingNode !== null && (
              <div className="node-edit-dialog">
                <div className="section-title">Nodo {editingNode + 1}</div>

                {/* Snap to defined AprilTag */}
                <label className="form-field">
                  Fijar a AprilTag (opcional)
                  <select
                    value={nodeApriltagId ?? ''}
                    onChange={e => setNodeApriltagId(e.target.value === '' ? null : parseInt(e.target.value, 10))}
                    className="full-width"
                  >
                    <option value="">— Ninguno (usar coordenadas del waypoint) —</option>
                    {definedTags.map(t => (
                      <option key={t.id} value={t.id}>
                        #{t.id} {t.label} ({t.type === 'rail_start' ? '⏸ riel' : '🟦 pared'})
                      </option>
                    ))}
                  </select>
                </label>

                <label className="form-field">
                  Acción
                  <select
                    value={nodeAction}
                    onChange={e => setNodeAction(e.target.value as WaypointAction)}
                    className="full-width"
                  >
                    <option value="none">Ninguna</option>
                    <option value="pause">Pausa (esperar N segundos)</option>
                    <option value="signal">Señal</option>
                    <option value="start_recording">Iniciar grabación</option>
                    <option value="stop_recording">Detener grabación</option>
                  </select>
                </label>
                {nodeAction === 'pause' && (
                  <div className="pause-input">
                    <label>Duración (s):</label>
                    <input
                      type="number"
                      min={1}
                      max={300}
                      value={nodePauseSec}
                      onChange={e => setNodePauseSec(parseInt(e.target.value) || 3)}
                    />
                  </div>
                )}
                <div className="btn-row">
                  <button className="small action-btn" onClick={handleSaveNodeEdit}>Aplicar</button>
                  <button className="small secondary" onClick={() => setEditingNode(null)}>Cancelar</button>
                </div>
              </div>
            )}

            <p className="capture-status">{pendingWaypoints.length} waypoints — clic en el mapa para añadir</p>
            <div className="btn-row">
              <button onClick={handleSave} disabled={!missionName.trim() || pendingWaypoints.length === 0}>
                Guardar
              </button>
              <button onClick={onClearWaypoints} className="secondary">Cancelar</button>
            </div>
          </>
        )}
      </div>

      {/* Mission list */}
      <div className="panel-section">
        <div className="section-title">Misiones</div>
        {missions.length === 0 && <p className="dim">Sin misiones guardadas</p>}
        {missions.map(m => {
          const nodeCount = (m.nodes || m.waypoints || []).length
          const isExpanded = expandedMission === m.id
          return (
            <div key={m.id} className="mission-row">
              <div className="mission-info" onClick={() => setExpandedMission(isExpanded ? null : m.id)}>
                <span className="mission-name">{m.name}</span>
                <span className="mission-meta">{nodeCount} {nodeCount === 1 ? 'nodo' : 'nodos'}</span>
              </div>
              {isExpanded && m.nodes && (
                <div className="mission-detail">
                  {m.nodes.map((n, i) => (
                    <div key={n.id || i} className="mission-detail-node">
                      <span className="wp-idx">{i + 1}</span>
                      <span className="wp-coord">({n.x.toFixed(1)}, {n.y.toFixed(1)})</span>
                      {n.action !== 'none' && (
                        <span className="wp-action-badge">{n.action}{n.pause_sec ? ` ${n.pause_sec}s` : ''}</span>
                      )}
                    </div>
                  ))}
                </div>
              )}
              <div className="btn-row">
                <button
                  className="small action-btn"
                  onClick={() => handleExecute(m)}
                  disabled={!actions.canExecuteMission || navActive}
                >
                  Ejecutar
                </button>
                <button className="small secondary" onClick={() => handleDelete(m)}>Borrar</button>
              </div>
            </div>
          )
        })}
      </div>
    </div>
  )
}
