/**
 * Traffic zone editor panel.
 * Connects to fleet manager (port 8091) to manage traffic zones.
 */

import { useState, useEffect, useCallback } from 'react'
import type { TrafficZone, ZoneOccupancy } from '../../api/types'
import * as api from '../../api/client'

interface Props {
  onDrawZone?: (enabled: boolean) => void
  drawnPolygon?: Array<{ x: number; y: number }> | null
  onClearDrawn?: () => void
}

const ZONE_COLORS: Record<string, string> = {
  exclusion: 'var(--red)',
  one_way: 'var(--orange)',
  yield: 'var(--blue)',
}

export function TrafficPanel({ onDrawZone, drawnPolygon, onClearDrawn }: Props) {
  const [zones, setZones] = useState<TrafficZone[]>([])
  const [occupancy, setOccupancy] = useState<ZoneOccupancy[]>([])
  const [newZoneName, setNewZoneName] = useState('')
  const [newZoneType, setNewZoneType] = useState<'exclusion' | 'one_way' | 'yield'>('exclusion')
  const [newMaxRobots, setNewMaxRobots] = useState(1)
  const [drawing, setDrawing] = useState(false)
  const [error, setError] = useState('')

  const refresh = useCallback(async () => {
    try {
      const [z, o] = await Promise.all([api.getTrafficZones(), api.getTrafficOccupancy()])
      setZones(z)
      setOccupancy(o)
    } catch { /* fleet manager may not be running */ }
  }, [])

  useEffect(() => { refresh() }, [refresh])
  useEffect(() => { const t = setInterval(refresh, 5000); return () => clearInterval(t) }, [refresh])

  const handleStartDraw = () => {
    setDrawing(true)
    onDrawZone?.(true)
  }

  const handleSaveZone = async () => {
    if (!newZoneName.trim() || !drawnPolygon || drawnPolygon.length < 3) {
      setError('Need name + at least 3 polygon points')
      return
    }
    setError('')
    try {
      await api.createTrafficZone({
        id: `zone_${Date.now()}`,
        type: newZoneType,
        polygon: drawnPolygon,
        maxRobots: newMaxRobots,
      })
      setNewZoneName('')
      setDrawing(false)
      onDrawZone?.(false)
      onClearDrawn?.()
      refresh()
    } catch (e) {
      setError('Failed to create zone')
    }
  }

  const handleDelete = async (id: string) => {
    try {
      await api.deleteTrafficZone(id)
      refresh()
    } catch { /* ignore */ }
  }

  const handleCancelDraw = () => {
    setDrawing(false)
    onDrawZone?.(false)
    onClearDrawn?.()
  }

  return (
    <div className="context-panel">
      <div className="panel-section">
        <div className="section-title">Traffic Zones</div>
        {zones.length === 0 && <p className="dim">No zones defined</p>}
        {zones.map(z => {
          const occ = occupancy.find(o => o.zoneId === z.id)
          return (
            <div key={z.id} className="traffic-zone-row">
              <span className="traffic-zone-dot" style={{ background: ZONE_COLORS[z.type] || 'var(--dim)' }} />
              <div className="traffic-zone-info">
                <span className="traffic-zone-name">{z.id.replace('zone_', '')}</span>
                <span className="traffic-zone-detail">
                  {z.type} | max {z.maxRobots}
                  {occ && occ.robotIds.length > 0 && ` | ${occ.robotIds.length} inside`}
                  {occ && occ.waitingRobots.length > 0 && ` | ${occ.waitingRobots.length} waiting`}
                </span>
              </div>
              <button className="small secondary" onClick={() => handleDelete(z.id)}>Del</button>
            </div>
          )
        })}
      </div>

      <div className="panel-section">
        <div className="section-title">Create Zone</div>
        {!drawing ? (
          <button className="full-width" onClick={handleStartDraw}>
            + Draw Zone on Map
          </button>
        ) : (
          <>
            <input
              type="text"
              placeholder="Zone name"
              value={newZoneName}
              onChange={e => setNewZoneName(e.target.value)}
              className="full-width"
            />
            <div className="template-row">
              {(['exclusion', 'one_way', 'yield'] as const).map(t => (
                <button
                  key={t}
                  className={`period-btn ${newZoneType === t ? 'period-btn-active' : ''}`}
                  onClick={() => setNewZoneType(t)}
                >
                  {t.replace('_', ' ')}
                </button>
              ))}
            </div>
            <div className="pause-input">
              <label>Max robots:</label>
              <input type="number" min={1} max={10} value={newMaxRobots}
                onChange={e => setNewMaxRobots(parseInt(e.target.value) || 1)} />
            </div>
            <p className="capture-status">
              {drawnPolygon ? `${drawnPolygon.length} points` : 'Click map to draw polygon'}
            </p>
            {error && <p className="login-error">{error}</p>}
            <div className="btn-row">
              <button onClick={handleSaveZone}
                disabled={!newZoneName.trim() || !drawnPolygon || drawnPolygon.length < 3}>
                Save
              </button>
              <button onClick={handleCancelDraw} className="secondary">Cancel</button>
            </div>
          </>
        )}
      </div>
    </div>
  )
}
