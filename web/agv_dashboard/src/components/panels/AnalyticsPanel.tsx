/**
 * Analytics panel — KPI display with custom Canvas sparklines.
 * Queries /api/analytics/summary and /api/analytics/timeseries.
 */

import { useState, useEffect, useRef, useCallback } from 'react'
import type { AnalyticsSummary, TimeseriesPoint, MissionRun } from '../../api/types'
import * as api from '../../api/client'

const PERIODS = [
  { label: '1h', value: '1h' },
  { label: '6h', value: '6h' },
  { label: '24h', value: '24h' },
  { label: '7d', value: '168h' },
]

function Sparkline({ data, color = '#4fc3f7', height = 40 }: { data: TimeseriesPoint[]; color?: string; height?: number }) {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas || data.length < 2) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const w = canvas.width
    const h = canvas.height
    ctx.clearRect(0, 0, w, h)

    const values = data.map(d => d.value)
    const min = Math.min(...values)
    const max = Math.max(...values)
    const range = max - min || 1

    ctx.strokeStyle = color
    ctx.lineWidth = 1.5
    ctx.beginPath()
    for (let i = 0; i < data.length; i++) {
      const x = (i / (data.length - 1)) * w
      const y = h - ((values[i] - min) / range) * (h - 4) - 2
      if (i === 0) ctx.moveTo(x, y)
      else ctx.lineTo(x, y)
    }
    ctx.stroke()

    // Fill under
    ctx.lineTo(w, h)
    ctx.lineTo(0, h)
    ctx.closePath()
    ctx.fillStyle = color.replace(')', ', 0.1)').replace('rgb', 'rgba')
    ctx.fill()
  }, [data, color, height])

  return (
    <canvas
      ref={canvasRef}
      width={200}
      height={height}
      style={{ width: '100%', height: `${height}px`, display: 'block' }}
    />
  )
}

function formatDuration(seconds: number): string {
  if (seconds < 60) return `${seconds}s`
  if (seconds < 3600) return `${Math.round(seconds / 60)}m`
  return `${(seconds / 3600).toFixed(1)}h`
}

async function fetchJson<T>(url: string): Promise<T | null> {
  try {
    const res = await fetch(url)
    if (!res.ok) return null
    return await res.json()
  } catch {
    return null
  }
}

export function AnalyticsPanel() {
  const [period, setPeriod] = useState('24h')
  const [summary, setSummary] = useState<AnalyticsSummary | null>(null)
  const [odomSeries, setOdomSeries] = useState<TimeseriesPoint[]>([])
  const [velSeries, setVelSeries] = useState<TimeseriesPoint[]>([])
  const [slamSeries, setSlamSeries] = useState<TimeseriesPoint[]>([])
  const [missionRuns, setMissionRuns] = useState<MissionRun[]>([])
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState('')

  const fetchData = useCallback(async () => {
    setLoading(true)
    setError('')

    const resolution = period.endsWith('h') && parseInt(period) <= 6 ? 30 : 60

    const [sumData, odomData, velData, slamData] = await Promise.all([
      fetchJson<AnalyticsSummary>(`/api/analytics/summary?period=${period}`),
      fetchJson<TimeseriesPoint[]>(`/api/analytics/timeseries?metric=odom_hz&resolution=${resolution}`),
      fetchJson<TimeseriesPoint[]>(`/api/analytics/timeseries?metric=linear_vel&resolution=${resolution}`),
      fetchJson<TimeseriesPoint[]>(`/api/analytics/timeseries?metric=slam_confidence&resolution=${resolution}`),
    ])

    if (sumData) {
      setSummary(sumData)
      setOdomSeries(odomData || [])
      setVelSeries(velData || [])
      setSlamSeries(slamData || [])

      // Mission runs
      const now = Date.now() / 1000
      const periodSec = period.endsWith('h') ? parseInt(period) * 3600 : 86400
      api.getMissionRuns(now - periodSec, now).then(setMissionRuns).catch(() => {})
    } else {
      setError('Analytics not available. Make sure the TypeScript backend is running (npm start in src/agv_ui_backend).')
    }

    setLoading(false)
  }, [period])

  useEffect(() => { fetchData() }, [fetchData])

  // Auto-refresh every 30s
  useEffect(() => {
    const timer = setInterval(fetchData, 30000)
    return () => clearInterval(timer)
  }, [fetchData])

  return (
    <div className="context-panel analytics-panel">
      {/* Period selector */}
      <div className="panel-section">
        <div className="section-title">Period</div>
        <div className="period-btns">
          {PERIODS.map(p => (
            <button
              key={p.value}
              className={`period-btn ${period === p.value ? 'period-btn-active' : ''}`}
              onClick={() => setPeriod(p.value)}
            >
              {p.label}
            </button>
          ))}
        </div>
      </div>

      {/* Error state */}
      {error && (
        <div className="panel-section">
          <div className="analytics-error">{error}</div>
          <button className="full-width" onClick={fetchData} style={{ marginTop: 8 }}>
            Retry
          </button>
        </div>
      )}

      {/* Loading state */}
      {loading && !summary && !error && <div className="analytics-loading">Loading...</div>}

      {/* No data state */}
      {!loading && !error && summary && summary.uptime_pct === 0 && summary.mission_count === 0 && odomSeries.length === 0 && (
        <div className="panel-section">
          <div className="analytics-empty">
            No telemetry data yet for this period. Data is recorded at 1Hz while the backend runs.
          </div>
        </div>
      )}

      {summary && (
        <>
          {/* KPI Cards */}
          <div className="panel-section">
            <div className="section-title">Key Metrics</div>
            <div className="kpi-grid">
              <div className="kpi-card">
                <span className="kpi-value">{summary.uptime_pct.toFixed(1)}%</span>
                <span className="kpi-label">Uptime</span>
              </div>
              <div className="kpi-card">
                <span className="kpi-value">{summary.distance_m.toFixed(1)}m</span>
                <span className="kpi-label">Distance</span>
              </div>
              <div className="kpi-card">
                <span className="kpi-value">{summary.mission_success_rate}%</span>
                <span className="kpi-label">Mission Success</span>
              </div>
              <div className="kpi-card">
                <span className="kpi-value">{summary.mission_count}</span>
                <span className="kpi-label">Missions</span>
              </div>
              <div className="kpi-card">
                <span className="kpi-value">{formatDuration(summary.avg_mission_duration_s)}</span>
                <span className="kpi-label">Avg Duration</span>
              </div>
              <div className="kpi-card">
                <span className="kpi-value">{summary.slam_good_pct}%</span>
                <span className="kpi-label">SLAM Good</span>
              </div>
            </div>
          </div>

          {/* Odom Hz chart */}
          {odomSeries.length > 1 && (
            <div className="panel-section">
              <div className="section-title">
                Odom Hz
                <span className="section-subtitle">
                  avg {summary.avg_odom_hz} | min {summary.min_odom_hz} | max {summary.max_odom_hz}
                </span>
              </div>
              <Sparkline data={odomSeries} color="#4fc3f7" />
            </div>
          )}

          {/* Velocity chart */}
          {velSeries.length > 1 && (
            <div className="panel-section">
              <div className="section-title">Linear Velocity (m/s)</div>
              <Sparkline data={velSeries} color="#4caf50" />
            </div>
          )}

          {/* SLAM confidence chart */}
          {slamSeries.length > 1 && (
            <div className="panel-section">
              <div className="section-title">SLAM Confidence</div>
              <Sparkline data={slamSeries} color="#ff9800" />
            </div>
          )}

          {/* Mission history (C5) */}
          {missionRuns.length > 0 && (
            <div className="panel-section">
              <div className="section-title">Mission History</div>
              <div className="mission-history">
                {missionRuns.map(run => {
                  const dur = run.ended ? Math.round(run.ended - run.started) : null
                  const statusColor = run.status === 'completed' ? 'var(--normal)' :
                    run.status === 'failed' ? 'var(--red)' : 'var(--dim)'
                  return (
                    <div key={run.id} className="mission-history-row">
                      <span className="mh-name">{run.mission_name || run.mission_id}</span>
                      <span className="mh-status" style={{ color: statusColor }}>{run.status}</span>
                      <span className="mh-nodes">{run.nodes_completed}/{run.total_nodes}</span>
                      {dur !== null && <span className="mh-dur">{formatDuration(dur)}</span>}
                    </div>
                  )
                })}
              </div>
            </div>
          )}
        </>
      )}
    </div>
  )
}
