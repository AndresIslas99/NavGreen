/**
 * Historical replay slider — shows robot path and state at any historical time.
 * Inspired by InOrbit Time Capsule / OTTO Snapshot Playback (simplified).
 *
 * Queries /api/replay/samples and /api/replay/events for a time range.
 * Renders a colored polyline on the map (color = robot_state) with a ghost marker.
 */

import { useState, useEffect, useCallback, useRef } from 'react'
import { apiUrl } from '../api/client'

interface ReplaySample {
  timestamp: number
  pose_x: number
  pose_y: number
  pose_theta: number
  linear_vel: number
  robot_state: string
  slam_confidence: string
}

interface ReplayEvent {
  timestamp: number
  severity: string
  subsystem: string
  text: string
}

interface Props {
  visible: boolean
  onGhostPose?: (pose: { x: number; y: number; theta: number } | null) => void
  onReplayEvents?: (events: ReplayEvent[]) => void
}

const STATE_COLORS: Record<string, string> = {
  ready: '#9e9e9e',
  idle: '#616161',
  mapping: '#1e88e5',
  navigating: '#1e88e5',
  executing_mission: '#1e88e5',
  blocked: '#ff9800',
  e_stop: '#f44336',
  fault: '#f44336',
  offline: '#424242',
}

const RANGE_OPTIONS = [
  { label: 'Last 1h', hours: 1 },
  { label: 'Last 6h', hours: 6 },
  { label: 'Last 24h', hours: 24 },
]

export function ReplaySlider({ visible, onGhostPose, onReplayEvents }: Props) {
  const [samples, setSamples] = useState<ReplaySample[]>([])
  const [events, setEvents] = useState<ReplayEvent[]>([])
  const [rangeHours, setRangeHours] = useState(1)
  const [sliderValue, setSliderValue] = useState(0) // 0 to samples.length-1
  const [loading, setLoading] = useState(false)
  const canvasRef = useRef<HTMLCanvasElement>(null)

  const fetchReplay = useCallback(async () => {
    setLoading(true)
    const now = Date.now() / 1000
    const from = now - rangeHours * 3600
    try {
      const [samplesRes, eventsRes] = await Promise.all([
        fetch(apiUrl(`/api/replay/samples?from=${from}&to=${now}`)),
        fetch(apiUrl(`/api/replay/events?from=${from}&to=${now}&limit=500`)),
      ])
      const s = await samplesRes.json()
      const e = await eventsRes.json()
      setSamples(s)
      setEvents(e)
      setSliderValue(s.length > 0 ? s.length - 1 : 0)
    } catch { /* ignore */ }
    setLoading(false)
  }, [rangeHours])

  useEffect(() => {
    if (visible) fetchReplay()
  }, [visible, fetchReplay])

  // Update ghost pose when slider changes
  useEffect(() => {
    if (!samples.length || !onGhostPose) return
    const s = samples[Math.min(sliderValue, samples.length - 1)]
    if (s) {
      onGhostPose({ x: s.pose_x, y: s.pose_y, theta: s.pose_theta })
    }
  }, [sliderValue, samples, onGhostPose])

  // Filter events to current time
  useEffect(() => {
    if (!samples.length || !onReplayEvents) return
    const current = samples[Math.min(sliderValue, samples.length - 1)]
    if (!current) return
    const windowStart = current.timestamp - 30
    const windowEnd = current.timestamp + 30
    const filtered = events.filter(e => e.timestamp >= windowStart && e.timestamp <= windowEnd)
    onReplayEvents(filtered)
  }, [sliderValue, samples, events, onReplayEvents])

  // Draw mini timeline canvas
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas || !samples.length) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const w = canvas.width
    const h = canvas.height
    ctx.clearRect(0, 0, w, h)

    // Draw colored segments by state
    const segWidth = w / samples.length
    for (let i = 0; i < samples.length; i++) {
      ctx.fillStyle = STATE_COLORS[samples[i].robot_state] || '#424242'
      ctx.fillRect(i * segWidth, 0, Math.ceil(segWidth) + 1, h)
    }

    // Draw current position line
    const cx = (sliderValue / (samples.length - 1)) * w
    ctx.strokeStyle = '#fff'
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(cx, 0)
    ctx.lineTo(cx, h)
    ctx.stroke()
  }, [samples, sliderValue])

  if (!visible) return null

  const currentSample = samples[Math.min(sliderValue, samples.length - 1)]
  const currentTime = currentSample
    ? new Date(currentSample.timestamp * 1000).toLocaleTimeString()
    : '--:--:--'

  return (
    <div className="replay-panel">
      <div className="replay-header">
        <span className="replay-title">Replay</span>
        <div className="replay-range-btns">
          {RANGE_OPTIONS.map(opt => (
            <button
              key={opt.hours}
              className={`period-btn ${rangeHours === opt.hours ? 'period-btn-active' : ''}`}
              onClick={() => setRangeHours(opt.hours)}
            >
              {opt.label}
            </button>
          ))}
        </div>
      </div>

      {loading ? (
        <div className="analytics-loading">Loading replay data...</div>
      ) : samples.length === 0 ? (
        <div className="analytics-loading">No data for this period</div>
      ) : (
        <>
          {/* State timeline */}
          <canvas
            ref={canvasRef}
            width={300}
            height={16}
            className="replay-timeline"
          />

          {/* Slider */}
          <input
            type="range"
            min={0}
            max={samples.length - 1}
            value={sliderValue}
            onChange={e => setSliderValue(parseInt(e.target.value))}
            className="replay-slider"
          />

          {/* Info */}
          <div className="replay-info">
            <span className="replay-time">{currentTime}</span>
            {currentSample && (
              <>
                <span className="replay-state"
                  style={{ color: STATE_COLORS[currentSample.robot_state] || '#999' }}>
                  {currentSample.robot_state}
                </span>
                <span className="replay-pos">
                  ({currentSample.pose_x.toFixed(2)}, {currentSample.pose_y.toFixed(2)})
                </span>
                <span className="replay-vel">
                  {currentSample.linear_vel.toFixed(2)} m/s
                </span>
              </>
            )}
          </div>
        </>
      )}
    </div>
  )
}
