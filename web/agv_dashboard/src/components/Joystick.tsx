import { useRef, useEffect, useCallback } from 'react'

interface Props {
  enabled: boolean
  maxLinear: number
  maxAngular: number
  onMove: (linear: number, angular: number) => void
}

export function Joystick({ enabled, maxLinear, maxAngular, onMove }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const stickRef = useRef({ x: 0, y: 0 })
  const dragging = useRef(false)
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null)

  const draw = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    const w = canvas.width
    const h = canvas.height
    const cx = w / 2
    const cy = h / 2
    const baseR = Math.min(w, h) / 2 - 10
    const stickR = baseR * 0.35

    ctx.clearRect(0, 0, w, h)

    // Base circle
    ctx.beginPath()
    ctx.arc(cx, cy, baseR, 0, Math.PI * 2)
    ctx.fillStyle = enabled ? 'rgba(79,195,247,0.1)' : 'rgba(100,100,100,0.05)'
    ctx.fill()
    ctx.strokeStyle = enabled ? 'rgba(79,195,247,0.4)' : 'rgba(100,100,100,0.2)'
    ctx.lineWidth = 2
    ctx.stroke()

    // Stick
    const sx = cx + stickRef.current.x * baseR
    const sy = cy + stickRef.current.y * baseR
    ctx.beginPath()
    ctx.arc(sx, sy, stickR, 0, Math.PI * 2)
    ctx.fillStyle = enabled
      ? (dragging.current ? 'rgba(79,195,247,0.6)' : 'rgba(79,195,247,0.3)')
      : 'rgba(100,100,100,0.15)'
    ctx.fill()
    ctx.strokeStyle = enabled ? '#4fc3f7' : '#555'
    ctx.lineWidth = 2
    ctx.stroke()
  }, [enabled])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    canvas.width = canvas.clientWidth
    canvas.height = canvas.clientHeight
    draw()

    const obs = new ResizeObserver(() => {
      canvas.width = canvas.clientWidth
      canvas.height = canvas.clientHeight
      draw()
    })
    obs.observe(canvas)
    return () => obs.disconnect()
  }, [draw])

  useEffect(() => {
    // Redraw at 30fps when dragging
    const id = setInterval(draw, 33)
    return () => clearInterval(id)
  }, [draw])

  // Send commands at 10Hz
  useEffect(() => {
    intervalRef.current = setInterval(() => {
      if (enabled && dragging.current) {
        const { x, y } = stickRef.current
        onMove(-y * maxLinear, -x * maxAngular)
      }
    }, 100)
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current)
    }
  }, [enabled, maxLinear, maxAngular, onMove])

  const getStickPos = (e: React.PointerEvent) => {
    const canvas = canvasRef.current!
    const rect = canvas.getBoundingClientRect()
    const cx = canvas.width / 2
    const cy = canvas.height / 2
    const baseR = Math.min(canvas.width, canvas.height) / 2 - 10
    let dx = (e.clientX - rect.left - cx) / baseR
    let dy = (e.clientY - rect.top - cy) / baseR
    const dist = Math.sqrt(dx * dx + dy * dy)
    if (dist > 1) { dx /= dist; dy /= dist }
    return { x: dx, y: dy }
  }

  const handlePointerDown = (e: React.PointerEvent) => {
    if (!enabled) return
    dragging.current = true
    canvasRef.current?.setPointerCapture(e.pointerId)
    stickRef.current = getStickPos(e)
  }

  const handlePointerMove = (e: React.PointerEvent) => {
    if (!dragging.current) return
    stickRef.current = getStickPos(e)
  }

  const handlePointerUp = () => {
    dragging.current = false
    stickRef.current = { x: 0, y: 0 }
    onMove(0, 0)
  }

  return (
    <div className="joystick-container">
      <canvas
        ref={canvasRef}
        className={`joystick-zone ${enabled ? '' : 'disabled'}`}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        style={{ touchAction: 'none' }}
      />
      {!enabled && <div className="joystick-overlay">Not in Teleop</div>}
    </div>
  )
}
