import { useRef, useEffect, useCallback } from 'react'
import type { MapUpdate, PathPoint } from '../api/types'

interface Props {
  mapData: MapUpdate | null
  pose: { x: number; y: number; theta: number }
  path: PathPoint[]
  scanPoints: PathPoint[]
  mode: string
  onGoalClick?: (x: number, y: number) => void
  waypoints?: { x: number; y: number }[]
}

export function MapCanvas({ mapData, pose, path, scanPoints, mode, onGoalClick, waypoints }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const imgRef = useRef<HTMLImageElement | null>(null)
  const trailRef = useRef<{ x: number; y: number }[]>([])
  const panRef = useRef({ x: 0, y: 0, scale: 1 })

  // Load map image
  useEffect(() => {
    if (!mapData?.png_base64) return
    const img = new window.Image()
    img.src = `data:image/png;base64,${mapData.png_base64}`
    img.onload = () => { imgRef.current = img }
  }, [mapData?.png_base64])

  // Render loop
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    let animId: number

    function draw() {
      const ctx = canvas!.getContext('2d')
      if (!ctx) return
      const w = canvas!.width
      const h = canvas!.height
      const { x: px, y: py, scale } = panRef.current

      ctx.fillStyle = '#1a1a2e'
      ctx.fillRect(0, 0, w, h)

      ctx.save()
      ctx.translate(w / 2 + px, h / 2 + py)
      ctx.scale(scale, scale)

      // Draw map image
      if (imgRef.current && mapData) {
        const iw = mapData.width
        const ih = mapData.height
        const res = mapData.resolution
        const ox = mapData.origin_x
        const oy = mapData.origin_y
        // Map pixels: (mx, my) → world: (ox + mx*res, oy + (height - my)*res)
        // We draw centered on world origin
        ctx.save()
        ctx.scale(1, -1)  // flip Y for world coords
        ctx.drawImage(imgRef.current, ox / res, -(oy / res + ih), iw, ih)
        ctx.restore()
      }

      // Draw grid lines (1m spacing)
      {
        const res = mapData?.resolution || 0.05
        const gridSpacing = 1.0 / res  // 1 meter in pixels
        const extent = 200  // grid extent in meters
        ctx.strokeStyle = 'rgba(255,255,255,0.06)'
        ctx.lineWidth = 1 / scale
        for (let m = -extent; m <= extent; m++) {
          const px = m * gridSpacing
          ctx.beginPath()
          ctx.moveTo(px, -extent * gridSpacing)
          ctx.lineTo(px, extent * gridSpacing)
          ctx.stroke()
          ctx.beginPath()
          ctx.moveTo(-extent * gridSpacing, px)
          ctx.lineTo(extent * gridSpacing, px)
          ctx.stroke()
        }
        // Origin crosshair
        ctx.strokeStyle = 'rgba(255,255,255,0.2)'
        ctx.lineWidth = 1.5 / scale
        ctx.beginPath()
        ctx.moveTo(-10 / scale, 0)
        ctx.lineTo(10 / scale, 0)
        ctx.moveTo(0, -10 / scale)
        ctx.lineTo(0, 10 / scale)
        ctx.stroke()
      }

      // Draw pose trail (breadcrumb path of where robot has been)
      {
        const res = mapData?.resolution || 0.05
        const trail = trailRef.current
        const last = trail[trail.length - 1]
        if (!last || Math.abs(pose.x - last.x) > 0.05 || Math.abs(pose.y - last.y) > 0.05) {
          trail.push({ x: pose.x, y: pose.y })
          if (trail.length > 2000) trail.shift()
        }
        if (trail.length > 1) {
          ctx.strokeStyle = 'rgba(76, 175, 80, 0.4)'
          ctx.lineWidth = 2 / scale
          ctx.beginPath()
          ctx.moveTo(trail[0].x / res, -trail[0].y / res)
          for (let i = 1; i < trail.length; i++) {
            ctx.lineTo(trail[i].x / res, -trail[i].y / res)
          }
          ctx.stroke()
        }
      }

      // Draw path
      if (path.length > 1) {
        ctx.strokeStyle = '#4fc3f7'
        ctx.lineWidth = 2 / scale
        ctx.beginPath()
        ctx.moveTo(path[0].x / (mapData?.resolution || 0.05), -path[0].y / (mapData?.resolution || 0.05))
        for (let i = 1; i < path.length; i++) {
          ctx.lineTo(path[i].x / (mapData?.resolution || 0.05), -path[i].y / (mapData?.resolution || 0.05))
        }
        ctx.stroke()
      }

      // Draw waypoint markers
      if (waypoints) {
        const r = mapData?.resolution || 0.05
        waypoints.forEach((wp, i) => {
          const sx = wp.x / r
          const sy = -wp.y / r
          ctx.fillStyle = '#ff9800'
          ctx.beginPath()
          ctx.arc(sx, sy, 6 / scale, 0, Math.PI * 2)
          ctx.fill()
          ctx.fillStyle = '#fff'
          ctx.font = `${12 / scale}px sans-serif`
          ctx.fillText(`${i + 1}`, sx + 8 / scale, sy + 4 / scale)
        })
      }

      // Draw scan points (live laser data as red dots)
      if (scanPoints.length > 0) {
        const r = mapData?.resolution || 0.05
        ctx.fillStyle = 'rgba(244, 67, 54, 0.7)'
        for (const pt of scanPoints) {
          const sx = pt.x / r
          const sy = -pt.y / r
          ctx.fillRect(sx - 1 / scale, sy - 1 / scale, 2 / scale, 2 / scale)
        }
      }

      // Draw robot pose
      if (mapData) {
        const r = mapData.resolution
        const rx = pose.x / r
        const ry = -pose.y / r
        const size = 8 / scale

        ctx.save()
        ctx.translate(rx, ry)
        ctx.rotate(-pose.theta)
        ctx.fillStyle = '#4caf50'
        ctx.beginPath()
        ctx.moveTo(size * 1.5, 0)
        ctx.lineTo(-size, -size)
        ctx.lineTo(-size, size)
        ctx.closePath()
        ctx.fill()
        ctx.restore()
      }

      ctx.restore()
      animId = requestAnimationFrame(draw)
    }

    draw()
    return () => cancelAnimationFrame(animId)
  }, [mapData, pose, path, scanPoints, waypoints])

  // Resize canvas
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const obs = new ResizeObserver(() => {
      canvas.width = canvas.clientWidth
      canvas.height = canvas.clientHeight
    })
    obs.observe(canvas)
    canvas.width = canvas.clientWidth
    canvas.height = canvas.clientHeight
    return () => obs.disconnect()
  }, [])

  // Click-to-goal
  const handleClick = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (mode !== 'nav' || !onGoalClick || !mapData) return
    const canvas = canvasRef.current!
    const rect = canvas.getBoundingClientRect()
    const { x: px, y: py, scale } = panRef.current
    const cx = (e.clientX - rect.left - canvas.width / 2 - px) / scale
    const cy = (e.clientY - rect.top - canvas.height / 2 - py) / scale
    const worldX = cx * mapData.resolution
    const worldY = -cy * mapData.resolution
    onGoalClick(worldX, worldY)
  }, [mode, onGoalClick, mapData])

  // Pan and zoom
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    let dragging = false
    let lastX = 0, lastY = 0

    const onWheel = (e: WheelEvent) => {
      e.preventDefault()
      const factor = e.deltaY > 0 ? 0.9 : 1.1
      panRef.current.scale *= factor
    }

    const onPointerDown = (e: PointerEvent) => {
      dragging = true
      lastX = e.clientX
      lastY = e.clientY
      canvas.setPointerCapture(e.pointerId)
    }

    const onPointerMove = (e: PointerEvent) => {
      if (!dragging) return
      panRef.current.x += e.clientX - lastX
      panRef.current.y += e.clientY - lastY
      lastX = e.clientX
      lastY = e.clientY
    }

    const onPointerUp = () => { dragging = false }

    canvas.addEventListener('wheel', onWheel, { passive: false })
    canvas.addEventListener('pointerdown', onPointerDown)
    canvas.addEventListener('pointermove', onPointerMove)
    canvas.addEventListener('pointerup', onPointerUp)

    return () => {
      canvas.removeEventListener('wheel', onWheel)
      canvas.removeEventListener('pointerdown', onPointerDown)
      canvas.removeEventListener('pointermove', onPointerMove)
      canvas.removeEventListener('pointerup', onPointerUp)
    }
  }, [])

  // Center on robot
  const centerOnRobot = useCallback(() => {
    if (!mapData) return
    const r = mapData.resolution
    panRef.current.x = -(pose.x / r) * panRef.current.scale
    panRef.current.y = (pose.y / r) * panRef.current.scale
  }, [mapData, pose])

  // Zoom buttons
  const zoomIn = () => { panRef.current.scale *= 1.3 }
  const zoomOut = () => { panRef.current.scale *= 0.7 }

  // Scale bar (unused var removed)
  void panRef.current.scale  // referenced for reactivity

  return (
    <div className="map-container">
      <canvas
        ref={canvasRef}
        className="map-canvas"
        onClick={handleClick}
      />
      {/* Map overlays */}
      <div className="map-overlay-tl">
        <span className="map-coord">
          ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}) {(pose.theta * 180 / Math.PI).toFixed(0)}°
        </span>
      </div>
      <div className="map-overlay-tr">
        <button className="map-btn" onClick={zoomIn} title="Zoom in">+</button>
        <button className="map-btn" onClick={zoomOut} title="Zoom out">−</button>
        <button className="map-btn" onClick={centerOnRobot} title="Center on robot">⊕</button>
      </div>
      <div className="map-overlay-bl">
        <div className="scale-bar">
          <div className="scale-line" />
          <span className="scale-text">1m</span>
        </div>
      </div>
    </div>
  )
}
