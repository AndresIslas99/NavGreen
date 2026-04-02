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
  const dirtyRef = useRef(true)
  const animIdRef = useRef<number>(0)

  // Offscreen grid canvas (cached, re-rendered only on zoom change)
  const gridCanvasRef = useRef<OffscreenCanvas | null>(null)
  const gridScaleRef = useRef(0)
  const gridResRef = useRef(0)

  // Load map image
  useEffect(() => {
    if (!mapData?.png_base64) return
    const img = new window.Image()
    img.src = `data:image/png;base64,${mapData.png_base64}`
    img.onload = () => { imgRef.current = img; dirtyRef.current = true }
  }, [mapData?.png_base64])

  // Mark dirty when data changes
  useEffect(() => { dirtyRef.current = true }, [pose, path, scanPoints, waypoints, mapData])

  // Render loop (only draws when dirty)
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    function draw() {
      if (dirtyRef.current) {
        dirtyRef.current = false
        renderFrame()
      }
      animIdRef.current = requestAnimationFrame(draw)
    }

    function renderFrame() {
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
        ctx.save()
        ctx.scale(1, -1)
        ctx.drawImage(imgRef.current, ox / res, -(oy / res + ih), iw, ih)
        ctx.restore()
      }

      // Draw grid (cached on offscreen canvas)
      drawGrid(ctx, scale)

      // Draw pose trail
      const res = mapData?.resolution || 0.05
      {
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
        ctx.moveTo(path[0].x / res, -path[0].y / res)
        for (let i = 1; i < path.length; i++) {
          ctx.lineTo(path[i].x / res, -path[i].y / res)
        }
        ctx.stroke()
      }

      // Draw waypoint markers
      if (waypoints) {
        waypoints.forEach((wp, i) => {
          const sx = wp.x / res
          const sy = -wp.y / res
          ctx.fillStyle = '#ff9800'
          ctx.beginPath()
          ctx.arc(sx, sy, 6 / scale, 0, Math.PI * 2)
          ctx.fill()
          ctx.fillStyle = '#fff'
          ctx.font = `${12 / scale}px sans-serif`
          ctx.fillText(`${i + 1}`, sx + 8 / scale, sy + 4 / scale)
        })
      }

      // Draw scan points
      if (scanPoints.length > 0) {
        ctx.fillStyle = 'rgba(244, 67, 54, 0.7)'
        for (const pt of scanPoints) {
          const sx = pt.x / res
          const sy = -pt.y / res
          ctx.fillRect(sx - 1 / scale, sy - 1 / scale, 2 / scale, 2 / scale)
        }
      }

      // Draw robot pose
      if (mapData) {
        const rx = pose.x / res
        const ry = -pose.y / res
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
    }

    function drawGrid(ctx: CanvasRenderingContext2D, scale: number) {
      const res = mapData?.resolution || 0.05
      const gridSpacing = 1.0 / res

      // Regenerate offscreen grid if scale or resolution changed
      if (typeof OffscreenCanvas !== 'undefined' &&
          (gridScaleRef.current !== scale || gridResRef.current !== res)) {
        const extent = 200
        const gridSize = extent * 2 * gridSpacing
        try {
          const offscreen = new OffscreenCanvas(Math.min(gridSize, 4096), Math.min(gridSize, 4096))
          const gctx = offscreen.getContext('2d')
          if (gctx) {
            const renderExtent = Math.min(extent, Math.floor(offscreen.width / gridSpacing / 2))
            gctx.strokeStyle = 'rgba(255,255,255,0.06)'
            gctx.lineWidth = 1 / scale
            const cx = offscreen.width / 2
            const cy = offscreen.height / 2
            for (let m = -renderExtent; m <= renderExtent; m++) {
              const px = cx + m * gridSpacing
              gctx.beginPath()
              gctx.moveTo(px, 0)
              gctx.lineTo(px, offscreen.height)
              gctx.stroke()
              gctx.beginPath()
              gctx.moveTo(0, cy + m * gridSpacing)
              gctx.lineTo(offscreen.width, cy + m * gridSpacing)
              gctx.stroke()
            }
            gridCanvasRef.current = offscreen
            gridScaleRef.current = scale
            gridResRef.current = res
          }
        } catch {
          // OffscreenCanvas not supported, fall through
        }
      }

      if (gridCanvasRef.current) {
        const gc = gridCanvasRef.current
        ctx.drawImage(gc, -gc.width / 2, -gc.height / 2)
      } else {
        // Fallback: draw directly (no caching)
        const gridExtent = 200
        ctx.strokeStyle = 'rgba(255,255,255,0.06)'
        ctx.lineWidth = 1 / scale
        for (let m = -gridExtent; m <= gridExtent; m++) {
          const px = m * gridSpacing
          ctx.beginPath()
          ctx.moveTo(px, -gridExtent * gridSpacing)
          ctx.lineTo(px, gridExtent * gridSpacing)
          ctx.stroke()
          ctx.beginPath()
          ctx.moveTo(-gridExtent * gridSpacing, px)
          ctx.lineTo(gridExtent * gridSpacing, px)
          ctx.stroke()
        }
      }

      // Origin crosshair (always drawn fresh)
      ctx.strokeStyle = 'rgba(255,255,255,0.2)'
      ctx.lineWidth = 1.5 / scale
      ctx.beginPath()
      ctx.moveTo(-10 / scale, 0)
      ctx.lineTo(10 / scale, 0)
      ctx.moveTo(0, -10 / scale)
      ctx.lineTo(0, 10 / scale)
      ctx.stroke()
    }

    draw()
    return () => cancelAnimationFrame(animIdRef.current)
  }, [mapData, pose, path, scanPoints, waypoints])

  // Resize canvas
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const obs = new ResizeObserver(() => {
      canvas.width = canvas.clientWidth
      canvas.height = canvas.clientHeight
      dirtyRef.current = true
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

  // Pan, zoom, and pinch-to-zoom
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    let dragging = false
    let lastX = 0, lastY = 0

    // Track active touches for pinch-to-zoom
    const activeTouches = new Map<number, { x: number; y: number }>()

    const onWheel = (e: WheelEvent) => {
      e.preventDefault()
      const factor = e.deltaY > 0 ? 0.9 : 1.1
      panRef.current.scale *= factor
      gridScaleRef.current = 0 // invalidate grid cache
      dirtyRef.current = true
    }

    const onPointerDown = (e: PointerEvent) => {
      activeTouches.set(e.pointerId, { x: e.clientX, y: e.clientY })
      if (activeTouches.size === 1) {
        dragging = true
        lastX = e.clientX
        lastY = e.clientY
        canvas.setPointerCapture(e.pointerId)
      }
    }

    const onPointerMove = (e: PointerEvent) => {
      activeTouches.set(e.pointerId, { x: e.clientX, y: e.clientY })

      if (activeTouches.size >= 2) {
        // Pinch-to-zoom
        const pts = Array.from(activeTouches.values())
        const dx = pts[1].x - pts[0].x
        const dy = pts[1].y - pts[0].y
        const dist = Math.sqrt(dx * dx + dy * dy)
        const prevRef = (canvas as any).__pinchDist as number | undefined
        if (prevRef && prevRef > 0) {
          const factor = dist / prevRef
          panRef.current.scale *= factor
          gridScaleRef.current = 0
          dirtyRef.current = true
        }
        ;(canvas as any).__pinchDist = dist
        dragging = false
        return
      }

      if (!dragging) return
      panRef.current.x += e.clientX - lastX
      panRef.current.y += e.clientY - lastY
      lastX = e.clientX
      lastY = e.clientY
      dirtyRef.current = true
    }

    const onPointerUp = (e: PointerEvent) => {
      activeTouches.delete(e.pointerId)
      if (activeTouches.size < 2) {
        ;(canvas as any).__pinchDist = 0
      }
      if (activeTouches.size === 0) {
        dragging = false
      }
    }

    canvas.addEventListener('wheel', onWheel, { passive: false })
    canvas.addEventListener('pointerdown', onPointerDown)
    canvas.addEventListener('pointermove', onPointerMove)
    canvas.addEventListener('pointerup', onPointerUp)
    canvas.addEventListener('pointercancel', onPointerUp)

    // Prevent default touch behavior (scrolling, zooming page)
    const preventTouch = (e: TouchEvent) => { if (e.touches.length >= 2) e.preventDefault() }
    canvas.addEventListener('touchstart', preventTouch, { passive: false })
    canvas.addEventListener('touchmove', preventTouch, { passive: false })

    return () => {
      canvas.removeEventListener('wheel', onWheel)
      canvas.removeEventListener('pointerdown', onPointerDown)
      canvas.removeEventListener('pointermove', onPointerMove)
      canvas.removeEventListener('pointerup', onPointerUp)
      canvas.removeEventListener('pointercancel', onPointerUp)
      canvas.removeEventListener('touchstart', preventTouch)
      canvas.removeEventListener('touchmove', preventTouch)
    }
  }, [])

  // Center on robot
  const centerOnRobot = useCallback(() => {
    if (!mapData) return
    const r = mapData.resolution
    panRef.current.x = -(pose.x / r) * panRef.current.scale
    panRef.current.y = (pose.y / r) * panRef.current.scale
    dirtyRef.current = true
  }, [mapData, pose])

  // Zoom buttons
  const zoomIn = () => {
    panRef.current.scale *= 1.3
    gridScaleRef.current = 0
    dirtyRef.current = true
  }
  const zoomOut = () => {
    panRef.current.scale *= 0.7
    gridScaleRef.current = 0
    dirtyRef.current = true
  }

  return (
    <div className="map-container">
      <canvas
        ref={canvasRef}
        className="map-canvas"
        onClick={handleClick}
        style={{ touchAction: 'none' }}
      />
      {/* Map overlays */}
      <div className="map-overlay-tl">
        <span className="map-coord">
          ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}) {(pose.theta * 180 / Math.PI).toFixed(0)}°
        </span>
      </div>
      <div className="map-overlay-tr">
        <button className="map-btn" onClick={zoomIn} title="Zoom in">+</button>
        <button className="map-btn" onClick={zoomOut} title="Zoom out">-</button>
        <button className="map-btn" onClick={centerOnRobot} title="Center on robot">&#8853;</button>
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
