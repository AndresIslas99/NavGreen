/**
 * Leaflet-based map view — replaces custom Canvas MapCanvas.
 *
 * Uses L.CRS.Simple for pixel-based indoor coordinates.
 * Layers: occupancy grid, scan points, nav path, pose trail, waypoints, robot pose.
 */

import { useRef, useEffect, useCallback, useState } from 'react'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'
import type { MapUpdate, PathPoint } from '../api/types'
import type { FleetRobot } from '../hooks/useFleetSocket'

interface Props {
  mapData: MapUpdate | null
  pose: { x: number; y: number; theta: number }
  path: PathPoint[]
  scanPoints: PathPoint[]
  mode: string
  onGoalClick?: (x: number, y: number) => void
  waypoints?: { x: number; y: number }[]
  fleetRobots?: FleetRobot[]
  selectedRobot?: string | null
  ghostPose?: { x: number; y: number; theta: number } | null
  mappingCoverage?: number
}

// Robot icon as rotated div
// In Leaflet CRS.Simple: X→right (lng), Y→up (lat)
// In ROS: theta=0 means facing X+ (right). CSS rotate(0)=up.
// So we need -90° offset: theta=0 → arrow points right
function robotIcon(theta: number): L.DivIcon {
  const deg = -(theta * 180 / Math.PI) + 90
  return L.divIcon({
    className: 'robot-marker',
    html: `<div class="robot-arrow" style="transform: rotate(${deg}deg)"></div>`,
    iconSize: [24, 24],
    iconAnchor: [12, 12],
  })
}

// Convert world coords to Leaflet LatLng (y=lat, x=lng in CRS.Simple)
function worldToLatLng(x: number, y: number): L.LatLng {
  return L.latLng(y, x)
}

export function MapView({ mapData, pose, path, scanPoints, mode, onGoalClick, waypoints, fleetRobots, selectedRobot, ghostPose, mappingCoverage }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const mapRef = useRef<L.Map | null>(null)

  // Layer refs
  const imageLayerRef = useRef<L.ImageOverlay | null>(null)
  const robotMarkerRef = useRef<L.Marker | null>(null)
  const pathLayerRef = useRef<L.Polyline | null>(null)
  const trailLayerRef = useRef<L.Polyline | null>(null)
  // scanLayerRef unused — scan points managed via scanGroupRef
  const scanGroupRef = useRef<L.LayerGroup | null>(null)
  const waypointLayerRef = useRef<L.LayerGroup | null>(null)

  // Trail accumulator
  const trailRef = useRef<L.LatLng[]>([])

  // Track if user has manually panned (don't auto-center)
  const userPannedRef = useRef(false)
  const [followRobot, setFollowRobot] = useState(true)

  // Initialize map
  useEffect(() => {
    if (!containerRef.current || mapRef.current) return

    const map = L.map(containerRef.current, {
      crs: L.CRS.Simple,
      minZoom: -3,
      maxZoom: 5,
      zoomControl: false,
      attributionControl: false,
      doubleClickZoom: false,
      touchZoom: true,
      dragging: true,
    })

    map.setView([0, 0], 1)

    // Add zoom control in top-right
    L.control.zoom({ position: 'topright' }).addTo(map)

    // Layer groups
    const scanGroup = L.layerGroup().addTo(map)
    scanGroupRef.current = scanGroup

    const waypointGroup = L.layerGroup().addTo(map)
    waypointLayerRef.current = waypointGroup

    // Track user interaction
    map.on('dragstart', () => {
      userPannedRef.current = true
      setFollowRobot(false)
    })

    // Click-to-goal
    map.on('click', (e: L.LeafletMouseEvent) => {
      if (mode === 'nav' && onGoalClick) {
        onGoalClick(e.latlng.lng, e.latlng.lat)
      }
    })

    mapRef.current = map

    return () => {
      map.remove()
      mapRef.current = null
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // Update click handler when mode/callback changes
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    map.off('click')
    map.on('click', (e: L.LeafletMouseEvent) => {
      if (mode === 'nav' && onGoalClick) {
        onGoalClick(e.latlng.lng, e.latlng.lat)
      }
    })
  }, [mode, onGoalClick])

  // Update map image overlay
  const isAccMap = useRef(false)
  useEffect(() => {
    const map = mapRef.current
    if (!map || !mapData?.png_base64) return

    const { width, height, resolution, origin_x, origin_y } = mapData
    const southWest = worldToLatLng(origin_x, origin_y)
    const northEast = worldToLatLng(origin_x + width * resolution, origin_y + height * resolution)
    const bounds = L.latLngBounds(southWest, northEast)

    // Detect accumulated map: 500x500 grid from scan_accumulator (not the 400x400 nav map)
    const isAccumulatedMap = width >= 500

    const imageUrl = `data:image/png;base64,${mapData.png_base64}`

    if (imageLayerRef.current) {
      imageLayerRef.current.setUrl(imageUrl)
      imageLayerRef.current.setBounds(bounds)
    } else {
      const overlay = L.imageOverlay(imageUrl, bounds, { opacity: 0.9 }).addTo(map)
      imageLayerRef.current = overlay

      if (isAccumulatedMap) {
        // Acc map: zoom to robot position, not full 50m grid
        isAccMap.current = true
        map.setView(worldToLatLng(pose.x, pose.y), 3)
      } else {
        // Static navigation map: fit to map bounds
        isAccMap.current = false
        map.fitBounds(bounds)
      }
    }
  }, [mapData])

  // Update robot position
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    const latlng = worldToLatLng(pose.x, pose.y)

    if (robotMarkerRef.current) {
      robotMarkerRef.current.setLatLng(latlng)
      robotMarkerRef.current.setIcon(robotIcon(pose.theta))
    } else {
      const marker = L.marker(latlng, {
        icon: robotIcon(pose.theta),
        zIndexOffset: 1000,
      }).addTo(map)
      robotMarkerRef.current = marker
    }

    // Update pose trail
    const trail = trailRef.current
    const lastPt = trail[trail.length - 1]
    if (!lastPt || latlng.distanceTo(lastPt) > 0.05) {
      trail.push(latlng)
      if (trail.length > 2000) trail.shift()
    }

    if (trailLayerRef.current) {
      trailLayerRef.current.setLatLngs(trail)
    } else {
      trailLayerRef.current = L.polyline(trail, {
        color: '#4caf50',
        weight: 2,
        opacity: 0.4,
      }).addTo(map)
    }

    // Follow robot
    if (followRobot && !userPannedRef.current) {
      map.panTo(latlng, { animate: false })
    }
  }, [pose, followRobot])

  // Update navigation path
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    const latlngs = path.map(p => worldToLatLng(p.x, p.y))

    if (pathLayerRef.current) {
      pathLayerRef.current.setLatLngs(latlngs)
    } else if (latlngs.length > 0) {
      pathLayerRef.current = L.polyline(latlngs, {
        color: '#4fc3f7',
        weight: 3,
        opacity: 0.8,
        dashArray: '8, 4',
      }).addTo(map)
    }

    if (latlngs.length === 0 && pathLayerRef.current) {
      pathLayerRef.current.setLatLngs([])
    }
  }, [path])

  // Update scan points
  useEffect(() => {
    const group = scanGroupRef.current
    if (!group) return

    // Clear old scan markers
    group.clearLayers()

    // Only show every Nth point for performance
    const step = Math.max(1, Math.floor(scanPoints.length / 200))
    for (let i = 0; i < scanPoints.length; i += step) {
      const pt = scanPoints[i]
      L.circleMarker(worldToLatLng(pt.x, pt.y), {
        radius: 2,
        color: '#f44336',
        fillColor: '#f44336',
        fillOpacity: 0.7,
        weight: 0,
      }).addTo(group)
    }
  }, [scanPoints])

  // Update waypoint markers
  useEffect(() => {
    const group = waypointLayerRef.current
    if (!group) return
    group.clearLayers()

    if (!waypoints) return
    waypoints.forEach((wp, i) => {
      const marker = L.circleMarker(worldToLatLng(wp.x, wp.y), {
        radius: 8,
        color: '#ff9800',
        fillColor: '#ff9800',
        fillOpacity: 0.8,
        weight: 2,
      }).addTo(group)

      marker.bindTooltip(`${i + 1}`, {
        permanent: true,
        direction: 'right',
        offset: [8, 0],
        className: 'waypoint-tooltip',
      })
    })
  }, [waypoints])

  // Fleet robot markers (P3.2)
  const fleetLayerRef = useRef<L.LayerGroup | null>(null)
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    if (!fleetLayerRef.current) {
      fleetLayerRef.current = L.layerGroup().addTo(map)
    }
    const group = fleetLayerRef.current
    group.clearLayers()

    if (!fleetRobots || fleetRobots.length === 0) return

    for (const robot of fleetRobots) {
      const pos = robot.position
      const deg = -(pos.theta * 180 / Math.PI)
      const isSelected = selectedRobot === robot.id
      const errorClass = robot.errorCount > 0 ? ' fleet-arrow-error' : ''
      const offlineClass = robot.connectionState !== 'ONLINE' ? ' fleet-arrow-offline' : ''
      const size = isSelected ? 'transform: scale(1.3);' : ''

      const icon = L.divIcon({
        className: 'fleet-marker',
        html: `<div class="fleet-arrow${errorClass}${offlineClass}" style="transform: rotate(${deg}deg); ${size}"></div>`,
        iconSize: [18, 18],
        iconAnchor: [9, 9],
      })

      const marker = L.marker(worldToLatLng(pos.x, pos.y), {
        icon,
        zIndexOffset: isSelected ? 900 : 500,
      }).addTo(group)

      const drivingLabel = robot.driving ? 'Driving' : 'Idle'
      marker.bindTooltip(
        `<b>${robot.id.split('/').pop()}</b><br/>${drivingLabel} — ${robot.connectionState}`,
        { direction: 'top', offset: [0, -10] }
      )
    }
  }, [fleetRobots, selectedRobot])

  // Ghost marker for replay (P2.4)
  const ghostMarkerRef = useRef<L.Marker | null>(null)
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    if (ghostPose) {
      const latlng = worldToLatLng(ghostPose.x, ghostPose.y)
      const deg = -(ghostPose.theta * 180 / Math.PI)
      const icon = L.divIcon({
        className: 'robot-marker',
        html: `<div class="robot-arrow" style="transform: rotate(${deg}deg); opacity: 0.5; border-bottom-color: #9c27b0;"></div>`,
        iconSize: [24, 24],
        iconAnchor: [12, 12],
      })

      if (ghostMarkerRef.current) {
        ghostMarkerRef.current.setLatLng(latlng)
        ghostMarkerRef.current.setIcon(icon)
      } else {
        ghostMarkerRef.current = L.marker(latlng, { icon, zIndexOffset: 800 }).addTo(map)
        ghostMarkerRef.current.bindTooltip('Replay', { direction: 'top', offset: [0, -10] })
      }
    } else if (ghostMarkerRef.current) {
      ghostMarkerRef.current.remove()
      ghostMarkerRef.current = null
    }
  }, [ghostPose])

  // Center on robot button
  const centerOnRobot = useCallback(() => {
    const map = mapRef.current
    if (!map) return
    userPannedRef.current = false
    setFollowRobot(true)
    map.panTo(worldToLatLng(pose.x, pose.y), { animate: true })
  }, [pose])

  return (
    <div className="map-container" style={{ position: 'relative' }}>
      <div ref={containerRef} className="map-leaflet" />

      {/* Overlays */}
      <div className="map-overlay-tl">
        <span className="map-coord">
          ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}) {(pose.theta * 180 / Math.PI).toFixed(0)}&deg;
        </span>
      </div>
      {mappingCoverage != null && mappingCoverage > 0 && (
        <div className="coverage-badge">
          Cobertura: {mappingCoverage.toFixed(1)}%
        </div>
      )}
      <div className="map-overlay-bl">
        <button
          className={`map-btn ${followRobot ? 'map-btn-active' : ''}`}
          onClick={centerOnRobot}
          title="Center on robot"
        >
          &#8853;
        </button>
      </div>
    </div>
  )
}
