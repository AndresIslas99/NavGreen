/**
 * Leaflet-based map view — replaces custom Canvas MapCanvas.
 *
 * Uses L.CRS.Simple for pixel-based indoor coordinates.
 * Layers: occupancy grid, scan points, nav path, pose trail, waypoints, robot pose.
 */

import { useRef, useEffect, useCallback, useState } from 'react'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'
import type { MapUpdate, PathPoint, DefinedTag } from '../api/types'
import { apiUrl } from '../api/client'
import type { FleetRobot } from '../hooks/useFleetSocket'

// Greenhouse rail aisle geometry. y-centers measured in world frame
// (meters), aisle half-width ≈ 0.35m (matches zone_detector params).
// X spans two segments: rear (x < GAP_MIN) and front (x > GAP_MAX);
// the gap between GAP_MIN..GAP_MAX is rail-free corridor.
const RAIL_AISLE_Y = [-4.4, -2.2, 0, 2.2, 4.4] as const
const RAIL_HALF_W = 0.35
const RAIL_X_MIN = -2
const RAIL_X_MAX = 13
const GAP_MIN = 3.5
const GAP_MAX = 7.5

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

// Robot icon: circular body + clear arrow head pointing in the heading direction.
// SVG default orientation: arrow tip at top (north). ROS theta=0 means facing
// X+ (east in our CRS.Simple → right). The conversion from ROS yaw to CSS
// rotation is the same as before: deg = -theta*180/π + 90.
function robotIcon(theta: number): L.DivIcon {
  const deg = -(theta * 180 / Math.PI) + 90
  const svg = `
    <svg viewBox="0 0 32 32" width="32" height="32"
         style="transform: rotate(${deg}deg); transform-origin: 16px 16px;">
      <!-- Body circle -->
      <circle cx="16" cy="16" r="9" fill="#1b5e20" stroke="#4caf50" stroke-width="2"/>
      <!-- Heading arrow: tip up, base inside circle -->
      <path d="M16,2 L24,15 L16,11 L8,15 Z"
            fill="#69f0ae" stroke="#fff" stroke-width="1" stroke-linejoin="round"/>
      <!-- Center dot for reference -->
      <circle cx="16" cy="16" r="1.5" fill="#fff"/>
    </svg>`
  return L.divIcon({
    className: 'robot-marker',
    html: svg,
    iconSize: [32, 32],
    iconAnchor: [16, 16],
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
  const railLayerRef = useRef<L.LayerGroup | null>(null)
  const tagLayerRef = useRef<L.LayerGroup | null>(null)

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

    // Rail aisle geometry: two rectangles per aisle centerline (rear + front),
    // skipping the gap. Drawn once on init — geometry is static.
    const railGroup = L.layerGroup().addTo(map)
    railLayerRef.current = railGroup
    for (const yc of RAIL_AISLE_Y) {
      const yLo = yc - RAIL_HALF_W
      const yHi = yc + RAIL_HALF_W
      // Rear segment (LatLngBoundsLiteral: [[lat,lng],[lat,lng]] where lat=y, lng=x)
      L.rectangle(
        [[yLo, RAIL_X_MIN], [yHi, GAP_MIN]],
        { color: '#4fc3f7', weight: 1, fillColor: '#4fc3f7', fillOpacity: 0.08, dashArray: '4,4', interactive: false },
      ).addTo(railGroup)
      // Front segment
      L.rectangle(
        [[yLo, GAP_MAX], [yHi, RAIL_X_MAX]],
        { color: '#4fc3f7', weight: 1, fillColor: '#4fc3f7', fillOpacity: 0.08, dashArray: '4,4', interactive: false },
      ).addTo(railGroup)
    }

    // AprilTag markers (rail_start). Tags loaded once via fetch below.
    const tagGroup = L.layerGroup().addTo(map)
    tagLayerRef.current = tagGroup

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

  // Fetch defined AprilTags and render them on the rail overlay. Polled
  // at a low rate (every 30s) so newly defined tags appear without reload.
  useEffect(() => {
    const group = tagLayerRef.current
    if (!group) return

    // Sub-fase 1.2.4 — Tag visualization with role-based color + orientation.
    //
    // The legacy DefinedTag schema only carries a binary `type` field
    // (wall|rail_start), so this function infers richer roles from the
    // label string emitted by the Tag Layout Loader's bulkImport:
    //   "rail_entry_*"            → blue (rail_entry)
    //   "charging*"               → amber (charging)
    //   "central_aisle_beacon*"   → green (beacon)
    //   "handoff*"                → purple (handoff)
    //   anything else / type=wall → gray (other)
    //
    // Orientation: a short line segment from the tag centre along the
    // (yaw) direction, helping the operator confirm tag rotation
    // matches the physical install. yaw is in radians on the
    // DefinedTag.
    const inferRole = (t: DefinedTag): string => {
      const lbl = (t.label || '').toLowerCase()
      if (t.type === 'rail_start' || lbl.startsWith('rail_entry')) return 'rail_entry'
      if (lbl.startsWith('charging')) return 'charging'
      if (lbl.startsWith('central_aisle_beacon')) return 'beacon'
      if (lbl.startsWith('handoff')) return 'handoff'
      return 'other'
    }
    const roleColor = (role: string): { stroke: string; fill: string } => {
      switch (role) {
        case 'rail_entry':   return { stroke: '#4fc3f7', fill: '#0277bd' }
        case 'charging':     return { stroke: '#ffd54f', fill: '#ffa000' }
        case 'beacon':       return { stroke: '#81c784', fill: '#388e3c' }
        case 'handoff':      return { stroke: '#ba68c8', fill: '#7b1fa2' }
        default:             return { stroke: '#b0bec5', fill: '#607d8b' }
      }
    }

    const render = (tags: DefinedTag[]) => {
      group.clearLayers()
      for (const t of tags) {
        const role = inferRole(t)
        const { stroke, fill } = roleColor(role)
        const center = worldToLatLng(t.x, t.y)

        // Tag circle marker.
        const marker = L.circleMarker(center, {
          radius: 6,
          color: stroke,
          fillColor: fill,
          fillOpacity: 0.9,
          weight: 2,
        }).addTo(group)

        // Orientation indicator: short line from centre along yaw.
        // Length in world units is small (~30 cm) to stay visually
        // local to the marker without overlapping neighbours.
        const len = 0.30
        const tipX = t.x + len * Math.cos(t.yaw)
        const tipY = t.y + len * Math.sin(t.yaw)
        L.polyline([center, worldToLatLng(tipX, tipY)], {
          color: stroke,
          weight: 2,
          opacity: 0.9,
          interactive: false,
        }).addTo(group)

        const yawDeg = (t.yaw * 180 / Math.PI).toFixed(0)
        marker.bindTooltip(`#${t.id} · ${t.label} (${role})  yaw=${yawDeg}°`, {
          direction: 'top',
          offset: [0, -6],
          className: 'apriltag-tooltip',
        })
      }
    }

    let canceled = false
    const fetchTags = () => {
      fetch(apiUrl('/api/apriltags'))
        .then(r => r.json())
        .then(s => { if (!canceled) render(s.defined_tags || []) })
        .catch(() => {})
    }
    fetchTags()
    const iv = setInterval(fetchTags, 30000)
    return () => { canceled = true; clearInterval(iv) }
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
  // Track which type of map is displayed to detect switches (static↔live)
  const currentMapType = useRef<'static' | 'live' | null>(null)
  useEffect(() => {
    const map = mapRef.current
    if (!map || !mapData?.png_base64) return

    const { width, height, resolution, origin_x, origin_y } = mapData
    const southWest = worldToLatLng(origin_x, origin_y)
    const northEast = worldToLatLng(origin_x + width * resolution, origin_y + height * resolution)
    const bounds = L.latLngBounds(southWest, northEast)

    const mapType = resolution < 0.04 ? 'live' : 'static'
    const typeChanged = currentMapType.current !== null && currentMapType.current !== mapType
    currentMapType.current = mapType

    const imageUrl = `data:image/png;base64,${mapData.png_base64}`

    // If map type switched (static↔live), destroy old overlay to avoid stale bounds/image
    if (typeChanged && imageLayerRef.current) {
      imageLayerRef.current.remove()
      imageLayerRef.current = null
    }

    if (imageLayerRef.current) {
      imageLayerRef.current.setBounds(bounds)
      imageLayerRef.current.setUrl(imageUrl)
    } else {
      const overlay = L.imageOverlay(imageUrl, bounds, { opacity: 0.9 }).addTo(map)
      imageLayerRef.current = overlay

      if (mapType === 'live') {
        map.setView(worldToLatLng(pose.x, pose.y), 3)
      } else {
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
