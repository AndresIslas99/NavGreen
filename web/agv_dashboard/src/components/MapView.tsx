/**
 * Leaflet-based map view — replaces custom Canvas MapCanvas.
 *
 * Uses L.CRS.Simple for pixel-based indoor coordinates.
 * Layers: occupancy grid, scan points, nav path, pose trail, waypoints, robot pose.
 */

import { useRef, useEffect, useState } from 'react'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'
import type { MapUpdate, PathPoint, DefinedTag, RailEntry, SemanticZone, RobotState, HomePoint } from '../api/types'
import { robotIcon } from './map/RobotIcon'
import { useCameraFollow } from './map/useCameraFollow'
import { RecenterButton } from './map/RecenterButton'
import { OffScreenIndicator } from './map/OffScreenIndicator'
import { LocateOff, Compass } from './ui/icons'
import { apiUrl } from '../api/client'
import type { FleetRobot } from '../hooks/useFleetSocket'
import {
  enclosureBounds,
  corridorBounds,
  approachStrips,
  rowBands,
  ghostRowBands,
  AISLE_CENTERS,
} from './map/greenhouseGeometry'

// Rail aisle geometry is now data-driven via GET /api/rails (backed by
// agv_rail_approach/list_rail_starts). The hardcoded RAIL_AISLE_Y array
// previously here was removed in Block C (specs/persistence.yaml +
// /api/rails) — it violated the "no hardcoded physical parameters" rule
// and forced manual edits whenever the greenhouse layout changed.
//
// Named semantic zones (BASE DE CARGA, ZONA DE TRABAJO A, ESTACIONAMIENTO)
// come from GET /api/zones (backed by ${AGV_DATA_DIR}/zones.yaml). The
// frontend draws polygons + labels; the backend is the SSOT.

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
  /** Robot state for icon coloring (accent/warn/crit). Optional — defaults to 'idle'. */
  state?: RobotState
  /** Operator-defined base/dock pose — rendered as a pulsing home landmark. */
  homePoint?: HomePoint | null
}

// Robot icon factory moved to './map/RobotIcon.tsx' — top-down vehicle outline
// with 4 wheels + heading wedge + state-aware coloring (accent / warn / crit).

// Convert world coords to Leaflet LatLng (y=lat, x=lng in CRS.Simple)
function worldToLatLng(x: number, y: number): L.LatLng {
  return L.latLng(y, x)
}

export function MapView({ mapData, pose, path, scanPoints, mode, onGoalClick, waypoints, fleetRobots, selectedRobot, ghostPose, mappingCoverage, state, homePoint }: Props) {
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
  const zoneLayerRef = useRef<L.LayerGroup | null>(null)
  // Greenhouse "place" layer — static structural geometry (enclosure outline,
  // drivable corridor, approach strips). Built once on map init from the
  // constants in greenhouseGeometry.ts.
  const greenhouseLayerRef = useRef<L.LayerGroup | null>(null)
  // Row band layer — re-rendered when /api/rails returns data.
  const rowBandLayerRef = useRef<L.LayerGroup | null>(null)
  // Map of letter+section → rectangle so M3 can flip active-row opacity.
  const rowRectsRef = useRef<Map<string, L.Rectangle>>(new Map())
  // Home / base landmark — pulses gently when defined.
  const homeMarkerRef = useRef<L.Marker | null>(null)

  // Trail accumulator
  const trailRef = useRef<L.LatLng[]>([])

  // State mirror of mapRef so the camera hook re-runs when the map exists.
  // (Refs don't trigger re-renders; the hook needs the live map instance.)
  const [mapInstance, setMapInstance] = useState<L.Map | null>(null)

  // Camera follow logic — owns the "always centered on robot" behavior with
  // smooth panTo animation, manual-pan detection (via movestart guarded by
  // programmaticMoveRef), and stale-pose freezing. The state machine
  // (follow|manual|frozen) drives the visibility of RecenterButton and
  // OffScreenIndicator below.
  const { cameraMode, recenter } = useCameraFollow(
    mapInstance,
    pose,
    worldToLatLng,
    { defaultZoom: 4, bottomBias: 0.20 },
  )

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

    // Initial view delegated to useCameraFollow: it sets a tighter zoom (4)
    // centered on the robot pose so the operator sees ~10-15 m around the
    // vehicle, Google-Maps style. If no pose has been received yet the hook
    // is a no-op and we fall back to default Leaflet zoom (handled below).
    map.setView([0, 0], 4, { animate: false })

    // Add zoom control in top-right
    L.control.zoom({ position: 'topright' }).addTo(map)

    // ── Greenhouse "place" layer (bottommost) ──
    // Static structural geometry built once from the greenhouseGeometry constants.
    // Enclosure outline + drivable corridor stripe + AprilTag approach strips.
    // Row bands (the green-tinted "crop row" rectangles) come later from the
    // rails fetch — see the rowBandLayer effect below.
    const greenhouseGroup = L.layerGroup().addTo(map)
    greenhouseLayerRef.current = greenhouseGroup
    {
      // Enclosure: subtle dashed outline + slightly darker cream fill so the
      // cultivation area reads as a defined space, not a void.
      const enc = enclosureBounds()
      L.rectangle(
        [[enc.minY, enc.minX], [enc.maxY, enc.maxX]],
        {
          color: '#d8d2c5',
          weight: 1.5,
          dashArray: '6,4',
          fillColor: '#efece5',   // = --surface-2 literal (Leaflet can't read CSS vars)
          fillOpacity: 1.0,
          interactive: false,
        },
      ).addTo(greenhouseGroup)

      // Drivable corridor — slightly lighter than the enclosure to suggest
      // "drive through here". Reads as the central passable lane.
      const cor = corridorBounds()
      L.rectangle(
        [[cor.minY, cor.minX], [cor.maxY, cor.maxX]],
        {
          color: 'transparent',
          weight: 0,
          fillColor: '#fefdfb',   // = --surface
          fillOpacity: 0.65,
          interactive: false,
        },
      ).addTo(greenhouseGroup)

      // Dashed centerline through the corridor (a visual hint of "drive lane").
      const corCenterX = (cor.minX + cor.maxX) / 2
      L.polyline(
        [[cor.minY, corCenterX], [cor.maxY, corCenterX]],
        {
          color: '#c5beae',
          weight: 1,
          dashArray: '2,12',
          interactive: false,
        },
      ).addTo(greenhouseGroup)

      // AprilTag approach strips (where rail_approach hands off from Nav2).
      // Faint warm-tan wash so operators see "this is the precision zone".
      const aps = approachStrips()
      for (const strip of [aps.rear, aps.front]) {
        L.rectangle(
          [[strip.minY, strip.minX], [strip.maxY, strip.maxX]],
          {
            color: 'transparent',
            weight: 0,
            fillColor: '#d4a373',   // = --amber
            fillOpacity: 0.16,
            interactive: false,
          },
        ).addTo(greenhouseGroup)
      }

      // Aisle centerline guides — very subtle dashed lines along each aisle
      // y-center, spanning the full enclosure. Helps operator align mentally
      // even before any rails are registered.
      for (const yc of AISLE_CENTERS) {
        L.polyline(
          [[yc, enc.minX], [yc, enc.maxX]],
          {
            color: '#c1d9b6',     // = --accent-soft-strong
            weight: 0.5,
            dashArray: '1,8',
            opacity: 0.65,
            interactive: false,
          },
        ).addTo(greenhouseGroup)
      }
    }

    // Row band layer — populated from /api/rails fetch (M2 effect below).
    const rowBandGroup = L.layerGroup().addTo(map)
    rowBandLayerRef.current = rowBandGroup

    // Layer groups
    const scanGroup = L.layerGroup().addTo(map)
    scanGroupRef.current = scanGroup

    const waypointGroup = L.layerGroup().addTo(map)
    waypointLayerRef.current = waypointGroup

    // Rail label overlay group. Filled by the /api/rails fetch effect below.
    // Data-driven (replaces the hardcoded RAIL_AISLE_Y / GAP_* constants).
    const railGroup = L.layerGroup().addTo(map)
    railLayerRef.current = railGroup

    // Semantic zone polygons (BASE / ZONA TRABAJO / ESTACIONAMIENTO).
    // Filled by the /api/zones fetch effect below.
    const zoneGroup = L.layerGroup().addTo(map)
    zoneLayerRef.current = zoneGroup

    // AprilTag markers (rail_start). Tags loaded once via fetch below.
    const tagGroup = L.layerGroup().addTo(map)
    tagLayerRef.current = tagGroup

    // User-pan detection is now owned by useCameraFollow (uses movestart +
    // zoomstart guarded by programmaticMoveRef). No dragstart listener here.

    // Click-to-goal
    map.on('click', (e: L.LeafletMouseEvent) => {
      if (mode === 'nav' && onGoalClick) {
        onGoalClick(e.latlng.lng, e.latlng.lat)
      }
    })

    mapRef.current = map
    setMapInstance(map)   // notify React-tree consumers (e.g. useCameraFollow)

    return () => {
      map.remove()
      mapRef.current = null
      setMapInstance(null)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // Fetch defined AprilTags and render them on the rail overlay. Polled
  // at a low rate (every 30s) so newly defined tags appear without reload.
  useEffect(() => {
    const group = tagLayerRef.current
    if (!group) return

    const render = (tags: DefinedTag[]) => {
      group.clearLayers()
      for (const t of tags) {
        // Type-distinct glyphs:
        //  - rail_start → forest-green diamond (suggests "approach target")
        //  - wall       → muted-grey square   (suggests "fixed reference")
        const isRail = t.type === 'rail_start'
        const glyphSvg = isRail
          ? `<svg width="14" height="14" viewBox="0 0 14 14">
               <rect x="3" y="3" width="8" height="8" rx="1.2"
                     transform="rotate(45 7 7)"
                     fill="#e2eedc" stroke="#2f6f2a" stroke-width="1.4"/>
               <path d="M5 7 L9 7" stroke="#2f6f2a" stroke-width="1.2" stroke-linecap="round"/>
             </svg>`
          : `<svg width="12" height="12" viewBox="0 0 12 12">
               <rect x="1.5" y="1.5" width="9" height="9" rx="1"
                     fill="#efece5" stroke="#7a847c" stroke-width="1.2"/>
             </svg>`
        const marker = L.marker(worldToLatLng(t.x, t.y), {
          icon: L.divIcon({
            className: `apriltag-marker apriltag-marker--${isRail ? 'rail' : 'wall'}`,
            html: glyphSvg,
            iconSize: isRail ? [14, 14] : [12, 12],
            iconAnchor: isRail ? [7, 7] : [6, 6],
          }),
          interactive: true,
        }).addTo(group)
        marker.bindTooltip(`#${t.id} · ${t.label}${isRail ? ' (rail)' : ''}`, {
          direction: 'top',
          offset: [0, -4],
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

  // Semantic zones overlay — fetches GET /api/zones (backed by zones.yaml)
  // and paints labeled polygons over the map. Graceful degradation: an
  // empty/missing zones.yaml on the backend returns {zones: []} and we
  // simply render nothing. Poll mirrors AprilTags (30 s).
  useEffect(() => {
    const group = zoneLayerRef.current
    if (!group) return

    const render = (zones: SemanticZone[]) => {
      group.clearLayers()
      for (const z of zones) {
        if (!z.polygon || z.polygon.length < 3) continue
        // Leaflet polygon expects [lat, lng] = [y, x]. worldToLatLng()
        // is the single source of truth for this swap.
        const latlngs = z.polygon.map(p => worldToLatLng(p.x, p.y))
        const poly = L.polygon(latlngs, {
          color: z.color || '#7a9d8e',
          fillColor: z.color || '#7a9d8e',
          fillOpacity: 0.14,
          weight: 1.5,
          interactive: false,
        }).addTo(group)
        // Permanent tooltip = label rendered directly on the map.
        poly.bindTooltip(z.label, {
          permanent: true,
          direction: 'center',
          className: 'zone-label-tooltip',
        })
      }
    }

    let canceled = false
    const fetchZones = () => {
      fetch(apiUrl('/api/zones'))
        .then(r => r.json())
        .then(s => { if (!canceled) render(s.zones || []) })
        .catch(() => {})
    }
    fetchZones()
    const iv = setInterval(fetchZones, 30000)
    return () => { canceled = true; clearInterval(iv) }
  }, [])

  // Rail label overlay — replaces the hardcoded RAIL_AISLE_Y constants
  // with a data-driven fetch of /api/rails (which proxies
  // agv_rail_approach/list_rail_starts). Each rail entry renders as a
  // small dot + "RIEL A", "RIEL B", … label so operators can quickly
  // identify which aisle the robot is in. Falls back to no labels when
  // the rail_approach service is unavailable.
  useEffect(() => {
    const railGroup = railLayerRef.current
    const bandGroup = rowBandLayerRef.current
    if (!railGroup || !bandGroup) return

    const render = (rails: RailEntry[]) => {
      // 1. Rail entry markers (small dot + tooltip).
      railGroup.clearLayers()
      for (const r of rails) {
        const ll = worldToLatLng(r.x, r.y)
        const dot = L.circleMarker(ll, {
          radius: 4,
          color: '#2f6f2a',       // = --accent (clearer rail-entry marker)
          fillColor: '#e2eedc',   // = --accent-soft
          fillOpacity: 0.9,
          weight: 1.5,
          interactive: false,
        }).addTo(railGroup)
        dot.bindTooltip(r.label, {
          permanent: true,
          direction: 'right',
          offset: [8, 0],
          className: 'rail-label-tooltip',
        })
      }

      // 2. Row bands — the green-tinted "crop row" rectangles spanning each
      // rail's length. This is the layer that makes the map read as a
      // greenhouse, not a coordinate plane.
      //
      // Behavior:
      //  - When the rail registry has entries: paint solid bands per rail.
      //  - When the registry is empty (e.g. on a freshly installed system,
      //    or in dev): paint GHOST bands at every possible aisle×section
      //    position so the operator still sees the greenhouse skeleton with
      //    a clear "not yet registered" visual hint (dashed border, lower
      //    opacity, suffix "(sin riel)").
      bandGroup.clearLayers()
      rowRectsRef.current.clear()
      const bands = rowBands(rails)
      const useGhost = bands.length === 0
      const bandsToRender = useGhost ? ghostRowBands() : bands
      for (const b of bandsToRender) {
        const rect = L.rectangle(
          [[b.yMin, b.xStart], [b.yMax, b.xEnd]],
          {
            color: '#c1d9b6',     // = --accent-soft-strong
            weight: useGhost ? 1 : 1.2,
            dashArray: useGhost ? '4,4' : undefined,
            fillColor: '#e2eedc', // = --accent-soft
            fillOpacity: useGhost ? 0.30 : 0.55,
            interactive: false,
          },
        ).addTo(bandGroup)
        const labelText = useGhost ? `${b.label} (sin riel)` : b.label
        rect.bindTooltip(labelText, {
          permanent: true,
          direction: 'center',
          className: useGhost ? 'row-band-label row-band-label--ghost' : 'row-band-label',
        })
        // Key by letter+section so the M3 active-row effect can flip opacity
        // and tooltip class for the specific band the robot occupies.
        rowRectsRef.current.set(`${b.letter}-${b.section}`, rect)
      }
    }

    let canceled = false
    const fetchRails = () => {
      fetch(apiUrl('/api/rails'))
        .then(r => r.json())
        .then(rails => { if (!canceled) render(Array.isArray(rails) ? rails : []) })
        .catch(() => {})
    }
    fetchRails()
    const iv = setInterval(fetchRails, 30000)
    return () => { canceled = true; clearInterval(iv) }
  }, [])

  // Home / base landmark — pulsing house glyph at home_point pose.
  // Visible only when an operator has set a home point. Clicking it opens
  // a tooltip showing the name; the actual "Ir a base" action lives in the
  // cockpit ActionStack.
  useEffect(() => {
    const map = mapRef.current
    if (!map) return
    // Tear down old marker on change/removal
    if (homeMarkerRef.current) {
      homeMarkerRef.current.remove()
      homeMarkerRef.current = null
    }
    if (!homePoint) return
    const svg = `
      <div class="home-landmark">
        <div class="home-landmark__pulse"></div>
        <div class="home-landmark__core">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none"
               stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M3 12 L12 4 L21 12"/>
            <path d="M5 10 V20 H10 V14 H14 V20 H19 V10"/>
          </svg>
        </div>
      </div>`
    const marker = L.marker(worldToLatLng(homePoint.x, homePoint.y), {
      icon: L.divIcon({
        className: 'home-landmark-wrapper',
        html: svg,
        iconSize: [40, 40],
        iconAnchor: [20, 20],
      }),
      zIndexOffset: 600,
      interactive: true,
    }).addTo(map)
    marker.bindTooltip(`Base: ${homePoint.name}`, {
      permanent: true,
      direction: 'bottom',
      offset: [0, 14],
      className: 'home-landmark-tooltip',
    })
    homeMarkerRef.current = marker
  }, [homePoint])

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
      // No explicit setView/fitBounds here — initial centering is owned by
      // useCameraFollow (it runs setView once on first pose receipt). Calling
      // setView/fitBounds here would fire movestart outside the hook's grace
      // window and flip followRobot=false the moment the SLAM map first
      // arrives, breaking the always-centered Google-Maps experience.
    }
  }, [mapData])

  // Update robot position
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    const latlng = worldToLatLng(pose.x, pose.y)

    if (robotMarkerRef.current) {
      robotMarkerRef.current.setLatLng(latlng)
      robotMarkerRef.current.setIcon(robotIcon(pose.theta, state))
    } else {
      const marker = L.marker(latlng, {
        icon: robotIcon(pose.theta, state),
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

    // Follow logic moved to useCameraFollow hook (smooth animated panTo).
  }, [pose, state])

  // Update navigation path (state-aware coloring + animated dashes via CSS).
  // Default: accent green dashed line flowing toward the goal.
  // Blocked: warm tan dashed, no animation.
  // E-stop/fault: crit red solid, no animation (stop signal).
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    const latlngs = path.map(p => worldToLatLng(p.x, p.y))

    const isCrit = state === 'e_stop' || state === 'fault'
    const isWarn = state === 'blocked'
    const color = isCrit ? '#a8392a' : isWarn ? '#b8612e' : '#2f6f2a'
    const dashArray = isCrit ? undefined : '8, 6'
    const className = (isCrit || isWarn) ? 'nav-path' : 'nav-path nav-path--animated'

    if (pathLayerRef.current) {
      pathLayerRef.current.setLatLngs(latlngs)
      pathLayerRef.current.setStyle({
        color, dashArray, opacity: latlngs.length > 0 ? 0.92 : 0,
      })
      ;(pathLayerRef.current.options as any).className = className
    } else if (latlngs.length > 0) {
      pathLayerRef.current = L.polyline(latlngs, {
        color,
        weight: 3,
        opacity: 0.92,
        dashArray,
        className,
      }).addTo(map)
    }

    if (latlngs.length === 0 && pathLayerRef.current) {
      pathLayerRef.current.setLatLngs([])
    }
  }, [path, state])

  // Update scan points — softened from aggressive red dots to a calm
  // muted-grey dust. The point cloud is informational ("here's what the
  // lidar sees"), not an alarm. Decimated to ~200 max visible markers
  // for render perf on the Jetson.
  useEffect(() => {
    const group = scanGroupRef.current
    if (!group) return

    group.clearLayers()
    const step = Math.max(1, Math.floor(scanPoints.length / 200))
    for (let i = 0; i < scanPoints.length; i += step) {
      const pt = scanPoints[i]
      L.circleMarker(worldToLatLng(pt.x, pt.y), {
        radius: 1.4,
        color: '#7a847c',       // = --dim
        fillColor: '#7a847c',
        fillOpacity: 0.5,
        opacity: 0.4,
        weight: 0,
        interactive: false,
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

  // Center-on-robot is now owned by the useCameraFollow hook (`recenter`),
  // surfaced through the floating <RecenterButton> FAB rendered below.
  // The previous inline ⊕ button in `.map-overlay-bl` was removed — it
  // duplicated the FAB and was easy to miss.

  return (
    <div className="map-container" style={{ position: 'relative' }}>
      <div ref={containerRef} className="map-leaflet" />

      {/* Pose coords (small dim chip, always visible top-left). */}
      <div className="map-overlay-tl">
        <span className="map-coord">
          ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}) {(pose.theta * 180 / Math.PI).toFixed(0)}&deg;
        </span>
      </div>

      {/* Camera status pill — centered top of the map. Tells the operator
          why the view is no longer auto-following (manual pan vs. stale pose). */}
      {cameraMode !== 'follow' && (
        <div className="camera-status-pill-wrap">
          <span
            className={`camera-status-pill camera-status-pill--${cameraMode}`}
            title={
              cameraMode === 'manual'
                ? 'Toca el botón Centrar para volver al robot'
                : 'Sin actualización de pose por más de 2 s'
            }
          >
            {cameraMode === 'manual' ? (
              <>
                <LocateOff size={12} strokeWidth={2.2} aria-hidden />
                <span>Vista manual</span>
              </>
            ) : (
              <>
                <Compass size={12} strokeWidth={2.2} aria-hidden />
                <span>Vista congelada</span>
              </>
            )}
          </span>
        </div>
      )}

      {mappingCoverage != null && mappingCoverage > 0 && (
        <div className="coverage-badge">
          Cobertura: {mappingCoverage.toFixed(1)}%
        </div>
      )}

      {/* Floating "back to robot" FAB. Auto-hidden in follow mode. */}
      <RecenterButton cameraMode={cameraMode} onRecenter={recenter} />

      {/* Edge chevron when the robot is off-screen. */}
      <OffScreenIndicator
        map={mapInstance}
        pose={pose}
        cameraMode={cameraMode}
        worldToLatLng={worldToLatLng}
        onRecenter={recenter}
      />
    </div>
  )
}
