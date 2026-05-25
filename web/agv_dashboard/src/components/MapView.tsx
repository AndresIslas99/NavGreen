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
import { CompassScale } from './map/CompassScale'
import { LocateOff, Compass } from './ui/icons'
import { apiUrl } from '../api/client'
import { useToast } from './ui/Toast'
import type { FleetRobot } from '../hooks/useFleetSocket'
import {
  enclosureBounds,
  corridorBounds,
  approachStrips,
  rowBands,
  ghostRowBands,
  activeRowBand,
  AISLE_CENTERS,
  type RowBand,
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
  /** Battery percent — triggers the low-battery blinking dot on the robot icon when < 15. */
  batteryPct?: number | null
}

// Robot icon factory moved to './map/RobotIcon.tsx' — top-down vehicle outline
// with 4 wheels + heading wedge + state-aware coloring (accent / warn / crit).

// Convert world coords to Leaflet LatLng (y=lat, x=lng in CRS.Simple)
function worldToLatLng(x: number, y: number): L.LatLng {
  return L.latLng(y, x)
}

export function MapView({ mapData, pose, path, scanPoints, mode, onGoalClick, waypoints, fleetRobots, selectedRobot, ghostPose, mappingCoverage, state, homePoint, batteryPct }: Props) {
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
  // Map of letter+section → rectangle so the spotlight effect can flip
  // active-row opacity. `rowBandsRef` mirrors the geometric definition of
  // each rendered band so the spotlight effect can also lookup the
  // operator-facing label without re-deriving it.
  const rowRectsRef = useRef<Map<string, L.Rectangle>>(new Map())
  const rowBandsRef = useRef<RowBand[]>([])
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

    // No Leaflet zoom control — it sat at z=1000 in the top-right and
    // crossed the translucent topbar's edge. The operator zooms via
    // pinch on touch, wheel on desktop, or the recenter FAB. The compass
    // chip handles "what scale am I at" feedback.

    // ── Greenhouse "place" layer (bottommost) ──
    // Static structural geometry built once from the greenhouseGeometry constants.
    // Enclosure outline + drivable corridor stripe + AprilTag approach strips.
    // Row bands (the green-tinted "crop row" rectangles) come later from the
    // rails fetch — see the rowBandLayer effect below.
    const greenhouseGroup = L.layerGroup().addTo(map)
    greenhouseLayerRef.current = greenhouseGroup
    {
      // Enclosure — built up in three concentric rectangles to read as
      // physical greenhouse walls on top of a continuing satellite-style
      // earth wash:
      //   1. Outer halo: a slightly larger dark band beyond the walls
      //      that gives the enclosure depth/elevation against the soil.
      //   2. Wall ring: solid border, slightly thicker; this is "the glass".
      //   3. Interior fill: brighter than the outside wash so it reads as
      //      a lit, structured space (skylight on the roof, cement floor).
      //   4. Inner trim: a thinner border 0.25 m inside the wall ring,
      //      suggesting the double-pane / steel frame of polycarbonate
      //      greenhouse construction. Pure visual detail, no functional
      //      meaning.
      const enc = enclosureBounds()
      const halo = 1.4    // metres of "soil shadow" beyond the walls
      const trim = 0.30   // metres inset of the inner trim line

      // 1. Outer halo — slightly darker soil ring extending past the walls.
      L.rectangle(
        [[enc.minY - halo, enc.minX - halo], [enc.maxY + halo, enc.maxX + halo]],
        {
          color: 'transparent',
          weight: 0,
          fillColor: '#9c9587',
          fillOpacity: 0.10,
          interactive: false,
        },
      ).addTo(greenhouseGroup)

      // 2. + 3. Enclosure wall ring + interior fill, as a single rectangle.
      L.rectangle(
        [[enc.minY, enc.minX], [enc.maxY, enc.maxX]],
        {
          color: '#b3a98f',           // warm tan-grey, reads as anodised frame
          weight: 2.4,
          fillColor: '#fbf8f0',       // brighter than outside wash → "interior"
          fillOpacity: 0.96,
          interactive: false,
        },
      ).addTo(greenhouseGroup)

      // 4. Inner trim — second-pane / frame, 0.30 m inside the main wall.
      L.rectangle(
        [[enc.minY + trim, enc.minX + trim], [enc.maxY - trim, enc.maxX - trim]],
        {
          color: '#c9bea0',
          weight: 0.8,
          fill: false,
          dashArray: '4,4',
          interactive: false,
        },
      ).addTo(greenhouseGroup)

      // 5. Skylight ridge — real polycarbonate greenhouses have a peaked
      // roof with a continuous skylight along the long axis. We render
      // this as a soft warm gradient strip along y=0 (the building's long
      // axis). It's the most "this place is lit by the sun" detail and
      // sells the indoor-but-bright feel.
      L.rectangle(
        [[-0.45, enc.minX + 0.6], [0.45, enc.maxX - 0.6]],
        {
          color: 'transparent',
          weight: 0,
          fillColor: '#fff8e2',
          fillOpacity: 0.55,
          interactive: false,
        },
      ).addTo(greenhouseGroup)
      // Skylight centerline — thin tinted seam representing the ridge cap.
      L.polyline(
        [[0, enc.minX + 0.6], [0, enc.maxX - 0.6]],
        {
          color: '#e8c875',
          weight: 0.6,
          opacity: 0.6,
          interactive: false,
        },
      ).addTo(greenhouseGroup)

      // 6. Corner posts — small filled circles at each of the 4 corners
      // suggesting steel uprights. Pure visual detail.
      for (const [py, px] of [
        [enc.minY, enc.minX], [enc.minY, enc.maxX],
        [enc.maxY, enc.minX], [enc.maxY, enc.maxX],
      ] as [number, number][]) {
        L.circleMarker([py, px], {
          radius: 3.2,
          color: '#7c6e4f',
          fillColor: '#a8997a',
          fillOpacity: 1,
          weight: 1.2,
          interactive: false,
        }).addTo(greenhouseGroup)
      }

      // 7. Door — a 1.2 m opening on the FRONT-east wall (x = enc.maxX),
      // centered on y=0. The wall already passes through this point; we
      // add a brighter accent gap + two small "door frame" posts and a
      // short label that reads "Acceso" as a tooltip. Operators learn
      // "the robot enters/exits here".
      const doorHalf = 0.6
      // The gap itself — a brighter accent-soft sliver on the wall.
      L.polyline(
        [[-doorHalf, enc.maxX], [doorHalf, enc.maxX]],
        {
          color: '#fef9e8',
          weight: 4,
          opacity: 1,
          lineCap: 'butt',
          interactive: false,
        },
      ).addTo(greenhouseGroup)
      // Door frame posts (top + bottom of the opening).
      for (const dy of [-doorHalf, doorHalf]) {
        L.circleMarker([dy, enc.maxX], {
          radius: 2.6,
          color: '#7c6e4f',
          fillColor: '#d4a373',
          fillOpacity: 1,
          weight: 1.1,
          interactive: false,
        }).addTo(greenhouseGroup)
      }
      // Door label — small permanent tooltip just outside the wall.
      L.marker([0, enc.maxX + 0.5], {
        icon: L.divIcon({
          className: 'greenhouse-door-label',
          html: '<span>Acceso</span>',
          iconSize: [44, 14],
          iconAnchor: [0, 7],
        }),
        interactive: false,
      }).addTo(greenhouseGroup)

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

      // ── Rural context: features outside the active greenhouse ──
      // Visible when the operator zooms out for an overview. Faint enough
      // to never compete with the live operating area, narrative enough
      // that the map reads as "this is a real place in the world".

      // Distant neighbour greenhouse to the west — same long-axis
      // orientation, slightly smaller. Dashed outline + very faint
      // interior. Reads as another building you'd see from a drone shot.
      const distMinX = enc.minX - 24   // 24 m west of our west wall
      const distMaxX = enc.minX - 6    // 6 m gap between the two
      const distMinY = enc.minY + 1.0
      const distMaxY = enc.maxY - 1.0
      L.rectangle(
        [[distMinY, distMinX], [distMaxY, distMaxX]],
        {
          color: '#a8997a',
          weight: 1,
          dashArray: '3,5',
          opacity: 0.55,
          fillColor: '#e8dfc9',
          fillOpacity: 0.45,
          interactive: false,
        },
      ).addTo(greenhouseGroup)
      // Distant greenhouse skylight hint — a soft warm strip.
      L.rectangle(
        [[(distMinY + distMaxY) / 2 - 0.35, distMinX + 0.4], [(distMinY + distMaxY) / 2 + 0.35, distMaxX - 0.4]],
        {
          color: 'transparent',
          weight: 0,
          fillColor: '#fff5d8',
          fillOpacity: 0.35,
          interactive: false,
        },
      ).addTo(greenhouseGroup)
      // Tiny label for the neighbour.
      L.marker([(distMinY + distMaxY) / 2 - 2.2, (distMinX + distMaxX) / 2], {
        icon: L.divIcon({
          className: 'rural-distant-label',
          html: '<span>Invernadero B</span>',
          iconSize: [80, 12],
          iconAnchor: [40, 6],
        }),
        interactive: false,
      }).addTo(greenhouseGroup)

      // Dirt road — starts at our greenhouse's "Acceso" (x=enc.maxX, y=0)
      // and curves east-then-south toward the field gate. Tan dashed
      // double-line to look like compacted dirt with wheel ruts.
      const roadPts: [number, number][] = [
        [0, enc.maxX + 0.6],
        [-0.6, enc.maxX + 3],
        [-1.8, enc.maxX + 7],
        [-3.4, enc.maxX + 11],
        [-4.6, enc.maxX + 15],
        [-5.4, enc.maxX + 19],
      ]
      // Wider lighter underline = the road bed.
      L.polyline(roadPts, {
        color: '#d4c19e',
        weight: 6,
        opacity: 0.55,
        lineCap: 'round',
        lineJoin: 'round',
        interactive: false,
      }).addTo(greenhouseGroup)
      // Two thin dashed wheel-rut lines on top.
      L.polyline(roadPts, {
        color: '#a8997a',
        weight: 0.8,
        dashArray: '6,4',
        opacity: 0.7,
        lineCap: 'round',
        interactive: false,
      }).addTo(greenhouseGroup)

      // Water tank — small filled circle northeast of the greenhouse,
      // next to the "main road". Slate-blue suggests stored water.
      const tankX = enc.maxX + 9
      const tankY = enc.maxY + 4
      L.circleMarker([tankY, tankX], {
        radius: 8,
        color: '#5b6e7e',
        fillColor: '#9bb0c2',
        fillOpacity: 0.6,
        weight: 1.4,
        interactive: false,
      }).addTo(greenhouseGroup)
      // Inner ring suggests a top-down view of a cylindrical tank.
      L.circleMarker([tankY, tankX], {
        radius: 5,
        color: '#5b6e7e',
        fill: false,
        weight: 0.8,
        opacity: 0.7,
        interactive: false,
      }).addTo(greenhouseGroup)
      // Label.
      L.marker([tankY - 2.0, tankX], {
        icon: L.divIcon({
          className: 'rural-distant-label',
          html: '<span>Cisterna</span>',
          iconSize: [60, 12],
          iconAnchor: [30, 6],
        }),
        interactive: false,
      }).addTo(greenhouseGroup)

      // Faint field plots far south — three small dashed rectangles
      // suggesting cultivated land beyond the greenhouse. Pure scenery.
      for (let i = 0; i < 3; i++) {
        const px = enc.minX + 4 + i * 12
        const py = enc.minY - 8
        L.rectangle(
          [[py - 1.8, px], [py, px + 9]],
          {
            color: '#a08a64',
            weight: 0.6,
            dashArray: '2,4',
            opacity: 0.45,
            fillColor: '#d8c8a0',
            fillOpacity: 0.20,
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
      // Mirror into ref so the proximity-ripple effect below can read it
      // without re-rendering on every poll.
      definedTagsRef.current = tags
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
      rowBandsRef.current = bandsToRender
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
        // Key by letter+section so the spotlight effect can flip opacity
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

  // Active-row spotlight — whenever the robot pose changes, find the band
  // it currently occupies (if any) and brighten that one while dimming
  // the rest. Turns the row layout into a narrator: "the robot is here,
  // working this row". Re-runs cheaply: just iterates ≤10 rectangles and
  // mutates Leaflet styles + tooltip classes.
  useEffect(() => {
    const bands = rowBandsRef.current
    if (!bands.length) return
    const active = activeRowBand(bands, pose)
    const activeKey = active ? `${active.letter}-${active.section}` : null

    rowRectsRef.current.forEach((rect, key) => {
      const isActive = key === activeKey
      const isGhost = bands.find(b => `${b.letter}-${b.section}` === key && b.label.includes('Hilera')) == null
      // Pure visual style flips — opacity, color and weight only.
      rect.setStyle({
        fillColor: isActive ? '#a4d090' : (isGhost ? '#e2eedc' : '#e2eedc'),
        fillOpacity: isActive ? 0.78 : (activeKey ? 0.32 : (isGhost ? 0.30 : 0.55)),
        color: isActive ? '#2f6f2a' : '#c1d9b6',
        weight: isActive ? 2.2 : 1.2,
      })
      // Tooltip class flip so the chip also shifts to the active treatment.
      const tooltip = rect.getTooltip() as L.Tooltip | undefined
      if (tooltip) {
        const el = tooltip.getElement()
        if (el) {
          el.classList.toggle('row-band-label--active', isActive)
          // Dim the labels of non-active bands while one IS active, so
          // attention focuses on the row being worked. We don't dim if
          // nothing is active.
          el.classList.toggle('row-band-label--dimmed', !isActive && activeKey != null)
        }
      }
    })
  }, [pose])

  // AprilTag ripple — when the robot pose comes within DETECTION_RADIUS of
  // a defined tag for the first time (since it left the radius), trigger
  // a brief expanding ring at the tag's position and a toast. This is a
  // proxy for "the camera just saw this tag" without requiring the backend
  // to publish a new event type — the heuristic is accurate enough at
  // typical greenhouse layouts where tags are placed on rail entries and
  // the robot reaches them deliberately.
  const definedTagsRef = useRef<DefinedTag[]>([])
  const recentlySeenTagsRef = useRef<Set<number>>(new Set())
  const toast = useToast()
  useEffect(() => {
    const tags = definedTagsRef.current
    const group = tagLayerRef.current
    if (!group || !tags.length) return
    const DETECTION_RADIUS = 2.0   // metres
    const recent = recentlySeenTagsRef.current
    for (const t of tags) {
      const dx = t.x - pose.x
      const dy = t.y - pose.y
      const d2 = dx * dx + dy * dy
      const inside = d2 < DETECTION_RADIUS * DETECTION_RADIUS
      if (inside && !recent.has(t.id)) {
        recent.add(t.id)
        // Brief expanding circle marker that dissolves after 800ms.
        const ring = L.circleMarker(worldToLatLng(t.x, t.y), {
          radius: 6,
          color: '#2f6f2a',
          fillColor: '#2f6f2a',
          fillOpacity: 0.35,
          weight: 2,
          interactive: false,
          className: 'apriltag-ripple',
        }).addTo(group)
        window.setTimeout(() => { try { group.removeLayer(ring) } catch {} }, 900)
        toast.push({
          tone: 'ok',
          title: `AprilTag ${t.label}`,
          description: `Etiqueta #${t.id} detectada — ${t.type === 'rail_start' ? 'entrada de riel' : 'referencia de pared'}`,
          durationMs: 2500,
        })
      } else if (!inside && recent.has(t.id)) {
        // Robot left the detection radius — re-arm so the next entry
        // ripples again.
        recent.delete(t.id)
      }
    }
  }, [pose, toast])

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

    // SLAM occupancy grid — rendered with `multiply` blend so white pixels
    // (free/unknown space) disappear into the greenhouse template and only
    // dark pixels (walls/obstacles) remain as subtle traces. When the
    // robot is NOT in mapping mode we further hide the raster entirely
    // (the greenhouse layer carries the visual), which the SLAM toggle
    // effect below applies.
    if (imageLayerRef.current) {
      imageLayerRef.current.setBounds(bounds)
      imageLayerRef.current.setUrl(imageUrl)
    } else {
      const overlay = L.imageOverlay(imageUrl, bounds, {
        opacity: 0.45,
        className: 'slam-overlay',
      }).addTo(map)
      imageLayerRef.current = overlay
      // No explicit setView/fitBounds here — initial centering is owned by
      // useCameraFollow (it runs setView once on first pose receipt). Calling
      // setView/fitBounds here would fire movestart outside the hook's grace
      // window and flip followRobot=false the moment the SLAM map first
      // arrives, breaking the always-centered Google-Maps experience.
    }
  }, [mapData])

  // Toggle SLAM raster visibility based on robot state. When the operator
  // is just driving / observing / idling, the greenhouse template is the
  // visual; the SLAM PNG behind it is redundant noise. We keep it loaded
  // (so a mode-switch to mapping/nav is instant) but fade it out via the
  // `.slam-hidden` modifier class.
  useEffect(() => {
    const overlay = imageLayerRef.current
    if (!overlay) return
    const el = overlay.getElement()
    if (!el) return
    const showSlam = state === 'mapping' || state === 'navigating'
    el.classList.toggle('slam-hidden', !showSlam)
  }, [state, mapData])

  // Update robot position
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    const latlng = worldToLatLng(pose.x, pose.y)

    const lowBattery = batteryPct != null && batteryPct >= 0 && batteryPct < 15
    const iconOpts = { lowBattery }

    if (robotMarkerRef.current) {
      robotMarkerRef.current.setLatLng(latlng)
      robotMarkerRef.current.setIcon(robotIcon(pose.theta, state, iconOpts))
    } else {
      const marker = L.marker(latlng, {
        icon: robotIcon(pose.theta, state, iconOpts),
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
  }, [pose, state, batteryPct])

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

      {/* Compass + scale chip — top-right of the map area, portal'd to body. */}
      <CompassScale map={mapInstance} theta={pose.theta} />

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
