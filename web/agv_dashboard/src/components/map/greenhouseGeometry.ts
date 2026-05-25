/**
 * Greenhouse geometry constants — SSOT for visual layout in the dashboard.
 *
 * **PROPORTIONS GROUND-TRUTHED (2026-05-25)** against the operator's
 * description of the real greenhouse:
 *   - Hileras (cucumber rows) ≈ 20 m long
 *   - 2 m separation between rows (aisle-to-aisle, center-to-center)
 *   - Central corridor 3 m wide
 *   - Rail gauge 57 cm (the two parallel rails the robot's wheels run on)
 *   - AGV footprint 80 × 90 cm (`agv_geometry.yaml`)
 *
 * These values are TUNED FOR VISUAL FIDELITY in the dashboard map. They
 * differ slightly from `agv_zone_detector/include/agv_zone_detector/
 * zone_classifier_impl.hpp` (which uses 2.2 m aisle spacing and 4 m
 * corridor — both within the classifier's ±0.35 m tolerance band, so
 * production behavior is unaffected). A future
 * `GET /api/greenhouse/geometry` endpoint will unify sim + nav +
 * dashboard around a single source; until then the dashboard chooses
 * accuracy of the operator-visible model over mirror-of-C++.
 *
 * Coordinate system: world frame in meters. `worldToLatLng(x, y) = L.latLng(y, x)`
 * is used by MapView throughout — y is "lat", x is "lng".
 */

// ── Rail sections (each 20 m long, separated by a 3 m corridor) ────────────
export const REAR_X_START  = -16.5;
export const REAR_X_END    =   3.5;
export const FRONT_X_START =   6.5;   // was 7.5 — corridor now 3 m (was 4 m)
export const FRONT_X_END   =  26.5;   // was 27.5 — keeps FRONT length = 20 m

// ── Approach strips (where rail_approach hands off from Nav2) ─────────────
// Repositioned to stay inside the narrower corridor (and keep their
// 0.5 m thickness flush with each section's drivable wall).
export const REAR_APPROACH_X_START  = 4.0;
export const REAR_APPROACH_X_END    = 4.5;
export const FRONT_APPROACH_X_START = 5.5;
export const FRONT_APPROACH_X_END   = 6.0;

// ── Aisle layout ──────────────────────────────────────────────────────────
// Aisle centers = rail centers (where the AGV drives). 2 m spacing between
// aisles per operator description; 5 aisles total (A bottom → E top).
export const AISLE_CENTERS  = [-4.0, -2.0, 0.0, 2.0, 4.0] as const;
// Half-width of the rail gauge (the two parallel rails the AGV's wheels
// run on). 0.285 m → 0.57 m total = real rail gauge.
export const AISLE_HALF_W   = 0.285;
// VISUAL half-width of the painted "row" band. With 2 m between aisles,
// 1 m half-width makes adjacent bands meet edge-to-edge with no overlap —
// which is what you'd see from above: continuous plant rows with the
// rail centerlines marking each one.
export const ROW_BAND_HALF_W = 1.0;

// ── Outer margin (cosmetic only — adds breathing room around the enclosure) ─
export const OUTER_MARGIN = 1.1;

// ── Spanish row labels (matches the operator's mental model) ──────────────
// y-center → letter. Bottom-up: y=-4.4 → A, y=-2.2 → B, y=0 → C, y=+2.2 → D, y=+4.4 → E.
const AISLE_LETTERS = ['A', 'B', 'C', 'D', 'E'] as const;

export interface EnclosureBounds {
  minX: number; maxX: number; minY: number; maxY: number;
}

export interface CorridorBounds extends EnclosureBounds {}

export interface RowBand {
  /** 'A'..'E' derived from y-center index (bottom-up). */
  letter: string;
  /** Operator-facing label, e.g. "Hilera A · Atrás" or "Hilera C · Frente". */
  label: string;
  /** 'rear' (REAR section) or 'front' (FRONT section). */
  section: 'rear' | 'front';
  /** y-center of the aisle this row occupies. */
  yCenter: number;
  /** y-min of the visual band (yCenter - ROW_BAND_HALF_W). */
  yMin: number;
  /** y-max of the visual band (yCenter + ROW_BAND_HALF_W). */
  yMax: number;
  /** x-start of the band (toward the corridor). */
  xStart: number;
  /** x-end of the band (toward the outer wall). */
  xEnd: number;
}

/** Outer enclosure rectangle including OUTER_MARGIN. */
export function enclosureBounds(): EnclosureBounds {
  return {
    minX: REAR_X_START - OUTER_MARGIN,
    maxX: FRONT_X_END  + OUTER_MARGIN,
    minY: AISLE_CENTERS[0] - ROW_BAND_HALF_W - OUTER_MARGIN * 0.4,
    maxY: AISLE_CENTERS[AISLE_CENTERS.length - 1] + ROW_BAND_HALF_W + OUTER_MARGIN * 0.4,
  };
}

/** Drivable corridor between REAR and FRONT sections (the rail-free gap). */
export function corridorBounds(): CorridorBounds {
  return {
    minX: REAR_X_END,
    maxX: FRONT_X_START,
    minY: AISLE_CENTERS[0] - ROW_BAND_HALF_W,
    maxY: AISLE_CENTERS[AISLE_CENTERS.length - 1] + ROW_BAND_HALF_W,
  };
}

/** y-aligned approach strips where rail_approach takes over from Nav2. */
export function approachStrips(): { rear: EnclosureBounds; front: EnclosureBounds } {
  const yMin = AISLE_CENTERS[0] - ROW_BAND_HALF_W;
  const yMax = AISLE_CENTERS[AISLE_CENTERS.length - 1] + ROW_BAND_HALF_W;
  return {
    rear:  { minX: REAR_APPROACH_X_START,  maxX: REAR_APPROACH_X_END,  minY: yMin, maxY: yMax },
    front: { minX: FRONT_APPROACH_X_START, maxX: FRONT_APPROACH_X_END, minY: yMin, maxY: yMax },
  };
}

/** Aisle letter for a given y-center, or empty string if no match. */
export function aisleLetterFor(yCenter: number): string {
  const idx = AISLE_CENTERS.findIndex(y => Math.abs(y - yCenter) < 0.2);
  return idx >= 0 ? AISLE_LETTERS[idx] : '';
}

/** Operator-facing label combining rail letter + section.
 *
 * Naming note: an earlier version used "Hilera X" — but "hilera" in Spanish
 * means "row of plants" (cucumbers), which is what GROWS BETWEEN the rails,
 * not what the AGV drives on. The bands we render are the rails themselves
 * (the operating lanes where the AGV runs); calling them "Riel" matches the
 * operator's mental model and removes the "robot is on cucumbers" confusion.
 */
export function aisleSpanishLabel(letter: string, section: 'rear' | 'front'): string {
  const sectionLabel = section === 'rear' ? 'Atrás' : 'Frente';
  return `Riel ${letter} · ${sectionLabel}`;
}

/**
 * Given the rails returned by GET /api/rails, derive the row bands to paint.
 *
 * Each rail tag sits at one end of a row. We classify by x position:
 *   - x in REAR section  → rear row, extends from rail.x toward REAR_X_START
 *   - x in FRONT section → front row, extends from rail.x toward FRONT_X_END
 *
 * If rails are sparse (e.g. only one section registered), only those bands
 * render. The enclosure + corridor always paint from constants.
 */
export interface RailLike {
  x: number;
  y: number;
  /** approach_yaw (radians) — currently unused but retained for future use. */
  yaw?: number;
}

export function rowBands(rails: RailLike[]): RowBand[] {
  const bands: RowBand[] = [];
  for (const r of rails) {
    const letter = aisleLetterFor(r.y);
    if (!letter) continue;  // y doesn't match any aisle — skip silently
    let section: 'rear' | 'front';
    let xStart: number;
    let xEnd:   number;
    if (r.x >= REAR_X_START && r.x <= REAR_X_END) {
      section = 'rear';
      xStart  = REAR_X_START;
      xEnd    = r.x;
    } else if (r.x >= FRONT_X_START && r.x <= FRONT_X_END) {
      section = 'front';
      xStart  = r.x;
      xEnd    = FRONT_X_END;
    } else {
      continue;  // rail is outside known sections — skip silently
    }
    bands.push({
      letter,
      label: aisleSpanishLabel(letter, section),
      section,
      yCenter: r.y,
      yMin: r.y - ROW_BAND_HALF_W,
      yMax: r.y + ROW_BAND_HALF_W,
      xStart,
      xEnd,
    });
  }
  return bands;
}

/**
 * Find the active row band (if any) given a robot pose and the available bands.
 * "Active" = robot is within the band's y-range AND x-range.
 */
export function activeRowBand(bands: RowBand[], pose: { x: number; y: number } | null | undefined): RowBand | null {
  if (!pose) return null;
  for (const b of bands) {
    if (pose.y >= b.yMin && pose.y <= b.yMax && pose.x >= b.xStart && pose.x <= b.xEnd) {
      return b;
    }
  }
  return null;
}

/**
 * Placeholder bands for the empty-registry state. Emits a band for every
 * aisle × section combination using the section's full x-extent. Visualized
 * with a "ghost" treatment (dashed border, lower opacity) so the operator
 * sees the greenhouse skeleton even before defining the AprilTag-anchored
 * rail registry.
 */
export function ghostRowBands(): RowBand[] {
  const out: RowBand[] = [];
  for (let i = 0; i < AISLE_CENTERS.length; i++) {
    const y = AISLE_CENTERS[i];
    const letter = AISLE_LETTERS[i];
    for (const section of ['rear', 'front'] as const) {
      const xStart = section === 'rear' ? REAR_X_START  : FRONT_X_START;
      const xEnd   = section === 'rear' ? REAR_X_END    : FRONT_X_END;
      out.push({
        letter,
        label: aisleSpanishLabel(letter, section),
        section,
        yCenter: y,
        yMin: y - ROW_BAND_HALF_W,
        yMax: y + ROW_BAND_HALF_W,
        xStart,
        xEnd,
      });
    }
  }
  return out;
}
