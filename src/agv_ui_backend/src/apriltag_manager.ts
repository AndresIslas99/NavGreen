/**
 * AprilTagManager — operator-driven tag definition and hardware-to-tag assignment.
 *
 * Workflow:
 *   1. Operator defines tags via dashboard with (x, y, yaw) coordinates
 *   2. When robot detects an AprilTag, the hardware ID is added to pending_detections
 *   3. Dashboard shows blocking modal: "Which defined tag is this hardware ID?"
 *   4. Operator assigns hardware_id → defined_tag_id
 *   5. Manager generates runtime_markers_registry.yaml and triggers marker_correction reload
 *
 * Persistence: ~/agv_data/apriltags.json (defined tags + assignments)
 * Generated:   ~/agv_data/runtime_markers_registry.yaml (consumed by marker_correction_node)
 */

import * as fs from 'fs';
import * as path from 'path';
import * as yaml from 'js-yaml';

export type TagType = 'wall' | 'rail_start';

// Sub-fase 1.2 — operator-facing schema for the Tag Layout Loader.
// Distinct from the internal `DefinedTag` because the operator types
// IDs as the HARDWARE AprilTag IDs (the integers physically printed
// on the markers), and yaw in DEGREES for readability. This schema
// also carries `role` / `rail_id` metadata that the legacy DefinedTag
// flattened into the binary `type` field.
export type LayoutRole = 'charging' | 'rail_entry' | 'central_aisle_beacon' | 'handoff' | 'other';

export interface LayoutTag {
  id: number;                                 // hardware AprilTag ID
  role: LayoutRole;
  rail_id?: string;                           // required if role === 'rail_entry'
  label?: string;
  description?: string;
  pose: { x: number; y: number; z: number; yaw_deg: number };
  size?: number;
  family?: string;
}

export interface LayoutFile {
  metadata?: {
    greenhouse_name?: string;
    block_id?: string;
    created_by?: string;
    created_at?: string;
    schema_version?: number;
    notes?: string;
  };
  defaults?: { family?: string; size?: number };
  tags: LayoutTag[];
}

export interface ValidationError {
  index: number;                              // index in tags[] of the offending entry, -1 if top-level
  id?: number;
  field: string;
  message: string;
}

export interface ValidationResult {
  valid: boolean;
  parsed: LayoutFile | null;
  errors: ValidationError[];
}

export interface DefinedTag {
  id: number;
  label: string;
  description: string;
  type: TagType;
  x: number;
  y: number;
  z: number;
  yaw: number;  // radians
  created_at: number;
}

export interface AprilTagState {
  defined_tags: DefinedTag[];
  hardware_assignments: Record<string, number>;  // hardware_id (str) -> defined_tag_id
}

export interface PendingDetection {
  hardware_id: number;
  first_seen: number;
}

export class AprilTagManager {
  private state: AprilTagState;
  private pendingDetections: Map<number, PendingDetection>;
  // Last-seen timestamp per hardware ID (assigned or not). Updated on
  // every detection. Used by hasRecentDetection() for the /align gate.
  private lastSeenByHwId: Map<number, number> = new Map();
  private nextDefinedId: number;
  private statePath: string;
  private registryYamlPath: string;
  private onPendingCallback: ((detection: PendingDetection) => void) | null = null;
  private onRegistryChangedCallback: (() => void) | null = null;

  constructor(dataDir: string) {
    this.statePath = path.join(dataDir, 'apriltags.json');
    this.registryYamlPath = path.join(dataDir, 'runtime_markers_registry.yaml');
    this.state = { defined_tags: [], hardware_assignments: {} };
    this.pendingDetections = new Map();
    this.nextDefinedId = 1;
    this.loadFromDisk();
  }

  // ── Persistence ──

  private loadFromDisk(): void {
    try {
      if (fs.existsSync(this.statePath)) {
        const raw = fs.readFileSync(this.statePath, 'utf-8');
        this.state = JSON.parse(raw);
        // Migrate: default type='wall' for tags created before type field existed
        for (const tag of this.state.defined_tags) {
          if (!tag.type) tag.type = 'wall';
        }
        // Compute next ID
        const maxId = this.state.defined_tags.reduce((m, t) => Math.max(m, t.id), 0);
        this.nextDefinedId = maxId + 1;
      }
    } catch (e) {
      console.warn('[AprilTagManager] Failed to load state:', e);
      this.state = { defined_tags: [], hardware_assignments: {} };
    }
  }

  private saveToDisk(): void {
    try {
      fs.writeFileSync(this.statePath, JSON.stringify(this.state, null, 2));
    } catch (e) {
      console.warn('[AprilTagManager] Failed to save state:', e);
    }
  }

  // ── Defined tag CRUD ──

  addDefinedTag(label: string, description: string, x: number, y: number, yaw: number,
                type: TagType = 'wall', z?: number): DefinedTag {
    // Default z: wall tags at 0.145m, rail_start (floor) tags at 0.002m
    const defaultZ = type === 'rail_start' ? 0.002 : 0.145;
    const tag: DefinedTag = {
      id: this.nextDefinedId++,
      label,
      description,
      type,
      x,
      y,
      z: z !== undefined ? z : defaultZ,
      yaw,
      created_at: Date.now() / 1000,
    };
    this.state.defined_tags.push(tag);
    this.saveToDisk();
    this.regenerateRegistryYaml();
    return tag;
  }

  /// Get a defined tag by ID (or null if not found)
  getDefinedTag(id: number): DefinedTag | null {
    return this.state.defined_tags.find(t => t.id === id) || null;
  }

  /// Find a tag near coordinates (within snap_radius_m)
  findTagNear(x: number, y: number, snap_radius_m = 0.5): DefinedTag | null {
    let nearest: DefinedTag | null = null;
    let nearest_dist = Infinity;
    for (const tag of this.state.defined_tags) {
      const dx = tag.x - x;
      const dy = tag.y - y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < snap_radius_m && dist < nearest_dist) {
        nearest = tag;
        nearest_dist = dist;
      }
    }
    return nearest;
  }

  /// Get the defined tag assigned to a hardware ID, or null
  getDefinedForHardware(hardware_id: number): DefinedTag | null {
    const definedId = this.state.hardware_assignments[String(hardware_id)];
    if (definedId === undefined) return null;
    return this.getDefinedTag(definedId);
  }

  updateDefinedTag(id: number, fields: Partial<Omit<DefinedTag, 'id' | 'created_at'>>): boolean {
    const tag = this.state.defined_tags.find(t => t.id === id);
    if (!tag) return false;
    Object.assign(tag, fields);
    this.saveToDisk();
    this.regenerateRegistryYaml();
    return true;
  }

  deleteDefinedTag(id: number): boolean {
    const idx = this.state.defined_tags.findIndex(t => t.id === id);
    if (idx === -1) return false;
    this.state.defined_tags.splice(idx, 1);
    // Remove any hardware assignments pointing to this defined tag
    for (const [hwId, defId] of Object.entries(this.state.hardware_assignments)) {
      if (defId === id) delete this.state.hardware_assignments[hwId];
    }
    this.saveToDisk();
    this.regenerateRegistryYaml();
    return true;
  }

  // ── Hardware assignment ──

  assignHardware(hardware_id: number, defined_id: number): boolean {
    const tag = this.state.defined_tags.find(t => t.id === defined_id);
    if (!tag) return false;
    this.state.hardware_assignments[String(hardware_id)] = defined_id;
    this.pendingDetections.delete(hardware_id);
    this.saveToDisk();
    this.regenerateRegistryYaml();
    return true;
  }

  unassignHardware(hardware_id: number): boolean {
    const key = String(hardware_id);
    if (!(key in this.state.hardware_assignments)) return false;
    delete this.state.hardware_assignments[key];
    this.saveToDisk();
    this.regenerateRegistryYaml();
    return true;
  }

  isAssigned(hardware_id: number): boolean {
    return String(hardware_id) in this.state.hardware_assignments;
  }

  // ── Pending detections ──

  recordPendingDetection(hardware_id: number): void {
    // Always update last-seen timestamp, even when the tag is assigned.
    // Used by hasRecentDetection() for the /align endpoint pre-flight.
    this.lastSeenByHwId.set(hardware_id, Date.now() / 1000);
    if (this.isAssigned(hardware_id)) return;
    if (this.pendingDetections.has(hardware_id)) return;  // Already pending
    const detection: PendingDetection = {
      hardware_id,
      first_seen: Date.now() / 1000,
    };
    this.pendingDetections.set(hardware_id, detection);
    if (this.onPendingCallback) this.onPendingCallback(detection);
  }

  dismissPending(hardware_id: number): boolean {
    return this.pendingDetections.delete(hardware_id);
  }

  getPendingDetections(): PendingDetection[] {
    return Array.from(this.pendingDetections.values()).sort((a, b) => a.first_seen - b.first_seen);
  }

  /**
   * True if hardware tag was seen within `max_age_s` seconds.
   * Used by /api/apriltags/:hw/align to fail fast when the tag is not
   * currently visible. Tracks all detections (assigned or not), so it
   * works for tags the operator already mapped.
   */
  hasRecentDetection(hardware_id: number, max_age_s: number): boolean {
    const t = this.lastSeenByHwId.get(hardware_id);
    if (t === undefined) return false;
    return (Date.now() / 1000 - t) <= max_age_s;
  }

  // ── Public state accessors ──

  getDefinedTags(): DefinedTag[] {
    return [...this.state.defined_tags];
  }

  getHardwareAssignments(): Record<string, number> {
    return { ...this.state.hardware_assignments };
  }

  // ── Callback registration ──

  onPendingDetection(cb: (detection: PendingDetection) => void): void {
    this.onPendingCallback = cb;
  }

  onRegistryChanged(cb: () => void): void {
    this.onRegistryChangedCallback = cb;
  }

  // ── YAML generation ──

  private regenerateRegistryYaml(): void {
    const lines: string[] = [
      '# AprilTag marker registry — auto-generated by AprilTagManager',
      '# DO NOT EDIT MANUALLY — changes will be overwritten',
      '# Edit via dashboard "AprilTags" tab',
      '#',
      '# type: wall (vertical, drift correction) | rail_start (horizontal, precision approach)',
      '',
      'markers:',
    ];
    for (const [hwIdStr, defId] of Object.entries(this.state.hardware_assignments)) {
      const tag = this.state.defined_tags.find(t => t.id === defId);
      if (!tag) continue;
      lines.push(`  - id: ${hwIdStr}  # ${tag.label}`);
      lines.push(`    x: ${tag.x.toFixed(4)}`);
      lines.push(`    y: ${tag.y.toFixed(4)}`);
      lines.push(`    z: ${tag.z.toFixed(4)}`);
      lines.push(`    yaw: ${tag.yaw.toFixed(4)}`);
      // Emit type only for rail_start (agv_rail_approach scans for this)
      if (tag.type === 'rail_start') {
        lines.push(`    type: rail_start`);
      }
    }
    try {
      fs.writeFileSync(this.registryYamlPath, lines.join('\n') + '\n');
      if (this.onRegistryChangedCallback) this.onRegistryChangedCallback();
    } catch (e) {
      console.warn('[AprilTagManager] Failed to write YAML:', e);
    }
  }

  getRegistryYamlPath(): string {
    return this.registryYamlPath;
  }

  // ──────────────────────────────────────────────────────────────────
  // Sub-fase 1.2 — Tag Layout Loader (bulk import + probe + export)
  //
  // The legacy methods above (addDefinedTag/assignHardware/etc.) are
  // the single-tag form-driven workflow. The Tag Layout Loader works
  // in bulk and keeps an operator-friendly YAML at
  // ${dataDir}/tags/current_layout.yaml plus a history dir for
  // snapshots. The runtime registry generation path is unchanged —
  // the layout YAML is the operator-facing record, not what ROS
  // consumes.
  // ──────────────────────────────────────────────────────────────────

  private layoutDir(): string {
    return path.join(path.dirname(this.statePath), 'tags');
  }
  private layoutPath(): string { return path.join(this.layoutDir(), 'current_layout.yaml'); }
  private historyDir(): string { return path.join(this.layoutDir(), 'history'); }
  private examplePath(): string { return path.join(this.layoutDir(), 'examples', 'sample_layout.yaml'); }

  /** Idempotent; safe to call multiple times. */
  private ensureLayoutDirs(): void {
    fs.mkdirSync(this.layoutDir(), { recursive: true });
    fs.mkdirSync(this.historyDir(), { recursive: true });
    fs.mkdirSync(path.dirname(this.examplePath()), { recursive: true });
  }

  /**
   * Parse + validate a YAML string against the operator-facing schema.
   * NEVER persists. Caller decides whether to apply.
   */
  validateLayoutYaml(yamlContent: string): ValidationResult {
    const errors: ValidationError[] = [];
    let raw: any;
    try {
      raw = yaml.load(yamlContent);
    } catch (e: any) {
      errors.push({ index: -1, field: 'yaml', message: `parse error: ${e?.message ?? e}` });
      return { valid: false, parsed: null, errors };
    }
    if (!raw || typeof raw !== 'object') {
      errors.push({ index: -1, field: 'root', message: 'top-level value must be a mapping' });
      return { valid: false, parsed: null, errors };
    }
    if (!Array.isArray(raw.tags)) {
      errors.push({ index: -1, field: 'tags', message: '`tags:` must be a sequence' });
      return { valid: false, parsed: null, errors };
    }
    const validRoles: LayoutRole[] = ['charging', 'rail_entry', 'central_aisle_beacon', 'handoff', 'other'];
    const seenIds = new Set<number>();
    const seenPoses = new Map<string, number>();
    const parsedTags: LayoutTag[] = [];
    raw.tags.forEach((t: any, i: number) => {
      if (!t || typeof t !== 'object') {
        errors.push({ index: i, field: 'tag', message: 'each tag must be a mapping' });
        return;
      }
      const id = t.id;
      if (!Number.isInteger(id) || id < 0) {
        errors.push({ index: i, id, field: 'id', message: 'id must be a non-negative integer' });
        return;
      }
      if (seenIds.has(id)) {
        errors.push({ index: i, id, field: 'id', message: `duplicate id ${id}` });
        return;
      }
      seenIds.add(id);
      const role = t.role;
      if (!validRoles.includes(role)) {
        errors.push({ index: i, id, field: 'role', message: `role must be one of ${validRoles.join('|')}` });
        return;
      }
      if (role === 'rail_entry' && (typeof t.rail_id !== 'string' || !t.rail_id.trim())) {
        errors.push({ index: i, id, field: 'rail_id', message: 'rail_id is required when role=rail_entry' });
        return;
      }
      if (!t.pose || typeof t.pose !== 'object') {
        errors.push({ index: i, id, field: 'pose', message: 'pose: {x,y,z,yaw_deg} is required' });
        return;
      }
      const { x, y, z, yaw_deg } = t.pose;
      for (const [k, v] of Object.entries({ x, y, z, yaw_deg })) {
        if (typeof v !== 'number' || !isFinite(v)) {
          errors.push({ index: i, id, field: `pose.${k}`, message: `${k} must be a finite number, got ${v}` });
          return;
        }
      }
      if (yaw_deg < -180 || yaw_deg > 180) {
        errors.push({ index: i, id, field: 'pose.yaw_deg', message: `yaw_deg must be in [-180, 180], got ${yaw_deg}` });
        return;
      }
      const size = t.size;
      if (size !== undefined) {
        if (typeof size !== 'number' || !isFinite(size) || size <= 0 || size >= 1) {
          errors.push({ index: i, id, field: 'size', message: `size must be in (0, 1) m, got ${size}` });
          return;
        }
      }
      const poseKey = `${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)}`;
      if (seenPoses.has(poseKey)) {
        errors.push({ index: i, id, field: 'pose',
          message: `pose collides with tag id ${seenPoses.get(poseKey)} at same (x,y,z)` });
        return;
      }
      seenPoses.set(poseKey, id);
      parsedTags.push({
        id, role,
        rail_id: t.rail_id,
        label: typeof t.label === 'string' ? t.label : undefined,
        description: typeof t.description === 'string' ? t.description : undefined,
        pose: { x, y, z, yaw_deg },
        size,
        family: typeof t.family === 'string' ? t.family : undefined,
      });
    });
    if (errors.length > 0) return { valid: false, parsed: null, errors };
    const layout: LayoutFile = {
      metadata: raw.metadata,
      defaults: raw.defaults,
      tags: parsedTags,
    };
    return { valid: true, parsed: layout, errors: [] };
  }

  /**
   * Apply a previously-validated layout. Writes current_layout.yaml,
   * snapshots history, replaces all defined_tags + hardware_assignments
   * with the new layout, and regenerates the runtime registry +
   * publishes the reload event (the latter via onRegistryChanged).
   *
   * `replace=true` (default): wipe all existing tags + assignments.
   * `replace=false`: merge — new tags add to existing set, conflicts on
   *   id reject.
   */
  applyLayout(layout: LayoutFile, replace: boolean = true): { applied_count: number; replaced: boolean } {
    this.ensureLayoutDirs();
    // History snapshot of the file we're about to overwrite (if any).
    if (fs.existsSync(this.layoutPath())) {
      const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
      const reason = replace ? 'replace' : 'merge';
      const dest = path.join(this.historyDir(), `${ts}_${reason}.yaml`);
      try { fs.copyFileSync(this.layoutPath(), dest); } catch { /* ignore */ }
    }

    if (replace) {
      this.state.defined_tags = [];
      this.state.hardware_assignments = {};
      this.nextDefinedId = 1;
    } else {
      for (const t of layout.tags) {
        if (this.isAssigned(t.id)) {
          throw new Error(`merge mode: hardware id ${t.id} already assigned`);
        }
      }
    }

    for (const t of layout.tags) {
      const tagType: TagType = t.role === 'rail_entry' ? 'rail_start' : 'wall';
      const yawRad = (t.pose.yaw_deg * Math.PI) / 180;
      const def: DefinedTag = {
        id: this.nextDefinedId++,
        label: t.label ?? `${t.role}_${t.id}`,
        description: t.description ?? (t.role === 'rail_entry' ? `rail entry ${t.rail_id}` : t.role),
        type: tagType,
        x: t.pose.x,
        y: t.pose.y,
        z: t.pose.z,
        yaw: yawRad,
        created_at: Date.now() / 1000,
      };
      this.state.defined_tags.push(def);
      this.state.hardware_assignments[String(t.id)] = def.id;
    }

    // Write the operator-facing layout file last so a crash mid-apply
    // doesn't leave the operator with a layout file that doesn't match
    // what the manager actually loaded.
    this.saveToDisk();
    this.regenerateRegistryYaml();
    this.writeLayoutYaml(layout);
    return { applied_count: layout.tags.length, replaced: replace };
  }

  /** Emit the layout file from an in-memory LayoutFile (operator-facing schema). */
  private writeLayoutYaml(layout: LayoutFile): void {
    this.ensureLayoutDirs();
    const dump = yaml.dump(layout, { noRefs: true, lineWidth: 100 });
    fs.writeFileSync(this.layoutPath(), dump);
  }

  /** Read the operator-facing layout file. Returns null if absent. */
  getCurrentLayout(): LayoutFile | null {
    if (!fs.existsSync(this.layoutPath())) return null;
    try {
      const raw = fs.readFileSync(this.layoutPath(), 'utf-8');
      return yaml.load(raw) as LayoutFile;
    } catch {
      return null;
    }
  }
  getCurrentLayoutYaml(): string {
    if (!fs.existsSync(this.layoutPath())) return '';
    return fs.readFileSync(this.layoutPath(), 'utf-8');
  }
  getExampleYaml(): string {
    if (!fs.existsSync(this.examplePath())) return SAMPLE_LAYOUT_YAML;
    return fs.readFileSync(this.examplePath(), 'utf-8');
  }
  installExampleAtBoot(): void {
    this.ensureLayoutDirs();
    if (!fs.existsSync(this.examplePath())) {
      try { fs.writeFileSync(this.examplePath(), SAMPLE_LAYOUT_YAML); } catch { /* ignore */ }
    }
  }

  /**
   * Single-tag add/update for the Robot Probe (Modo 3).
   * Hardware id is the AprilTag id observed; pose is in map frame.
   * If hw_id already exists, behaves as update.
   */
  addOrUpdateProbedTag(
    hw_id: number, role: LayoutRole, rail_id: string | undefined,
    x: number, y: number, z: number, yaw_rad: number, size?: number,
  ): { added: boolean; updated: boolean } {
    const tagType: TagType = role === 'rail_entry' ? 'rail_start' : 'wall';
    const existingDefId = this.state.hardware_assignments[String(hw_id)];
    let updated = false;
    if (existingDefId !== undefined) {
      const tag = this.state.defined_tags.find(t => t.id === existingDefId);
      if (tag) {
        tag.type = tagType;
        tag.x = x; tag.y = y; tag.z = z; tag.yaw = yaw_rad;
        tag.label = `${role}_${hw_id}`;
        tag.description = role === 'rail_entry' ? `rail entry ${rail_id ?? ''}` : role;
        updated = true;
      }
    } else {
      const def: DefinedTag = {
        id: this.nextDefinedId++,
        label: `${role}_${hw_id}`,
        description: role === 'rail_entry' ? `rail entry ${rail_id ?? ''}` : role,
        type: tagType,
        x, y, z, yaw: yaw_rad,
        created_at: Date.now() / 1000,
      };
      this.state.defined_tags.push(def);
      this.state.hardware_assignments[String(hw_id)] = def.id;
    }
    this.saveToDisk();
    this.regenerateRegistryYaml();
    // Rewrite layout YAML so the operator file stays in sync.
    this.writeLayoutYaml(this.exportLayoutFromState());
    void size; // size handled at registry level via default; per-tag size not yet plumbed
    return { added: !updated, updated };
  }

  /**
   * Build a LayoutFile (operator-facing schema) from the current
   * in-memory state. Used to keep current_layout.yaml in sync after
   * single-tag operations (probe save, legacy form CRUD).
   */
  private exportLayoutFromState(): LayoutFile {
    const tags: LayoutTag[] = [];
    for (const [hwIdStr, defId] of Object.entries(this.state.hardware_assignments)) {
      const def = this.state.defined_tags.find(t => t.id === defId);
      if (!def) continue;
      const id = parseInt(hwIdStr, 10);
      // We lost the original role beyond {wall, rail_start}. Map back
      // by best-effort: rail_start → rail_entry, wall → 'other'. The
      // user can refine via re-import if they had a richer original
      // layout.
      const role: LayoutRole = def.type === 'rail_start' ? 'rail_entry' : 'other';
      tags.push({
        id,
        role,
        rail_id: role === 'rail_entry' ? (def.description?.replace(/^rail entry\s*/i, '') || undefined) : undefined,
        label: def.label,
        description: def.description,
        pose: {
          x: def.x, y: def.y, z: def.z,
          yaw_deg: (def.yaw * 180) / Math.PI,
        },
      });
    }
    return { metadata: { schema_version: 1 }, tags };
  }
}

// Operator-facing sample shipped to ${AGV_DATA_DIR}/tags/examples/.
const SAMPLE_LAYOUT_YAML = `# AGV Tag Layout — Example
#
# Operator-facing layout file for the Tag Layout Loader UI. Edit and
# re-import via the dashboard's AprilTags panel (Modo 1: YAML Import)
# OR build it incrementally via Modo 3 (Robot Probe).
#
# Schema notes:
#   - id              MUST be the AprilTag hardware id (integer printed on the tag).
#   - role            charging | rail_entry | central_aisle_beacon | handoff | other.
#   - rail_id         required when role == rail_entry; freeform string operators agree on.
#   - pose.yaw_deg    in DEGREES, range [-180, 180]. Backend converts to radians.
#   - size            optional, in meters. Falls back to defaults.size.
#
# After import, the runtime markers registry consumed by ROS
# (\${AGV_DATA_DIR}/runtime_markers_registry.yaml) is regenerated and
# /agv/markers/registry_reload is published so marker_correction and
# rail_approach pick up the change without a service restart.

metadata:
  greenhouse_name: "Example-Greenhouse"
  block_id: "block_A"
  schema_version: 1
  notes: |
    Sample layout shipped with the dashboard. Replace with your real
    layout before deploying. Six tags: 1 charging, 2 rail entries,
    3 central aisle beacons.

defaults:
  family: tag36h11
  size: 0.20

tags:
  - id: 99
    role: charging
    label: charging_dock
    pose:
      x: 0.0
      y: -2.5
      z: 0.10
      yaw_deg: 0.0

  - id: 100
    role: rail_entry
    rail_id: rail_1_north
    pose:
      x: 1.0
      y: 1.5
      z: 0.10
      yaw_deg: 90.0

  - id: 101
    role: rail_entry
    rail_id: rail_2_north
    pose:
      x: 3.0
      y: 1.5
      z: 0.10
      yaw_deg: 90.0

  - id: 150
    role: central_aisle_beacon
    pose:
      x: 1.0
      y: 0.0
      z: 1.20
      yaw_deg: 0.0

  - id: 151
    role: central_aisle_beacon
    pose:
      x: 2.0
      y: 0.0
      z: 1.20
      yaw_deg: 0.0

  - id: 152
    role: central_aisle_beacon
    pose:
      x: 3.0
      y: 0.0
      z: 1.20
      yaw_deg: 0.0
`;
