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

export interface DefinedTag {
  id: number;
  label: string;
  description: string;
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

  addDefinedTag(label: string, description: string, x: number, y: number, yaw: number, z = 0.145): DefinedTag {
    const tag: DefinedTag = {
      id: this.nextDefinedId++,
      label,
      description,
      x,
      y,
      z,
      yaw,
      created_at: Date.now() / 1000,
    };
    this.state.defined_tags.push(tag);
    this.saveToDisk();
    this.regenerateRegistryYaml();
    return tag;
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
}
