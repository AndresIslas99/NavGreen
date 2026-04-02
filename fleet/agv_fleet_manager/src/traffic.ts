/**
 * Traffic Manager — Zone-based mutual exclusion for multi-robot coordination.
 *
 * Progressive features:
 * 1. Mutual exclusion zones — only one robot at a time
 * 2. One-way lanes — directed corridors
 * 3. Yield points — check before entering
 * 4. Priority scheduling — higher priority gets right-of-way
 *
 * This runs inside the fleet manager, checking robot positions against
 * defined zones and issuing pause/resume via VDA 5050 instant actions.
 */

export interface TrafficZone {
  id: string;
  type: 'exclusion' | 'one_way' | 'yield';
  polygon: Array<{ x: number; y: number }>;  // convex polygon vertices
  direction?: number;       // radians, for one_way zones
  directionTolerance?: number; // allowed deviation from direction
  maxRobots: number;        // 1 for mutual exclusion
  priority?: number;        // higher = more important
}

export interface ZoneOccupancy {
  zoneId: string;
  robotIds: string[];
  waitingRobots: string[];  // robots waiting to enter
}

export interface TrafficEvent {
  timestamp: number;
  type: 'enter' | 'exit' | 'wait' | 'grant' | 'conflict';
  zoneId: string;
  robotId: string;
  detail?: string;
}

function pointInPolygon(x: number, y: number, polygon: Array<{ x: number; y: number }>): boolean {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x, yi = polygon[i].y;
    const xj = polygon[j].x, yj = polygon[j].y;
    if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
      inside = !inside;
    }
  }
  return inside;
}

const DEBOUNCE_MS = 500; // Prevent rapid enter/exit oscillation

export class TrafficManager {
  private zones: Map<string, TrafficZone> = new Map();
  private occupancy: Map<string, Set<string>> = new Map();    // zoneId → robotIds inside
  private waitQueues: Map<string, string[]> = new Map();      // zoneId → robots waiting (ordered)
  private lastEntryTime: Map<string, number> = new Map();     // "zoneId:robotId" → timestamp
  private events: TrafficEvent[] = [];
  private onPauseRobot?: (robotId: string) => void;
  private onResumeRobot?: (robotId: string) => void;

  constructor(
    onPause?: (robotId: string) => void,
    onResume?: (robotId: string) => void,
  ) {
    this.onPauseRobot = onPause;
    this.onResumeRobot = onResume;
  }

  // ---------------------------------------------------------------------------
  // Zone management
  // ---------------------------------------------------------------------------

  addZone(zone: TrafficZone): void {
    this.zones.set(zone.id, zone);
    this.occupancy.set(zone.id, new Set());
    this.waitQueues.set(zone.id, []);
  }

  removeZone(id: string): void {
    this.zones.delete(id);
    this.occupancy.delete(id);
    this.waitQueues.delete(id);
  }

  getZones(): TrafficZone[] {
    return Array.from(this.zones.values());
  }

  getOccupancy(): ZoneOccupancy[] {
    return Array.from(this.zones.keys()).map(zoneId => ({
      zoneId,
      robotIds: Array.from(this.occupancy.get(zoneId) || []),
      waitingRobots: this.waitQueues.get(zoneId) || [],
    }));
  }

  getEvents(limit = 100): TrafficEvent[] {
    return this.events.slice(-limit);
  }

  // ---------------------------------------------------------------------------
  // Position update — called for each robot position change
  // ---------------------------------------------------------------------------

  updateRobotPosition(robotId: string, x: number, y: number, theta: number): {
    paused: boolean;
    reason?: string;
  } {
    let shouldPause = false;
    let pauseReason: string | undefined;

    for (const [zoneId, zone] of this.zones) {
      const inside = pointInPolygon(x, y, zone.polygon);
      const occupants = this.occupancy.get(zoneId)!;
      const queue = this.waitQueues.get(zoneId)!;
      const wasInside = occupants.has(robotId);

      if (inside && !wasInside) {
        // Debounce: skip if robot just exited this zone (C4)
        const debounceKey = `${zoneId}:${robotId}`;
        const lastExit = this.lastEntryTime.get(debounceKey) || 0;
        if (Date.now() - lastExit < DEBOUNCE_MS) continue;
        this.lastEntryTime.set(debounceKey, Date.now());

        // Robot entering zone
        if (occupants.size >= zone.maxRobots) {
          // Zone full — pause robot, add to wait queue
          if (!queue.includes(robotId)) {
            queue.push(robotId);
            this.emitEvent('wait', zoneId, robotId, `Zone full (${occupants.size}/${zone.maxRobots})`);
          }
          shouldPause = true;
          pauseReason = `Waiting for zone ${zoneId}`;
        } else {
          // Zone has space — allow entry
          occupants.add(robotId);
          this.emitEvent('enter', zoneId, robotId);

          // One-way check
          if (zone.type === 'one_way' && zone.direction !== undefined) {
            const tolerance = zone.directionTolerance || Math.PI / 4;
            const angleDiff = Math.abs(theta - zone.direction);
            const normalizedDiff = Math.min(angleDiff, 2 * Math.PI - angleDiff);
            if (normalizedDiff > tolerance) {
              this.emitEvent('conflict', zoneId, robotId, 'Wrong direction in one-way zone');
              shouldPause = true;
              pauseReason = `Wrong direction in zone ${zoneId}`;
            }
          }
        }
      } else if (!inside && wasInside) {
        // Robot leaving zone
        occupants.delete(robotId);
        this.emitEvent('exit', zoneId, robotId);

        // Remove from wait queue if present
        const qIdx = queue.indexOf(robotId);
        if (qIdx >= 0) queue.splice(qIdx, 1);

        // Grant access to first waiting robot
        if (queue.length > 0 && occupants.size < zone.maxRobots) {
          const nextRobot = queue.shift()!;
          occupants.add(nextRobot);
          this.emitEvent('grant', zoneId, nextRobot);
          this.onResumeRobot?.(nextRobot);
        }
      } else if (inside && wasInside) {
        // Still inside — check one-way compliance
        if (zone.type === 'one_way' && zone.direction !== undefined) {
          const tolerance = zone.directionTolerance || Math.PI / 4;
          const angleDiff = Math.abs(theta - zone.direction);
          const normalizedDiff = Math.min(angleDiff, 2 * Math.PI - angleDiff);
          if (normalizedDiff > tolerance) {
            shouldPause = true;
            pauseReason = `Wrong direction in zone ${zoneId}`;
          }
        }
      }
    }

    if (shouldPause) {
      this.onPauseRobot?.(robotId);
    }

    return { paused: shouldPause, reason: pauseReason };
  }

  /** Remove robot from all zones (disconnect handling) */
  removeRobot(robotId: string): void {
    for (const [zoneId, occupants] of this.occupancy) {
      if (occupants.delete(robotId)) {
        this.emitEvent('exit', zoneId, robotId, 'Robot disconnected');

        // Grant to waiting
        const queue = this.waitQueues.get(zoneId)!;
        const qIdx = queue.indexOf(robotId);
        if (qIdx >= 0) queue.splice(qIdx, 1);

        const zone = this.zones.get(zoneId)!;
        if (queue.length > 0 && occupants.size < zone.maxRobots) {
          const nextRobot = queue.shift()!;
          occupants.add(nextRobot);
          this.emitEvent('grant', zoneId, nextRobot);
          this.onResumeRobot?.(nextRobot);
        }
      }
    }
  }

  private emitEvent(type: TrafficEvent['type'], zoneId: string, robotId: string, detail?: string): void {
    this.events.push({ timestamp: Date.now() / 1000, type, zoneId, robotId, detail });
    if (this.events.length > 1000) this.events = this.events.slice(-500);
  }
}
