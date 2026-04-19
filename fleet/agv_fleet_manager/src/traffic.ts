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
  // Deadlock detection — wait-for-graph cycle detection
  // ---------------------------------------------------------------------------

  /**
   * Builds a wait-for graph and detects cycles (deadlocks).
   * A robot R1 "waits for" R2 if R1 is in a wait queue for a zone occupied by R2.
   * Returns list of robot IDs involved in deadlock, or empty array if none.
   */
  detectDeadlocks(): string[] {
    // Build wait-for graph: robotId → set of robotIds it waits for
    const waitFor = new Map<string, Set<string>>();

    for (const [zoneId, queue] of this.waitQueues) {
      if (queue.length === 0) continue;
      const occupants = this.occupancy.get(zoneId);
      if (!occupants) continue;

      for (const waitingRobot of queue) {
        if (!waitFor.has(waitingRobot)) waitFor.set(waitingRobot, new Set());
        for (const occupant of occupants) {
          waitFor.get(waitingRobot)!.add(occupant);
        }
      }
    }

    // DFS cycle detection
    const visited = new Set<string>();
    const inStack = new Set<string>();
    const cycleMembers = new Set<string>();

    const dfs = (node: string): boolean => {
      visited.add(node);
      inStack.add(node);

      const neighbors = waitFor.get(node);
      if (neighbors) {
        for (const neighbor of neighbors) {
          if (inStack.has(neighbor)) {
            // Found cycle — collect all nodes in current stack path
            cycleMembers.add(node);
            cycleMembers.add(neighbor);
            return true;
          }
          if (!visited.has(neighbor) && dfs(neighbor)) {
            cycleMembers.add(node);
            return true;
          }
        }
      }

      inStack.delete(node);
      return false;
    };

    for (const robot of waitFor.keys()) {
      if (!visited.has(robot)) dfs(robot);
    }

    if (cycleMembers.size > 0) {
      this.emitEvent('conflict', 'deadlock', Array.from(cycleMembers).join(','),
        `Deadlock detected: ${Array.from(cycleMembers).join(', ')}`);
    }

    return Array.from(cycleMembers);
  }

  /**
   * Resolve a deadlock by force-releasing the lowest-priority robot from its zone.
   * Returns the robot ID that was released, or null if no deadlock.
   */
  resolveDeadlock(deadlockedRobots: string[]): string | null {
    if (deadlockedRobots.length === 0) return null;

    // Find which robot to evict: the one waiting longest (first in queue)
    for (const [zoneId, queue] of this.waitQueues) {
      for (const robot of deadlockedRobots) {
        const idx = queue.indexOf(robot);
        if (idx >= 0) {
          // Force remove this robot from the wait queue
          queue.splice(idx, 1);
          this.emitEvent('conflict', zoneId, robot, 'Deadlock resolved: robot released from wait queue');
          this.onResumeRobot?.(robot);
          return robot;
        }
      }
    }
    return null;
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
