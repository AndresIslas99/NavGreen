/**
 * Unit tests for the TrafficManager — zone-based mutual exclusion.
 *
 * These tests exercise the real invariants of the traffic module:
 * point-in-polygon occupancy, mutual-exclusion capacity, FIFO wait
 * queues, grant-on-exit semantics, enter/exit debounce, one-way lane
 * direction checks, disconnect handling, and wait-for-graph deadlock
 * detection/resolution.
 *
 * Fake timers are used because the manager debounces zone re-entry
 * (DEBOUNCE_MS = 500) using Date.now().
 */

import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { TrafficManager, TrafficZone } from '../src/traffic';

function square(cx: number, cy: number, half = 1): Array<{ x: number; y: number }> {
  return [
    { x: cx - half, y: cy - half },
    { x: cx + half, y: cy - half },
    { x: cx + half, y: cy + half },
    { x: cx - half, y: cy + half },
  ];
}

function exclusionZone(id: string, cx = 0, cy = 0, overrides: Partial<TrafficZone> = {}): TrafficZone {
  return { id, type: 'exclusion', polygon: square(cx, cy), maxRobots: 1, ...overrides };
}

describe('TrafficManager', () => {
  let onPause: ReturnType<typeof vi.fn>;
  let onResume: ReturnType<typeof vi.fn>;
  let tm: TrafficManager;

  const occ = (zoneId: string) => {
    const entry = tm.getOccupancy().find(o => o.zoneId === zoneId);
    if (!entry) throw new Error(`zone ${zoneId} not found in occupancy`);
    return entry;
  };

  beforeEach(() => {
    vi.useFakeTimers();
    // Debounce compares Date.now() against 0 for first-time entries, so the
    // fake clock must start well past DEBOUNCE_MS.
    vi.setSystemTime(new Date('2026-07-07T12:00:00Z'));
    onPause = vi.fn();
    onResume = vi.fn();
    tm = new TrafficManager(onPause, onResume);
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it('registers a robot inside the zone polygon as occupant and ignores robots outside', () => {
    tm.addZone(exclusionZone('z1'));

    const inside = tm.updateRobotPosition('A', 0, 0, 0);
    expect(inside.paused).toBe(false);
    expect(occ('z1').robotIds).toEqual(['A']);

    const outside = tm.updateRobotPosition('B', 5, 5, 0);
    expect(outside.paused).toBe(false);
    expect(occ('z1').robotIds).toEqual(['A']);
    expect(occ('z1').waitingRobots).toEqual([]);
    expect(onPause).not.toHaveBeenCalled();
  });

  it('pauses and queues a second robot entering a full mutual-exclusion zone', () => {
    tm.addZone(exclusionZone('z1'));
    tm.updateRobotPosition('A', 0, 0, 0);

    const result = tm.updateRobotPosition('B', 0.5, 0, 0);

    expect(result).toEqual({ paused: true, reason: 'Waiting for zone z1' });
    expect(occ('z1').robotIds).toEqual(['A']);
    expect(occ('z1').waitingRobots).toEqual(['B']);
    expect(onPause).toHaveBeenCalledTimes(1);
    expect(onPause).toHaveBeenCalledWith('B');

    const waitEvent = tm.getEvents().find(e => e.type === 'wait');
    expect(waitEvent).toMatchObject({ zoneId: 'z1', robotId: 'B', detail: 'Zone full (1/1)' });
  });

  it('admits robots up to maxRobots before excluding', () => {
    tm.addZone(exclusionZone('z1', 0, 0, { maxRobots: 2 }));

    expect(tm.updateRobotPosition('A', -0.5, 0, 0).paused).toBe(false);
    expect(tm.updateRobotPosition('B', 0.5, 0, 0).paused).toBe(false);
    expect(occ('z1').robotIds).toEqual(['A', 'B']);

    expect(tm.updateRobotPosition('C', 0, 0.5, 0).paused).toBe(true);
    expect(occ('z1').robotIds).toEqual(['A', 'B']);
    expect(occ('z1').waitingRobots).toEqual(['C']);
  });

  it('grants the zone to waiters in FIFO order when an occupant exits', () => {
    tm.addZone(exclusionZone('z1'));
    tm.updateRobotPosition('A', 0, 0, 0);
    tm.updateRobotPosition('B', 0.3, 0, 0);
    tm.updateRobotPosition('C', -0.3, 0, 0);
    expect(occ('z1').waitingRobots).toEqual(['B', 'C']);

    tm.updateRobotPosition('A', 10, 10, 0); // A leaves the polygon

    expect(occ('z1').robotIds).toEqual(['B']); // first waiter promoted
    expect(occ('z1').waitingRobots).toEqual(['C']); // second still queued
    expect(onResume).toHaveBeenCalledTimes(1);
    expect(onResume).toHaveBeenCalledWith('B');
    const grant = tm.getEvents().find(e => e.type === 'grant');
    expect(grant).toMatchObject({ zoneId: 'z1', robotId: 'B' });
  });

  it('does not duplicate a robot in the wait queue on repeated blocked entries', () => {
    tm.addZone(exclusionZone('z1'));
    tm.updateRobotPosition('A', 0, 0, 0);

    tm.updateRobotPosition('B', 0.5, 0, 0);
    vi.advanceTimersByTime(600); // get past the debounce window
    const second = tm.updateRobotPosition('B', 0.5, 0, 0);

    expect(second.paused).toBe(true);
    expect(occ('z1').waitingRobots).toEqual(['B']); // queued exactly once
    expect(onPause).toHaveBeenCalledTimes(2); // but pause is re-asserted
  });

  it('debounces rapid exit/re-entry oscillation within 500 ms', () => {
    tm.addZone(exclusionZone('z1'));

    tm.updateRobotPosition('A', 0, 0, 0); // enter at t0
    vi.advanceTimersByTime(100);
    tm.updateRobotPosition('A', 10, 0, 0); // exit at t0+100
    vi.advanceTimersByTime(100);

    const bounced = tm.updateRobotPosition('A', 0, 0, 0); // t0+200 < debounce
    expect(bounced.paused).toBe(false);
    expect(occ('z1').robotIds).toEqual([]); // re-entry suppressed

    vi.advanceTimersByTime(400); // t0+600 — window elapsed
    tm.updateRobotPosition('A', 0, 0, 0);
    expect(occ('z1').robotIds).toEqual(['A']);
  });

  it('pauses a robot entering a one-way lane against the direction', () => {
    tm.addZone({ id: 'lane', type: 'one_way', polygon: square(0, 0), direction: 0, maxRobots: 1 });

    const result = tm.updateRobotPosition('A', 0, 0, Math.PI); // heading opposite

    expect(result.paused).toBe(true);
    expect(result.reason).toBe('Wrong direction in zone lane');
    expect(occ('lane').robotIds).toEqual(['A']); // admitted, but paused
    expect(onPause).toHaveBeenCalledWith('A');
    const conflict = tm.getEvents().find(e => e.type === 'conflict');
    expect(conflict).toMatchObject({ zoneId: 'lane', robotId: 'A', detail: 'Wrong direction in one-way zone' });
  });

  it('normalizes angle wrap-around in the one-way direction check', () => {
    tm.addZone({ id: 'lane', type: 'one_way', polygon: square(0, 0), direction: 0, maxRobots: 1 });

    // 2π − 0.1 rad is only 0.1 rad off the lane direction once wrapped —
    // well inside the default π/4 tolerance.
    const result = tm.updateRobotPosition('A', 0, 0, 2 * Math.PI - 0.1);

    expect(result.paused).toBe(false);
    expect(onPause).not.toHaveBeenCalled();
  });

  it('flags wrong direction for a robot already inside a one-way lane', () => {
    tm.addZone({
      id: 'lane', type: 'one_way', polygon: square(0, 0),
      direction: 0, directionTolerance: Math.PI / 4, maxRobots: 1,
    });
    expect(tm.updateRobotPosition('A', 0, 0, 0).paused).toBe(false); // compliant entry

    const turned = tm.updateRobotPosition('A', 0.2, 0, Math.PI / 2); // now off-axis

    expect(turned.paused).toBe(true);
    expect(turned.reason).toBe('Wrong direction in zone lane');
    expect(occ('lane').robotIds).toEqual(['A']); // stays occupant
  });

  it('removeRobot frees its zones and promotes the next waiter (disconnect handling)', () => {
    tm.addZone(exclusionZone('z1'));
    tm.updateRobotPosition('A', 0, 0, 0);
    tm.updateRobotPosition('B', 0.5, 0, 0);

    tm.removeRobot('A');

    expect(occ('z1').robotIds).toEqual(['B']);
    expect(occ('z1').waitingRobots).toEqual([]);
    expect(onResume).toHaveBeenCalledWith('B');
    const exit = tm.getEvents().find(e => e.type === 'exit');
    expect(exit).toMatchObject({ zoneId: 'z1', robotId: 'A', detail: 'Robot disconnected' });
  });

  // Two overlapping exclusion zones: A holds z1 and waits for z2, B holds z2
  // and waits for z1 — a classic two-robot wait cycle.
  function makeDeadlock() {
    tm.addZone(exclusionZone('z1', 0, 0, { polygon: square(0, 0, 2) })); // x ∈ [-2, 2]
    tm.addZone(exclusionZone('z2', 3, 0, { polygon: square(3, 0, 2) })); // x ∈ [1, 5]
    tm.updateRobotPosition('A', -1, 0, 0); // occupies z1
    tm.updateRobotPosition('B', 4, 0, 0);  // occupies z2
    tm.updateRobotPosition('A', 1.5, 0, 0); // overlap: still in z1, blocked on z2
    tm.updateRobotPosition('B', 1.7, 0, 0); // overlap: still in z2, blocked on z1
  }

  it('detects a two-robot wait-for cycle as a deadlock', () => {
    makeDeadlock();
    expect(occ('z2').waitingRobots).toEqual(['A']);
    expect(occ('z1').waitingRobots).toEqual(['B']);

    const deadlocked = tm.detectDeadlocks();

    expect(new Set(deadlocked)).toEqual(new Set(['A', 'B']));
    const conflict = tm.getEvents().find(e => e.type === 'conflict' && e.zoneId === 'deadlock');
    expect(conflict).toBeDefined();
  });

  it('reports no deadlock for acyclic waiting', () => {
    tm.addZone(exclusionZone('z1'));
    tm.updateRobotPosition('A', 0, 0, 0);  // A occupies
    tm.updateRobotPosition('B', 0.5, 0, 0); // B waits for A — no cycle

    expect(tm.detectDeadlocks()).toEqual([]);
  });

  it('resolveDeadlock evicts one waiter from its queue and resumes it', () => {
    makeDeadlock();
    const deadlocked = tm.detectDeadlocks();

    const released = tm.resolveDeadlock(deadlocked);

    expect(released === 'A' || released === 'B').toBe(true);
    const stillWaiting = tm.getOccupancy().flatMap(o => o.waitingRobots);
    expect(stillWaiting).not.toContain(released); // evicted from its queue
    const other = released === 'A' ? 'B' : 'A';
    expect(stillWaiting).toContain(other); // the other robot still waits
    expect(onResume).toHaveBeenCalledWith(released);

    expect(tm.resolveDeadlock([])).toBeNull(); // no deadlock → nothing released
  });

  it('removeZone drops the zone, its occupancy and its wait queue', () => {
    tm.addZone(exclusionZone('z1'));
    tm.updateRobotPosition('A', 0, 0, 0);
    tm.updateRobotPosition('B', 0.5, 0, 0);

    tm.removeZone('z1');

    expect(tm.getZones()).toEqual([]);
    expect(tm.getOccupancy()).toEqual([]);
    // Position updates after removal must not throw or pause anyone.
    expect(tm.updateRobotPosition('A', 0, 0, 0)).toEqual({ paused: false, reason: undefined });
  });

  it('getEvents returns the most recent events bounded by limit', () => {
    tm.addZone(exclusionZone('z1'));
    tm.updateRobotPosition('A', 0, 0, 0);   // enter
    tm.updateRobotPosition('A', 10, 0, 0);  // exit
    vi.advanceTimersByTime(600);
    tm.updateRobotPosition('A', 0, 0, 0);   // enter again

    expect(tm.getEvents()).toHaveLength(3);
    const lastTwo = tm.getEvents(2);
    expect(lastTwo.map(e => e.type)).toEqual(['exit', 'enter']);
    // Timestamps are seconds (Date.now() / 1000)
    expect(lastTwo[1].timestamp).toBeCloseTo(Date.now() / 1000, 0);
  });
});
