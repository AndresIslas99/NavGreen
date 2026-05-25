/**
 * Rail map labels — proxies agv_rail_approach/list_rail_starts service into REST.
 *
 * Replaces the hardcoded RAIL_AISLE_Y constants that previously lived in
 * MapView.tsx (and violated the "no hardcoded physical parameters" engineering
 * rule). The frontend now draws rail labels from this endpoint, falling back
 * to an empty overlay when the service is unavailable.
 *
 * Caching: 30 seconds. Greenhouse rails don't move between calls; the service
 * roundtrip would otherwise hit on every dashboard tab focus.
 *
 * aisle_letter is derived in the backend (sort rails by y ascending, label
 * A..Z by index) so every client renders identical labels.
 */

import type { Express } from 'express';
import type { AppDeps } from '../app_deps';
import type { RosBridge } from '../app_deps';

export interface RailEntry {
  id: string;          // 'tag_<id>' — stable across calls
  tag_id: number;      // AprilTag ID at the rail start
  label: string;       // 'RIEL A', 'RIEL B', … (derived from aisle_letter)
  x: number;
  y: number;
  yaw: number;
  aisle_letter: string;
  kind: 'rail_entry';
}

interface Cache {
  rails: RailEntry[] | null;
  expires: number;       // wall-clock seconds
  pending: Promise<RailEntry[]> | null;
}

function aisleLetter(index: number): string {
  // 0 → A, 1 → B, …, 25 → Z, 26 → AA, … (overflow is unlikely in a greenhouse).
  if (index < 26) return String.fromCharCode(65 + index);
  const hi = Math.floor(index / 26) - 1;
  const lo = index % 26;
  return String.fromCharCode(65 + hi) + String.fromCharCode(65 + lo);
}

async function fetchRails(ros: RosBridge): Promise<RailEntry[]> {
  if (!(ros as any).listRailStarts) return [];
  try {
    const raw = await (ros as any).listRailStarts();
    if (!Array.isArray(raw)) return [];
    const sorted = [...raw].sort((a: any, b: any) => (a.y ?? 0) - (b.y ?? 0));
    return sorted.map((r: any, i: number) => {
      const letter = aisleLetter(i);
      return {
        id: `tag_${r.tag_id}`,
        tag_id: r.tag_id,
        label: `RIEL ${letter}`,
        x: r.x,
        y: r.y,
        yaw: r.approach_yaw,
        aisle_letter: letter,
        kind: 'rail_entry' as const,
      };
    });
  } catch {
    return [];
  }
}

export function register(app: Express, deps: AppDeps): void {
  const { ros } = deps;
  const cache: Cache = { rails: null, expires: 0, pending: null };

  app.get('/api/rails', async (_req, res) => {
    const now = Date.now() / 1000;
    if (cache.rails && now < cache.expires) {
      return res.json(cache.rails);
    }
    if (!cache.pending) {
      cache.pending = fetchRails(ros).then(rails => {
        cache.rails = rails;
        cache.expires = Date.now() / 1000 + 30;
        cache.pending = null;
        return rails;
      });
    }
    try {
      const rails = await cache.pending;
      res.json(rails);
    } catch {
      // Last-ditch graceful degradation — never 500 here.
      res.json([]);
    }
  });
}
