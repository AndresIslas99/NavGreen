/**
 * Battery time-to-empty (TTE) heuristic derivation.
 *
 * The BatteryState message exposes `percentage` directly. For our greenhouse
 * MVP we have not characterized the battery enough to use coulomb counting
 * (current × Δt), so we estimate a slope of pct vs. time over a rolling
 * window and divide remaining percentage by that slope.
 *
 * Why this is a heuristic and not a contract:
 *  - early samples are noisy (boot transients, harness arming)
 *  - any charge phase flips the slope sign — we suppress that case
 *  - very flat slopes produce huge TTEs (>8h) that aren't actionable
 *
 * Returns seconds remaining, or null when the answer would be misleading
 * (charging, flat, insufficient data). The dashboard renders the secondary
 * line conditionally on this value being non-null.
 */
import type { BatterySample } from './app_deps';

export interface TteOptions {
  minSamples?: number;   // minimum samples required (default 5)
  minWindowS?: number;   // minimum window duration in seconds (default 60)
  minS?: number;         // lower clamp (default 60)
  maxS?: number;         // upper clamp (default 8*3600)
}

export function deriveBatteryTte(
  samples: ReadonlyArray<BatterySample>,
  opts: TteOptions = {},
): number | null {
  const minSamples = opts.minSamples ?? 5;
  const minWindowS = opts.minWindowS ?? 60;
  const minS = opts.minS ?? 60;
  const maxS = opts.maxS ?? 8 * 3600;

  if (samples.length < minSamples) return null;

  const first = samples[0];
  const last = samples[samples.length - 1];
  const window = last.t_s - first.t_s;
  if (window < minWindowS) return null;

  // Simple least-squares slope of pct over time (pct/s). Robust enough at
  // this sample count; we already cap the buffer to 30 in the caller.
  let sumT = 0, sumP = 0;
  for (const s of samples) { sumT += s.t_s; sumP += s.pct; }
  const meanT = sumT / samples.length;
  const meanP = sumP / samples.length;
  let num = 0, den = 0;
  for (const s of samples) {
    const dt = s.t_s - meanT;
    num += dt * (s.pct - meanP);
    den += dt * dt;
  }
  if (den === 0) return null;
  const slope = num / den;  // pct per second; negative when discharging

  // Charging or flat → no useful prediction. The dashboard interprets null
  // as "show only %, hide TTE secondary line".
  if (slope >= -0.0005) return null;  // < 1.8 pct/hour discharge is "flat enough"

  const tte = last.pct / (-slope);
  if (!Number.isFinite(tte) || tte <= 0) return null;
  return Math.round(Math.max(minS, Math.min(maxS, tte)));
}
