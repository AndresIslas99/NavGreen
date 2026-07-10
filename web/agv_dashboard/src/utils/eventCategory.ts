/**
 * Maps the canonical Subsystem enum (defined in api/types.ts and emitted by
 * the backend EventLog) into a short abbreviation + color tier for the
 * dashboard's severity-tagged event pills.
 *
 * Why frontend-side: the canonical enum is already canonical (every event
 * carries `subsystem: 'DRIVE' | 'NAV' | …`). The mapping to display tokens
 * (NAV/MOT/SLAM/SAFE/MAP/MIS/SYS) is pure presentation — adding a backend
 * field for it would be a duplicate truth.
 */
import type { Subsystem, Severity } from '../api/types';

export type CategoryColor = 'crit' | 'warn' | 'info' | 'muted';

export interface Category {
  short: string;
  color: CategoryColor;
}

const SUBSYSTEM_SHORT: Record<Subsystem, string> = {
  DRIVE:   'MOT',
  NAV:     'NAV',
  SLAM:    'SLAM',
  SAFETY:  'SAFE',
  MAPPING: 'MAP',
  MISSION: 'MIS',
  SYSTEM:  'SYS',
};

/**
 * The pill color is dominated by *severity*, not subsystem — operators care
 * "how bad is this?" first, then "what subsystem". So a crit event in SLAM
 * looks the same as a crit event in NAV. The subsystem string differentiates
 * them via the text inside the pill.
 *
 * The fallback for unknown severities is 'muted' so a missing/garbled severity
 * doesn't accidentally appear critical-red.
 */
export function categoryFor(subsystem: Subsystem, severity: Severity): Category {
  const short = SUBSYSTEM_SHORT[subsystem] ?? 'SYS';
  let color: CategoryColor = 'muted';
  if (severity === 'crit') color = 'crit';
  else if (severity === 'warn') color = 'warn';
  else if (severity === 'info') color = 'info';
  return { short, color };
}
