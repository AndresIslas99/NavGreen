/**
 * Pill — small inline chip/badge for status, category, severity.
 *
 * tone:
 *   neutral — surface-2 bg, dim text. Default "no opinion".
 *   accent  — accent-soft bg, accent text. Positive/active.
 *   warn    — warn-soft bg, warn text. Degraded.
 *   crit    — crit-soft bg, crit text. Reserved for critical.
 *   info    — info-soft bg, info text. Rare, contextual info.
 *
 * size:
 *   xs — 11px text, 2px y-padding. Event-pill style.
 *   sm — 12px text, 3px y-padding. Default.
 *   md — 13px text, 4px y-padding. More breathing.
 *
 * leadingIcon: pequeño glyph antes del label.
 */
import type { ReactNode } from 'react';
import type { LucideIcon } from './icons';

type Tone = 'neutral' | 'accent' | 'warn' | 'crit' | 'info';
type Size = 'xs' | 'sm' | 'md';

interface PillProps {
  tone?: Tone;
  size?: Size;
  leadingIcon?: LucideIcon;
  children: ReactNode;
}

const ICON_SIZE: Record<Size, number> = { xs: 10, sm: 12, md: 14 };

export function Pill({ tone = 'neutral', size = 'sm', leadingIcon: LeadingIcon, children }: PillProps) {
  return (
    <span className={`ui-pill ui-pill--tone-${tone} ui-pill--${size}`}>
      {LeadingIcon && <LeadingIcon size={ICON_SIZE[size]} className="ui-pill__icon" />}
      <span className="ui-pill__label">{children}</span>
    </span>
  );
}
