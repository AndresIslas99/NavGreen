/**
 * StatusDot — colored dot with optional ring + pulse animation.
 *
 * Used for: connection indicator, recording indicator, mission running
 * indicator. Combines color + motion + accessible label for live status.
 *
 * tone:
 *   ok      — accent (forest green)
 *   warn    — warm tan
 *   crit    — red
 *   neutral — grey
 *
 * pulse: animates a soft ring outward, indicating "live". Respects
 * prefers-reduced-motion.
 */
type Tone = 'ok' | 'warn' | 'crit' | 'neutral';
type Size = 'sm' | 'md';

interface StatusDotProps {
  tone?: Tone;
  size?: Size;
  pulse?: boolean;
  label?: string;
}

export function StatusDot({ tone = 'neutral', size = 'sm', pulse = false, label }: StatusDotProps) {
  return (
    <span
      className={`ui-dot ui-dot--tone-${tone} ui-dot--${size} ${pulse ? 'ui-dot--pulse' : ''}`}
      role={label ? 'status' : undefined}
      aria-label={label}
      title={label}
    />
  );
}
