/**
 * Skeleton — animated placeholder while data loads.
 *
 * Variants: bar (default), circle, text.
 * Respects prefers-reduced-motion (renders static block when reduced).
 *
 * For the dashboard, used mainly inside hero tiles and cockpit cards when
 * `status === null && connected`. Once the first WS message arrives, the
 * skeleton swaps out for live data without layout shift.
 */
type Variant = 'bar' | 'circle' | 'text';

interface SkeletonProps {
  variant?: Variant;
  width?: number | string;
  height?: number | string;
  className?: string;
}

export function Skeleton({ variant = 'bar', width, height, className = '' }: SkeletonProps) {
  return (
    <span
      className={`ui-skeleton ui-skeleton--${variant} ${className}`}
      aria-hidden="true"
      style={{ width, height }}
    />
  );
}
