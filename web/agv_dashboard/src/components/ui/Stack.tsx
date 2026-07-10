/**
 * Layout primitives — Stack (vertical), Cluster (wrap horizontal), Inline (no wrap).
 *
 * Gap values are 1-16 in the 4-px grid scale (see global.css :root --sp-*).
 * Saves us from sprinkling `display: flex; gap: 12px` everywhere.
 */
import type { ReactNode, CSSProperties, ElementType } from 'react';

type GapKey = 1 | 2 | 3 | 4 | 5 | 6 | 8 | 10 | 12 | 16;

interface BaseProps {
  gap?: GapKey;
  as?: ElementType;
  align?: 'start' | 'center' | 'end' | 'stretch' | 'baseline';
  justify?: 'start' | 'center' | 'end' | 'between' | 'around';
  className?: string;
  style?: CSSProperties;
  children: ReactNode;
}

interface ClusterProps extends BaseProps {
  wrap?: boolean;
}

const ALIGN_MAP: Record<NonNullable<BaseProps['align']>, string> = {
  start: 'flex-start',
  center: 'center',
  end: 'flex-end',
  stretch: 'stretch',
  baseline: 'baseline',
};

const JUSTIFY_MAP: Record<NonNullable<BaseProps['justify']>, string> = {
  start: 'flex-start',
  center: 'center',
  end: 'flex-end',
  between: 'space-between',
  around: 'space-around',
};

function gapVar(gap?: GapKey): string | undefined {
  return gap ? `var(--sp-${gap})` : undefined;
}

export function Stack({
  gap = 3,
  as: Tag = 'div',
  align,
  justify,
  className = '',
  style,
  children,
}: BaseProps) {
  return (
    <Tag
      className={`ui-stack ${className}`}
      style={{
        display: 'flex',
        flexDirection: 'column',
        gap: gapVar(gap),
        alignItems: align ? ALIGN_MAP[align] : undefined,
        justifyContent: justify ? JUSTIFY_MAP[justify] : undefined,
        ...style,
      }}
    >
      {children}
    </Tag>
  );
}

export function Cluster({
  gap = 2,
  as: Tag = 'div',
  align = 'center',
  justify,
  wrap = true,
  className = '',
  style,
  children,
}: ClusterProps) {
  return (
    <Tag
      className={`ui-cluster ${className}`}
      style={{
        display: 'flex',
        flexWrap: wrap ? 'wrap' : 'nowrap',
        alignItems: ALIGN_MAP[align],
        justifyContent: justify ? JUSTIFY_MAP[justify] : undefined,
        gap: gapVar(gap),
        ...style,
      }}
    >
      {children}
    </Tag>
  );
}

export function Inline({
  gap = 2,
  as: Tag = 'span',
  align = 'center',
  className = '',
  style,
  children,
}: BaseProps) {
  return (
    <Tag
      className={`ui-inline ${className}`}
      style={{
        display: 'inline-flex',
        alignItems: ALIGN_MAP[align],
        gap: gapVar(gap),
        ...style,
      }}
    >
      {children}
    </Tag>
  );
}
