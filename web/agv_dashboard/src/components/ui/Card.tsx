/**
 * Card — white surface with subtle elevation. The most-used primitive.
 *
 * Use for: hero tiles, cockpit sections, mission strip cards, modals, any
 * container that should "lift" off the cream bg.
 *
 * Padding ladder (4-8-12-16-24-32 grid):
 *   compact     = 12px   (event ticker rows, tight pills containers)
 *   default     = 16px   (typical card)
 *   comfortable = 20px   (hero tiles, mission progress card)
 *   spacious    = 24px   (modals, focal-attention surfaces)
 *
 * Tone tints the top border to indicate semantic meaning without coloring
 * the whole card (preserves the calm palette).
 */
import type { ReactNode, HTMLAttributes } from 'react';

type Padding = 'compact' | 'default' | 'comfortable' | 'spacious';
type Tone = 'neutral' | 'accent' | 'warn' | 'crit' | 'info';
type Shadow = 'none' | 'sm' | 'md';

interface CardProps extends HTMLAttributes<HTMLDivElement> {
  padding?: Padding;
  tone?: Tone;
  shadow?: Shadow;
  interactive?: boolean;
  as?: 'div' | 'article' | 'section' | 'aside';
  children?: ReactNode;
}

const PADDING_CLASS: Record<Padding, string> = {
  compact:     'ui-card--p-compact',
  default:     'ui-card--p-default',
  comfortable: 'ui-card--p-comfortable',
  spacious:    'ui-card--p-spacious',
};

export function Card({
  padding = 'default',
  tone = 'neutral',
  shadow = 'sm',
  interactive = false,
  as: Tag = 'div',
  className = '',
  children,
  ...rest
}: CardProps) {
  const classes = [
    'ui-card',
    PADDING_CLASS[padding],
    `ui-card--tone-${tone}`,
    `ui-card--shadow-${shadow}`,
    interactive ? 'ui-card--interactive' : '',
    className,
  ].filter(Boolean).join(' ');

  return (
    <Tag className={classes} {...rest}>
      {children}
    </Tag>
  );
}
