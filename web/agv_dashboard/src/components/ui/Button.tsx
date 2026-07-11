/**
 * Button — single primitive, four variants, three sizes.
 *
 * variant:
 *   primary     — accent (forest green) bg, white text. The one "go" action.
 *   secondary   — surface bg, dark text, subtle border. Most buttons.
 *   ghost       — transparent, hover lights up. Low-emphasis (Cancel, Limpiar).
 *   destructive — crit text on surface (or crit bg as "danger primary").
 *
 * size:
 *   sm — 36px high, font-sm. Inline within rows.
 *   md — 44px high, font-base. Default, WCAG min touch.
 *   lg — 56px high, font-md. ActionStack-style CTAs.
 *
 * loading: replaces the leading icon with a spinning Loader and disables click.
 * leadingIcon / trailingIcon: any Lucide component. Aligned vertically with text.
 */
import type { ReactNode, ButtonHTMLAttributes } from 'react';
import type { LucideIcon } from './icons';
import { Loader } from './icons';

type Variant = 'primary' | 'secondary' | 'ghost' | 'destructive';
type Size = 'sm' | 'md' | 'lg';

interface ButtonProps extends Omit<ButtonHTMLAttributes<HTMLButtonElement>, 'children'> {
  variant?: Variant;
  size?: Size;
  leadingIcon?: LucideIcon;
  trailingIcon?: LucideIcon;
  loading?: boolean;
  block?: boolean;        // full-width
  children?: ReactNode;
}

const ICON_SIZE: Record<Size, number> = { sm: 14, md: 16, lg: 18 };

export function Button({
  variant = 'secondary',
  size = 'md',
  leadingIcon: LeadingIcon,
  trailingIcon: TrailingIcon,
  loading = false,
  block = false,
  disabled = false,
  className = '',
  children,
  ...rest
}: ButtonProps) {
  const classes = [
    'ui-btn',
    `ui-btn--${variant}`,
    `ui-btn--${size}`,
    block ? 'ui-btn--block' : '',
    loading ? 'ui-btn--loading' : '',
    className,
  ].filter(Boolean).join(' ');

  const iconSize = ICON_SIZE[size];

  return (
    <button
      type="button"
      className={classes}
      disabled={disabled || loading}
      {...rest}
    >
      {loading
        ? <Loader size={iconSize} className="ui-btn__icon ui-btn__icon--spin" />
        : LeadingIcon && <LeadingIcon size={iconSize} className="ui-btn__icon" />}
      {children && <span className="ui-btn__label">{children}</span>}
      {TrailingIcon && !loading && <TrailingIcon size={iconSize} className="ui-btn__icon" />}
    </button>
  );
}
