/**
 * EmptyState — uniform "nothing here yet" pattern.
 *
 * Used wherever the dashboard would otherwise render blank space because the
 * underlying data is genuinely empty (no mission, no home set, no events,
 * no map). NEVER for loading or error states — those have their own
 * primitives (Skeleton, Toast, ConnectionBanner).
 *
 * Tone is intentionally neutral. Empty is the calm zero-state, not a problem.
 */
import type { ReactNode } from 'react';
import type { LucideIcon } from './icons';

interface EmptyStateProps {
  icon?: LucideIcon;
  title: string;
  description?: ReactNode;
  action?: ReactNode;
  compact?: boolean;
}

export function EmptyState({ icon: Icon, title, description, action, compact = false }: EmptyStateProps) {
  return (
    <div className={`ui-empty ${compact ? 'ui-empty--compact' : ''}`}>
      {Icon && (
        <div className="ui-empty__icon" aria-hidden="true">
          <Icon size={compact ? 18 : 28} strokeWidth={1.5} />
        </div>
      )}
      <div className="ui-empty__text">
        <p className="ui-empty__title">{title}</p>
        {description && <p className="ui-empty__description">{description}</p>}
      </div>
      {action && <div className="ui-empty__action">{action}</div>}
    </div>
  );
}
