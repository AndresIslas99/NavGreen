/**
 * Section — titled block inside the cockpit panel.
 *
 * Replaces the dispersed `<div className="panel-section">` patterns with a
 * proper semantic + visual primitive. Title + optional description sit in a
 * compact header; the body holds children.
 *
 * The section is intentionally NOT a Card by default — sections sit on the
 * cream cockpit bg and use vertical rhythm to separate. Pass `boxed` to
 * render them as individual cards (used in the mission strip layout).
 */
import type { ReactNode } from 'react';

interface SectionProps {
  title: string;
  description?: string;
  actions?: ReactNode;
  boxed?: boolean;
  children: ReactNode;
  className?: string;
}

export function Section({
  title,
  description,
  actions,
  boxed = false,
  children,
  className = '',
}: SectionProps) {
  return (
    <section className={`ui-section ${boxed ? 'ui-section--boxed' : ''} ${className}`}>
      <header className="ui-section__header">
        <div className="ui-section__heading">
          <h3 className="ui-section__title">{title}</h3>
          {description && <p className="ui-section__description">{description}</p>}
        </div>
        {actions && <div className="ui-section__actions">{actions}</div>}
      </header>
      <div className="ui-section__body">{children}</div>
    </section>
  );
}
