/**
 * Tile — compositional card with icon circle + content stack.
 *
 * Used for hero indicators (StateTile, BatteryTile, LocalizationTile) and
 * future at-a-glance metrics. Structure:
 *
 *   <Tile tone="accent">
 *     <Tile.Icon><Compass /></Tile.Icon>
 *     <Tile.Content>
 *       <Tile.Eyebrow>LOCALIZATION</Tile.Eyebrow>
 *       <Tile.Value>Optimal</Tile.Value>
 *       <Tile.Sub>Mapa: greenhouse-mvp</Tile.Sub>
 *     </Tile.Content>
 *   </Tile>
 *
 * tone:
 *   neutral — grey icon circle. Default idle states.
 *   accent  — green icon circle. Healthy / active.
 *   warn    — amber icon circle. Degraded.
 *   crit    — red icon circle. Failed.
 */
import type { ReactNode } from 'react';

type Tone = 'neutral' | 'accent' | 'warn' | 'crit';

interface TileProps {
  tone?: Tone;
  children: ReactNode;
  className?: string;
}

interface SimpleProps { children: ReactNode }

function TileRoot({ tone = 'neutral', children, className = '' }: TileProps) {
  return (
    <article
      className={`ui-tile ui-tile--tone-${tone} ${className}`}
      role="status"
      aria-live="polite"
    >
      {children}
    </article>
  );
}

function Icon({ children }: SimpleProps) {
  return <div className="ui-tile__icon">{children}</div>;
}

function Content({ children }: SimpleProps) {
  return <div className="ui-tile__content">{children}</div>;
}

function Eyebrow({ children }: SimpleProps) {
  return <span className="ui-tile__eyebrow">{children}</span>;
}

function Value({ children }: SimpleProps) {
  return <span className="ui-tile__value">{children}</span>;
}

function Sub({ children }: SimpleProps) {
  return <span className="ui-tile__sub">{children}</span>;
}

export const Tile = Object.assign(TileRoot, { Icon, Content, Eyebrow, Value, Sub });
