/**
 * MapEmptyState — overlay shown when no occupancy map is loaded yet.
 *
 * Sits on top of the (already-rendered) greenhouse template so the operator
 * doesn't see a void — they see "the greenhouse exists, just no SLAM map
 * has been built yet" + a clear next-action CTA.
 */
import { Card, Button } from '../ui';
import { MapPin, MapIcon } from '../ui/icons';

interface Props {
  onStartMapping: () => void;
  onOpenMapPanel: () => void;
}

export function MapEmptyState({ onStartMapping, onOpenMapPanel }: Props) {
  return (
    <div className="map-empty-overlay" aria-live="polite">
      <Card padding="spacious" shadow="md" className="map-empty-card">
        <div className="map-empty-card__icon">
          <MapIcon size={28} strokeWidth={1.5} />
        </div>
        <span className="map-empty-card__eyebrow">Mapa</span>
        <p className="map-empty-card__title">Sin mapa cargado</p>
        <p className="map-empty-card__description">
          Inicia un mapeo para construir uno nuevo, o carga uno existente.
        </p>
        <div className="map-empty-card__actions">
          <Button variant="primary" size="md" leadingIcon={MapPin} onClick={onStartMapping}>
            Iniciar mapeo
          </Button>
          <Button variant="ghost" size="md" onClick={onOpenMapPanel}>
            Cargar mapa
          </Button>
        </div>
      </Card>
    </div>
  );
}
