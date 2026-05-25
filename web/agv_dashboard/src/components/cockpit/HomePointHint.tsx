/**
 * HomePointHint — inline panel for managing the operator's "home / base"
 * pose. Two presentations:
 *
 *   - No home defined → EmptyState with "Fijar base aquí" CTA.
 *   - Home defined    → calm passive card showing the saved name + coords +
 *                       a "Cambiar" ghost button to redefine.
 *
 * Modal capture uses the Card primitive on a modal-overlay backdrop.
 */
import { useState } from 'react';
import type { HomePoint } from '../../api/types';
import * as api from '../../api/client';
import { Section } from '../ui/Section';
import { Card } from '../ui/Card';
import { Button } from '../ui/Button';
import { EmptyState } from '../ui/EmptyState';
import { Home, X } from '../ui/icons';

interface Props {
  homePoint: HomePoint | null;
  currentPose: { x: number; y: number; theta: number } | null;
  onSet: (hp: HomePoint) => void;
}

export function HomePointHint({ homePoint, currentPose, onSet }: Props) {
  const [modalOpen, setModalOpen] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleConfirm = async (name: string) => {
    if (!currentPose) return;
    setBusy(true); setError(null);
    try {
      const resp: any = await api.setHomePoint({ ...currentPose, name });
      if (resp?.home_point) onSet(resp.home_point);
      setModalOpen(false);
    } catch (e: any) {
      setError(e?.message || 'No se pudo guardar la base');
    } finally {
      setBusy(false);
    }
  };

  return (
    <Section title="Base">
      {homePoint ? (
        <Card padding="default" className="home-point-card home-point-card--set">
          <div className="home-point-card__body">
            <div className="home-point-card__icon"><Home size={18} strokeWidth={1.8} /></div>
            <div className="home-point-card__text">
              <span className="home-point-card__name">{homePoint.name}</span>
              <span className="home-point-card__coords">
                ({homePoint.x.toFixed(1)}, {homePoint.y.toFixed(1)})
              </span>
            </div>
          </div>
          <Button variant="ghost" size="sm" onClick={() => setModalOpen(true)}>
            Cambiar
          </Button>
        </Card>
      ) : (
        <EmptyState
          icon={Home}
          title="Sin base definida"
          description={
            <>
              Marca la pose actual del robot como la base/cargador para
              habilitar <strong>Ir a base</strong>.
            </>
          }
          action={
            <Button
              variant="primary"
              size="sm"
              disabled={!currentPose}
              onClick={() => setModalOpen(true)}
            >
              Fijar base aquí
            </Button>
          }
          compact
        />
      )}

      {modalOpen && (
        <HomeModal
          currentPose={currentPose}
          initialName={homePoint?.name ?? 'Base'}
          busy={busy}
          error={error}
          onClose={() => { setModalOpen(false); setError(null); }}
          onConfirm={handleConfirm}
        />
      )}
    </Section>
  );
}

interface ModalProps {
  currentPose: { x: number; y: number; theta: number } | null;
  initialName: string;
  busy: boolean;
  error: string | null;
  onClose: () => void;
  onConfirm: (name: string) => void;
}

function HomeModal({ currentPose, initialName, busy, error, onClose, onConfirm }: ModalProps) {
  const [name, setName] = useState(initialName);

  return (
    <div className="modal-overlay" onClick={onClose} role="dialog" aria-modal="true">
      <Card
        as="div"
        padding="spacious"
        shadow="md"
        className="modal-card"
        onClick={(e: React.MouseEvent) => e.stopPropagation()}
      >
        <div className="modal-card__header">
          <h3 className="modal-card__title">Fijar punto base</h3>
          <Button variant="ghost" size="sm" leadingIcon={X} onClick={onClose} aria-label="Cerrar" />
        </div>
        <p className="modal-card__body-text">
          Guarda la pose actual del robot como base/cargador. El botón
          <strong> Ir a base </strong>
          enviará un goal de navegación a esta pose.
        </p>

        {currentPose ? (
          <p className="modal-card__pose">
            Pose actual: x={currentPose.x.toFixed(2)} m · y={currentPose.y.toFixed(2)} m · θ={currentPose.theta.toFixed(2)} rad
          </p>
        ) : (
          <p className="modal-card__pose modal-card__pose--err">
            Pose no disponible (sin conexión).
          </p>
        )}

        <label htmlFor="home-name-input" className="modal-card__label">Nombre</label>
        <input
          id="home-name-input"
          className="modal-card__input"
          type="text"
          value={name}
          onChange={e => setName(e.target.value)}
          autoFocus
          placeholder="Base"
        />

        {error && <p className="modal-card__error">{error}</p>}

        <div className="modal-card__footer">
          <Button variant="ghost" onClick={onClose} disabled={busy}>Cancelar</Button>
          <Button
            variant="primary"
            onClick={() => onConfirm(name.trim() || 'Base')}
            disabled={busy || !currentPose}
            loading={busy}
          >
            Guardar base
          </Button>
        </div>
      </Card>
    </div>
  );
}
