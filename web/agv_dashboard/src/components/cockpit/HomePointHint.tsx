/**
 * HomePointHint — small inline CTA that appears when no home/base point is
 * defined. Lets the operator capture the robot's current pose as the base
 * with a single click + confirmation.
 *
 * Why a separate component: the empty state for the home point isn't just
 * "hide the button" — the IR A BASE button stays *visible but disabled* so
 * the layout doesn't reflow, and this hint sits next to the action stack to
 * teach the operator how to enable it.
 */

import { useState } from 'react';
import type { HomePoint } from '../../api/types';
import * as api from '../../api/client';

interface Props {
  homePoint: HomePoint | null;
  currentPose: { x: number; y: number; theta: number } | null;
  onSet: (hp: HomePoint) => void;
}

export function HomePointHint({ homePoint, currentPose, onSet }: Props) {
  const [open, setOpen] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  if (homePoint) {
    // When the home point is already set, render a tiny passive line so the
    // operator can see WHAT the base is + has a quick way to redefine it.
    return (
      <div className="home-point-hint home-point-hint--set">
        <span>
          Base: <strong>{homePoint.name}</strong>
          {' '}<span className="home-point-coords">({homePoint.x.toFixed(1)}, {homePoint.y.toFixed(1)})</span>
        </span>
        <button className="home-point-btn-secondary" onClick={() => setOpen(true)}>
          Cambiar
        </button>
        {open && (
          <HomeModal
            currentPose={currentPose}
            initialName={homePoint.name}
            busy={busy}
            error={error}
            onClose={() => { setOpen(false); setError(null); }}
            onConfirm={async (name) => {
              if (!currentPose) return;
              setBusy(true); setError(null);
              try {
                const resp: any = await api.setHomePoint({ ...currentPose, name });
                if (resp?.home_point) onSet(resp.home_point);
                setOpen(false);
              } catch (e: any) {
                setError(e?.message || 'No se pudo guardar la base');
              } finally {
                setBusy(false);
              }
            }}
          />
        )}
      </div>
    );
  }

  return (
    <div className="home-point-hint home-point-hint--empty">
      <span>Sin base definida — fija aquí para habilitar <strong>IR A BASE</strong>.</span>
      <button className="home-point-btn-primary"
              disabled={!currentPose}
              onClick={() => setOpen(true)}>
        Fijar base aquí
      </button>
      {open && (
        <HomeModal
          currentPose={currentPose}
          initialName="Base"
          busy={busy}
          error={error}
          onClose={() => { setOpen(false); setError(null); }}
          onConfirm={async (name) => {
            if (!currentPose) return;
            setBusy(true); setError(null);
            try {
              const resp: any = await api.setHomePoint({ ...currentPose, name });
              if (resp?.home_point) onSet(resp.home_point);
              setOpen(false);
            } catch (e: any) {
              setError(e?.message || 'No se pudo guardar la base');
            } finally {
              setBusy(false);
            }
          }}
        />
      )}
    </div>
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
    <div className="modal-overlay" onClick={onClose}>
      <div className="modal-body" onClick={e => e.stopPropagation()}>
        <h3 style={{ margin: '0 0 8px 0' }}>Fijar punto base</h3>
        <p style={{ fontSize: 13, opacity: 0.85, lineHeight: 1.4, marginBottom: 12 }}>
          Guarda la pose actual del robot como la base/cargador. El botón
          <strong> IR A BASE </strong>
          enviará un nav goal a esta pose.
        </p>
        {currentPose ? (
          <p style={{ fontSize: 12, opacity: 0.7, marginBottom: 12, fontFamily: 'monospace' }}>
            Pose: x={currentPose.x.toFixed(2)} y={currentPose.y.toFixed(2)} θ={currentPose.theta.toFixed(2)}
          </p>
        ) : (
          <p style={{ fontSize: 12, color: 'var(--orange)', marginBottom: 12 }}>
            Pose no disponible (sin conexión al backend).
          </p>
        )}
        <label style={{ display: 'block', fontSize: 13, marginBottom: 6 }}>Nombre</label>
        <input
          type="text"
          value={name}
          onChange={e => setName(e.target.value)}
          style={{ width: '100%', padding: '8px', fontSize: 14, marginBottom: 12 }}
          autoFocus
        />
        {error && (
          <p style={{ fontSize: 12, color: 'var(--red)', marginBottom: 12 }}>{error}</p>
        )}
        <div style={{ display: 'flex', gap: 8, justifyContent: 'flex-end' }}>
          <button onClick={onClose} disabled={busy}>Cancelar</button>
          <button
            onClick={() => onConfirm(name.trim() || 'Base')}
            disabled={busy || !currentPose}
          >
            {busy ? 'Guardando…' : 'Guardar base'}
          </button>
        </div>
      </div>
    </div>
  );
}
