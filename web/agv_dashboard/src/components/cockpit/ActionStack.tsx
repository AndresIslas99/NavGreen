/**
 * ActionStack — vertical column of big primary actions in the mission cockpit:
 * PAUSAR MISIÓN, REANUDAR, IR A BASE, CANCELAR TAREA.
 *
 * Each button is gated by the backend's AllowedActions plus contextual state
 * (mission_progress, home_point). When a button can't be safely pressed, it's
 * disabled with a tooltip explaining why — never hidden, so the layout stays
 * stable and operators learn the gating rules.
 */
import { useState } from 'react';
import type { AllowedActions, MissionProgress, HomePoint } from '../../api/types';
import * as api from '../../api/client';

interface Props {
  actions: AllowedActions;
  missionProgress: MissionProgress | null;
  homePoint: HomePoint | null;
  navActive: boolean;
  onCancelNav: () => void;
}

type BtnState = 'idle' | 'busy' | 'ok' | 'err';

interface ActionBtnProps {
  label: string;
  variant: 'primary' | 'home' | 'danger';
  disabled: boolean;
  disabledReason?: string;
  onClick: () => unknown | Promise<unknown>;
}

function ActionBtn({ label, variant, disabled, disabledReason, onClick }: ActionBtnProps) {
  const [state, setState] = useState<BtnState>('idle');
  const handle = async () => {
    if (disabled) return;
    setState('busy');
    try {
      await onClick();
      setState('ok');
      window.setTimeout(() => setState('idle'), 800);
    } catch {
      setState('err');
      window.setTimeout(() => setState('idle'), 1400);
    }
  };
  return (
    <button
      className={`cockpit-action-btn cockpit-action-btn--${variant} cockpit-action-btn--${state}`}
      disabled={disabled || state === 'busy'}
      title={disabled ? disabledReason : undefined}
      onClick={handle}
    >
      {label}
    </button>
  );
}

export function ActionStack({ actions, missionProgress, homePoint, navActive, onCancelNav }: Props) {
  const missionRunning = missionProgress?.status === 'running';
  const missionPaused  = missionProgress?.status === 'paused';
  const canPause   = !!actions.canPauseMission && missionRunning;
  const canResume  = !!actions.canPauseMission && missionPaused;
  const canGoHome  = !!homePoint && !!actions.canSendGoal && !missionRunning;
  const canCancel  = !!actions.canCancelNav || missionRunning || missionPaused || navActive;

  return (
    <div className="cockpit-section">
      <div className="cockpit-eyebrow">ACCIONES</div>
      <div className="action-stack">
        <ActionBtn
          label="PAUSAR MISIÓN"
          variant="primary"
          disabled={!canPause}
          disabledReason={
            !missionProgress ? 'Sin misión activa.' :
            !missionRunning   ? `La misión está en estado: ${missionProgress.status}.` :
            'Pausa no permitida en el estado actual.'
          }
          onClick={() => api.pauseMission()}
        />

        <ActionBtn
          label="REANUDAR"
          variant="primary"
          disabled={!canResume}
          disabledReason={
            !missionProgress ? 'Sin misión activa.' :
            !missionPaused    ? `La misión no está pausada (estado: ${missionProgress.status}).` :
            'Reanudación no permitida en el estado actual.'
          }
          onClick={() => api.resumeMission()}
        />

        <ActionBtn
          label="IR A BASE"
          variant="home"
          disabled={!canGoHome}
          disabledReason={
            !homePoint       ? 'Sin base definida — fija una en el panel de cockpit.' :
            missionRunning   ? 'Cancela la misión activa antes de ir a base.' :
            !actions.canSendGoal ? 'Goals de navegación no permitidos en el estado actual.' :
            'Acción no permitida.'
          }
          onClick={() => api.goHome()}
        />

        <ActionBtn
          label="CANCELAR TAREA"
          variant="danger"
          disabled={!canCancel}
          disabledReason="No hay misión ni nav goal activos."
          onClick={async () => {
            // Cancel both mission (if any) and any active nav goal. The
            // backend safely no-ops on whichever isn't active.
            if (missionRunning || missionPaused) await api.cancelGoal();
            if (navActive) onCancelNav();
            else await api.cancelGoal();
          }}
        />
      </div>
    </div>
  );
}
