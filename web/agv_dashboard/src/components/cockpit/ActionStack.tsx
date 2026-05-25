/**
 * ActionStack — vertical column of primary actions in the mission cockpit.
 *
 *   Pausar misión   — secondary (only enabled while a mission runs)
 *   Reanudar        — primary green (only enabled while paused)
 *   Ir a base       — primary green (only enabled when home is set + can nav)
 *   Cancelar tarea  — destructive (only enabled when something to cancel)
 *
 * Each button:
 *   - Uses the <Button> ui primitive (variant + size + leadingIcon).
 *   - Shows a tooltip explaining WHY it's disabled (never hides — layout
 *     stays stable; the operator learns the gating rules).
 *   - Surfaces success/error via the toast system (Commit 8).
 */
import type { AllowedActions, MissionProgress, HomePoint } from '../../api/types';
import * as api from '../../api/client';
import { Section } from '../ui/Section';
import { Button } from '../ui/Button';
import { Pause, RotateCcw, Home, XOctagon } from '../ui/icons';

interface Props {
  actions: AllowedActions;
  missionProgress: MissionProgress | null;
  homePoint: HomePoint | null;
  navActive: boolean;
  onCancelNav: () => void;
}

export function ActionStack({
  actions, missionProgress, homePoint, navActive, onCancelNav,
}: Props) {
  const missionRunning = missionProgress?.status === 'running';
  const missionPaused  = missionProgress?.status === 'paused';
  const canPause   = !!actions.canPauseMission && missionRunning;
  const canResume  = !!actions.canPauseMission && missionPaused;
  const canGoHome  = !!homePoint && !!actions.canSendGoal && !missionRunning;
  const canCancel  = !!actions.canCancelNav || missionRunning || missionPaused || navActive;

  const handleCancel = async () => {
    if (missionRunning || missionPaused) await api.cancelGoal();
    if (navActive) onCancelNav();
    else await api.cancelGoal();
  };

  return (
    <Section title="Acciones">
      <div className="action-stack">
        <Button
          variant="secondary"
          size="lg"
          block
          leadingIcon={Pause}
          disabled={!canPause}
          title={
            !missionProgress    ? 'Sin misión activa.' :
            !missionRunning     ? `La misión está ${missionProgress.status}.` :
            'Pausa no permitida en el estado actual.'
          }
          onClick={() => api.pauseMission()}
        >
          Pausar misión
        </Button>

        <Button
          variant="primary"
          size="lg"
          block
          leadingIcon={RotateCcw}
          disabled={!canResume}
          title={
            !missionProgress ? 'Sin misión activa.' :
            !missionPaused   ? `La misión no está pausada (${missionProgress.status}).` :
            'Reanudación no permitida.'
          }
          onClick={() => api.resumeMission()}
        >
          Reanudar
        </Button>

        <Button
          variant="primary"
          size="lg"
          block
          leadingIcon={Home}
          disabled={!canGoHome}
          title={
            !homePoint            ? 'Sin base definida — fíjala en la sección de abajo.' :
            missionRunning        ? 'Cancela la misión antes de ir a base.' :
            !actions.canSendGoal  ? 'Goals de navegación no permitidos ahora.' :
            'Enviar al robot al punto base.'
          }
          onClick={() => api.goHome()}
        >
          Ir a base
        </Button>

        <Button
          variant="destructive"
          size="lg"
          block
          leadingIcon={XOctagon}
          disabled={!canCancel}
          title="Cancelar misión y navegación activa."
          onClick={handleCancel}
        >
          Cancelar tarea
        </Button>
      </div>
    </Section>
  );
}
