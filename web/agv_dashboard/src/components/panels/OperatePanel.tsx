/**
 * OperatePanel — the "mission cockpit" rebuild.
 *
 * Vertical column of sections, top to bottom:
 *   1. Control mode rail (TELEOP / NAV pills)
 *   2. Action stack (PAUSAR / REANUDAR / IR A BASE / CANCELAR)
 *   3. Task info card (mode-aware visualization)
 *   4. Home point hint (set / change base)
 *   5. Motors arm/disarm row
 *   6. Joystick (always rendered; only enabled in teleop)
 *
 * Existing safety behaviors are preserved: the joystick gating still respects
 * actions.canTeleop and mode='teleop', the motor button still respects
 * actions.canMotorEnable. No flows were removed.
 */
import { useCallback } from 'react';
import { Joystick } from '../Joystick';
import { ControlModeRail } from '../cockpit/ControlModeRail';
import { ActionStack } from '../cockpit/ActionStack';
import { TaskInfoCard } from '../cockpit/TaskInfoCard';
import { HomePointHint } from '../cockpit/HomePointHint';
import { Section, Button } from '../ui';
import { Zap, Power } from '../ui/icons';
import type { AllowedActions, RobotStatus, HomePoint } from '../../api/types';

interface Props {
  actions: AllowedActions;
  motorsArmed: boolean;
  mode: string;
  status: RobotStatus | null;
  onCmdVel: (linear: number, angular: number) => void;
  onMotorEnable: (active: boolean) => void;
  onModeChange: (mode: string) => void;
  onCancelNav: () => void;
  onHomePointSet?: (hp: HomePoint) => void;
}

export function OperatePanel({
  actions, motorsArmed, mode, status,
  onCmdVel, onMotorEnable, onModeChange, onCancelNav, onHomePointSet,
}: Props) {
  const handleModeChange = useCallback((m: string) => onModeChange(m), [onModeChange]);

  const missionProgress = status?.mission_progress ?? null;
  const homePoint = status?.home_point ?? null;
  const navActive = !!status?.nav_state?.active;
  const pose = status?.pose ?? null;

  // Whether mode switching is allowed at all. Backend gates via canTeleop /
  // canSendGoal indirectly; we expose a simple "true unless mission is
  // actively driving" rule so the operator can't switch modes mid-mission.
  const canChangeMode =
    !navActive &&
    !(missionProgress?.status === 'running');

  return (
    <div className="context-panel cockpit-panel">
      <ControlModeRail mode={mode} canChange={canChangeMode} onChange={handleModeChange} />

      <ActionStack
        actions={actions}
        missionProgress={missionProgress}
        homePoint={homePoint}
        navActive={navActive}
        onCancelNav={onCancelNav}
      />

      <TaskInfoCard mode={mode} status={status} missionProgress={missionProgress} />

      <HomePointHint
        homePoint={homePoint}
        currentPose={pose}
        onSet={hp => onHomePointSet?.(hp)}
      />

      <Section title="Motores">
        <Button
          variant={motorsArmed ? 'primary' : 'secondary'}
          size="lg"
          block
          leadingIcon={motorsArmed ? Zap : Power}
          disabled={!actions.canMotorEnable && !motorsArmed}
          onClick={() => onMotorEnable(!motorsArmed)}
          title={
            !motorsArmed && !actions.canMotorEnable
              ? 'Activar motores no permitido en el estado actual'
              : motorsArmed ? 'Desactivar motores' : 'Activar motores'
          }
        >
          {motorsArmed ? 'Desactivar motores' : 'Activar motores'}
        </Button>
      </Section>

      <Section title="Joystick" description={mode === 'nav' ? 'Disponible en modo Manual' : undefined}>
        <Joystick
          enabled={actions.canTeleop && mode === 'teleop'}
          maxLinear={0.5}
          maxAngular={0.5}
          onMove={onCmdVel}
        />
        {mode === 'nav' && (
          <p className="cockpit-hint">Click en el mapa para enviar un goal de navegación</p>
        )}
      </Section>
    </div>
  );
}
