/**
 * RecoveryPanel — safety controls + subsystem health + raw diagnostic state.
 *
 * Rewritten on top of the design system primitives (Section, Button, Pill)
 * with Spanish copy throughout. Previously stuck in English with legacy
 * `.panel-section` markup — the only panel that hadn't received the
 * light-minimal refresh.
 */
import type { RobotStatus, RobotState, HealthMap } from '../../api/types'
import { Section, Button, Pill } from '../ui'
import { AlertOctagon, Power, Zap, XOctagon, CheckCircle2, AlertCircle, AlertTriangle } from '../ui/icons'

interface Props {
  status: RobotStatus | null
  state: RobotState
  health: HealthMap
  onEStop: (active: boolean) => void
  onMotorEnable: (active: boolean) => void
  onNavCancel: () => void
}

const SUBSYSTEM_LABELS: Record<string, string> = {
  drive:   'Tracción / Odometría',
  imu:     'IMU / Odometría global',
  slam:    'SLAM visual',
  nav:     'Stack Nav2',
  network: 'Red',
}

type HealthTone = 'accent' | 'warn' | 'crit' | 'neutral'
const HEALTH_META: Record<string, { tone: HealthTone; label: string; icon: typeof CheckCircle2 }> = {
  ok:      { tone: 'accent',  label: 'OK',          icon: CheckCircle2 },
  warn:    { tone: 'warn',    label: 'Degradado',   icon: AlertTriangle },
  error:   { tone: 'crit',    label: 'Falla',       icon: AlertOctagon },
  unknown: { tone: 'neutral', label: 'Sin datos',   icon: AlertCircle },
}

const STATE_LABELS: Record<RobotState, string> = {
  offline:           'Sin conexión',
  idle:              'En espera',
  ready:             'Listo',
  mapping:           'Mapeando',
  navigating:        'Navegando',
  executing_mission: 'En misión',
  blocked:           'Bloqueado',
  e_stop:            'Paro activo',
  fault:             'Falla',
}

export function RecoveryPanel({ status, state, health, onEStop, onMotorEnable, onNavCancel }: Props) {
  const s = status

  return (
    <div className="context-panel cockpit-panel">
      <Section title="Paro de emergencia">
        <Button
          variant={s?.e_stop ? 'primary' : 'destructive'}
          size="lg"
          block
          leadingIcon={AlertOctagon}
          onClick={() => onEStop(!s?.e_stop)}
        >
          {s?.e_stop ? 'Liberar paro' : 'Activar paro'}
        </Button>
      </Section>

      <Section title="Motores">
        <Button
          variant={s?.motors_armed ? 'primary' : 'secondary'}
          size="lg"
          block
          leadingIcon={s?.motors_armed ? Zap : Power}
          onClick={() => onMotorEnable(!s?.motors_armed)}
        >
          {s?.motors_armed ? 'Desactivar motores' : 'Activar motores'}
        </Button>
      </Section>

      {s?.nav_state?.active && (
        <Section title="Navegación">
          <Button
            variant="destructive"
            size="lg"
            block
            leadingIcon={XOctagon}
            onClick={onNavCancel}
          >
            Cancelar navegación
          </Button>
        </Section>
      )}

      <Section title="Salud de subsistemas">
        <div className="health-grid">
          {Object.entries(health).map(([key, h]) => {
            const meta = HEALTH_META[h.status] ?? HEALTH_META.unknown
            const Icon = meta.icon
            return (
              <div key={key} className="health-row">
                <span className={`health-dot health-dot--${meta.tone}`} aria-hidden="true" />
                <div className="health-info">
                  <div className="health-info-head">
                    <span className="health-name">{SUBSYSTEM_LABELS[key] || key}</span>
                    <Pill tone={meta.tone} size="xs" leadingIcon={Icon}>{meta.label}</Pill>
                  </div>
                  {h.detail && <span className="health-detail">{h.detail}</span>}
                  {h.action && h.status !== 'ok' && (
                    <span className="health-action">{h.action}</span>
                  )}
                </div>
              </div>
            )
          })}
        </div>
      </Section>

      <Section title="Diagnóstico">
        <div className="diag-grid">
          <span className="diag-label">Estado del robot</span>
          <span className="diag-value">{STATE_LABELS[state] || state}</span>
          <span className="diag-label">Modo</span>
          <span className="diag-value">{s?.mode === 'teleop' ? 'Manual' : s?.mode === 'nav' ? 'Navegación' : s?.mode === 'mapping' ? 'Mapeo' : '—'}</span>
          <span className="diag-label">Motor I / D</span>
          <span className="diag-value">{s?.left_state ?? '—'} / {s?.right_state ?? '—'}</span>
          <span className="diag-label">Clientes</span>
          <span className="diag-value">{s?.clients || 0}</span>
        </div>
      </Section>
    </div>
  )
}
