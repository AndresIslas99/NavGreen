/**
 * MappingPanel — Control de mapeo SLAM.
 *
 * Reescrito durante Round 2 del audit perfeccionista: pasaba de strings
 * en inglés y clases legacy (`.panel-section`, `.action-btn`) a las
 * primitivas del design system (Section / Button / Stack) y copy en
 * español sentence case. Mantiene la misma lógica: iniciar/detener
 * grabación + joystick + guardar/cargar mapas.
 */
import { useState, useEffect } from 'react'
import type { MapInfo, RobotState, AllowedActions } from '../../api/types'
import { Joystick } from '../Joystick'
import { Section } from '../ui/Section'
import { Button } from '../ui/Button'
import { Stack } from '../ui/Stack'
import { MapPin, Square, Trash2, Save, FolderOpen, RotateCcw } from '../ui/icons'
import * as api from '../../api/client'

interface Props {
  state: RobotState
  actions: AllowedActions
  motorsArmed: boolean
  onModeChange: (mode: string) => void
  onRecording: (action: 'start' | 'stop') => void
  onCmdVel: (linear: number, angular: number) => void
  recordingResult?: { success: boolean; message: string } | null
}

export function MappingPanel({
  state, actions, motorsArmed, onModeChange, onRecording, onCmdVel, recordingResult,
}: Props) {
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [saveName, setSaveName] = useState('')
  const [selectedMap, setSelectedMap] = useState('')
  const [busy, setBusy] = useState(false)
  const [msg, setMsg] = useState('')

  const refresh = () => api.listMaps().then(setMaps).catch(() => {})
  useEffect(() => { refresh() }, [])

  const handleStartMapping = () => {
    onModeChange('mapping')
    onRecording('start')
  }

  const handleStopMapping = () => {
    onRecording('stop')
    onModeChange('teleop')
  }

  const handleSave = async () => {
    if (!saveName.trim()) return
    setBusy(true); setMsg('')
    const r = await api.saveMap(saveName.trim()) as { success?: boolean }
    setBusy(false)
    setMsg(r.success ? 'Mapa guardado' : 'No se pudo guardar')
    if (r.success) { setSaveName(''); refresh() }
    setTimeout(() => setMsg(''), 3000)
  }

  const handleLoad = async () => {
    if (!selectedMap) return
    setBusy(true); setMsg('')
    const r = await api.loadMap(selectedMap) as { success?: boolean }
    setBusy(false)
    setMsg(r.success ? 'Mapa cargado' : 'No se pudo cargar')
    setTimeout(() => setMsg(''), 3000)
  }

  const mapping = state === 'mapping'

  return (
    <div className="cockpit-panel context-panel">
      <Section title="Control de mapeo">
        <Stack gap={2}>
          {!mapping ? (
            <Button
              variant="primary"
              size="lg"
              block
              leadingIcon={MapPin}
              disabled={!actions.canStartMapping}
              onClick={handleStartMapping}
            >
              Iniciar mapeo
            </Button>
          ) : (
            <Button
              variant="destructive"
              size="lg"
              block
              leadingIcon={Square}
              onClick={handleStopMapping}
            >
              Detener mapeo
            </Button>
          )}
          {mapping && (
            <Button
              variant="secondary"
              size="md"
              block
              leadingIcon={Trash2}
              onClick={() => api.clearAccMap()}
            >
              Limpiar acumulador
            </Button>
          )}
          {recordingResult && (
            <p
              className="ui-section__description"
              role="status"
              style={recordingResult.success ? undefined : { color: 'var(--crit)' }}
            >
              {recordingResult.success ? 'Grabación guardada' : `Error: ${recordingResult.message}`}
            </p>
          )}
        </Stack>
      </Section>

      <Section title="Conducción" description={!motorsArmed ? 'Activa los motores desde el panel Recuperar.' : undefined}>
        <Joystick
          enabled={motorsArmed && (state === 'mapping' || state === 'ready')}
          maxLinear={0.4}
          maxAngular={0.2}
          onMove={onCmdVel}
        />
      </Section>

      <Section title="Guardar mapa">
        <div className="cockpit-input-row">
          <input
            className="cockpit-input"
            type="text"
            placeholder="Nombre del mapa"
            value={saveName}
            onChange={e => setSaveName(e.target.value)}
            disabled={busy}
          />
          <Button
            variant="primary"
            size="md"
            leadingIcon={Save}
            disabled={!saveName.trim() || busy || !actions.canSaveMap}
            onClick={handleSave}
          >
            Guardar
          </Button>
        </div>
      </Section>

      <Section
        title="Cargar mapa"
        actions={
          <Button
            variant="ghost"
            size="sm"
            leadingIcon={RotateCcw}
            onClick={refresh}
            title="Actualizar lista de mapas"
            aria-label="Actualizar lista de mapas"
          >
            <span className="visually-hidden">Actualizar</span>
          </Button>
        }
      >
        <div className="cockpit-input-row">
          <select
            className="cockpit-input"
            value={selectedMap}
            onChange={e => setSelectedMap(e.target.value)}
            disabled={busy}
          >
            <option value="">Elige un mapa…</option>
            {maps.map(m => <option key={m.name} value={m.name}>{m.name}</option>)}
          </select>
          <Button
            variant="primary"
            size="md"
            leadingIcon={FolderOpen}
            disabled={!selectedMap || busy || !actions.canLoadMap}
            onClick={handleLoad}
          >
            Cargar
          </Button>
        </div>
        {msg && <p className="ui-section__description" role="status">{msg}</p>}
      </Section>
    </div>
  )
}
