/**
 * Toast — non-blocking status messages for action results.
 *
 * Patterns:
 *   - Success (Home point saved) → tone="ok", auto-dismiss 3 s.
 *   - Error (Save failed: …) → tone="crit", auto-dismiss 6 s, sticky on hover.
 *   - Info (Mission paused) → tone="info", auto-dismiss 4 s.
 *
 * Mounted via <ToastProvider/> at the app root; components anywhere can call
 * useToast().push({ tone, message }) to surface one. No portal — toasts
 * stack in a fixed bottom-right region.
 */
import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react';
import type { ReactNode } from 'react';
import { CheckCircle2, AlertCircle, AlertOctagon, X } from './icons';

type Tone = 'ok' | 'warn' | 'crit' | 'info';

interface ToastEntry {
  id: number;
  tone: Tone;
  title: string;
  description?: string;
  durationMs: number;
}

interface ToastPushOptions {
  tone?: Tone;
  title: string;
  description?: string;
  durationMs?: number;
}

interface ToastContextValue {
  push: (opts: ToastPushOptions) => void;
  dismiss: (id: number) => void;
}

const ToastContext = createContext<ToastContextValue | null>(null);

const TONE_DEFAULT_DURATION: Record<Tone, number> = {
  ok: 3000, info: 4000, warn: 5000, crit: 6000,
};

const TONE_ICON = {
  ok:   CheckCircle2,
  info: AlertCircle,
  warn: AlertCircle,
  crit: AlertOctagon,
};

export function ToastProvider({ children }: { children: ReactNode }) {
  const [toasts, setToasts] = useState<ToastEntry[]>([]);
  const nextId = useRef(1);

  const dismiss = useCallback((id: number) => {
    setToasts(prev => prev.filter(t => t.id !== id));
  }, []);

  const push = useCallback((opts: ToastPushOptions) => {
    const id = nextId.current++;
    const tone = opts.tone ?? 'info';
    const entry: ToastEntry = {
      id,
      tone,
      title: opts.title,
      description: opts.description,
      durationMs: opts.durationMs ?? TONE_DEFAULT_DURATION[tone],
    };
    setToasts(prev => [...prev, entry]);
  }, []);

  const value = useMemo(() => ({ push, dismiss }), [push, dismiss]);

  return (
    <ToastContext.Provider value={value}>
      {children}
      <div className="ui-toast-region" aria-live="polite" aria-atomic="false">
        {toasts.map(t => (
          <ToastItem key={t.id} entry={t} onDismiss={() => dismiss(t.id)} />
        ))}
      </div>
    </ToastContext.Provider>
  );
}

function ToastItem({ entry, onDismiss }: { entry: ToastEntry; onDismiss: () => void }) {
  useEffect(() => {
    const t = window.setTimeout(onDismiss, entry.durationMs);
    return () => window.clearTimeout(t);
  }, [entry.durationMs, onDismiss]);

  const Icon = TONE_ICON[entry.tone];

  return (
    <div className={`ui-toast ui-toast--${entry.tone}`} role="status">
      <Icon size={20} className="ui-toast__icon" strokeWidth={2} />
      <div className="ui-toast__body">
        <p className="ui-toast__title">{entry.title}</p>
        {entry.description && <p className="ui-toast__description">{entry.description}</p>}
      </div>
      <button
        type="button"
        className="ui-toast__dismiss"
        onClick={onDismiss}
        aria-label="Cerrar notificación"
      >
        <X size={16} />
      </button>
    </div>
  );
}

export function useToast(): ToastContextValue {
  const ctx = useContext(ToastContext);
  if (!ctx) {
    throw new Error('useToast must be used within a <ToastProvider/>');
  }
  return ctx;
}
