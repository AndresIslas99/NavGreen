/**
 * ConnectionBanner — full-width banner shown when the WebSocket disconnects.
 *
 * Slides in from the top above the topbar so operators can't miss it. Doesn't
 * appear during the initial connect (waits ~3 s before first showing) so brief
 * boot races don't trigger alarm.
 *
 * Auto-hides when connected returns true.
 */
import { useEffect, useState } from 'react';
import { WifiOff } from './ui/icons';

interface Props {
  connected: boolean;
}

export function ConnectionBanner({ connected }: Props) {
  const [show, setShow] = useState(false);

  useEffect(() => {
    if (connected) {
      setShow(false);
      return;
    }
    // 3 s grace so a quick reconnect doesn't flash the banner.
    const t = window.setTimeout(() => setShow(true), 3000);
    return () => window.clearTimeout(t);
  }, [connected]);

  if (!show) return null;

  return (
    <div className="connection-banner" role="alert" aria-live="assertive">
      <WifiOff size={16} strokeWidth={2} />
      <span className="connection-banner__text">
        Sin conexión con el robot — intentando reconectar…
      </span>
    </div>
  );
}
