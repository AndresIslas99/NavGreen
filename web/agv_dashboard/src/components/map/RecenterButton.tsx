/**
 * RecenterButton — floating action button that brings the camera back to
 * the robot. Visible only when `cameraMode !== 'follow'` (i.e. the operator
 * has panned/zoomed manually OR the pose has gone stale).
 *
 * Rendered via a portal into <body> so it lives outside .map-bg's stacking
 * context. Inside .map-bg, Leaflet's compositor layers can suppress the
 * paint of sibling absolute-positioned content even with explicit z-index —
 * the portal sidesteps that entirely and the FAB renders cleanly above
 * everything else, sized via fixed-position offsets in CSS.
 *
 * Positioned bottom-right with enough offset to clear the cockpit drawer's
 * left edge (see `--drawer-w-expanded` in global.css). On viewport widths
 * below 900 px (responsive fallback), the drawer becomes a bottom sheet
 * and the FAB shifts to the bottom-left corner — handled in CSS.
 */
import { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';
import { LocateFixed } from '../ui/icons';

interface Props {
  /** Camera state from useCameraFollow. */
  cameraMode: 'follow' | 'manual' | 'frozen';
  /** Click handler — call hook's recenter(). */
  onRecenter: () => void;
}

export function RecenterButton({ cameraMode, onRecenter }: Props) {
  // Show a one-shot label flash the first time the FAB appears in a session.
  // After 4 s it collapses to icon-only so it doesn't keep grabbing attention.
  const [showLabel, setShowLabel] = useState(true);

  useEffect(() => {
    if (cameraMode === 'follow') {
      setShowLabel(true);   // reset for next disengage
      return;
    }
    const t = window.setTimeout(() => setShowLabel(false), 4000);
    return () => window.clearTimeout(t);
  }, [cameraMode]);

  if (cameraMode === 'follow') return null;

  const tone = cameraMode === 'frozen' ? 'frozen' : 'manual';
  const label = cameraMode === 'frozen' ? 'Esperando pose' : 'Centrar en robot';

  return createPortal(
    <button
      type="button"
      className={`recenter-fab recenter-fab--${tone} ${showLabel ? 'recenter-fab--expanded' : ''}`}
      onClick={onRecenter}
      onPointerDown={(e) => e.stopPropagation()}
      aria-label={label}
      title={label}
    >
      <LocateFixed size={20} strokeWidth={2} aria-hidden />
      <span className="recenter-fab__label">{label}</span>
    </button>,
    document.body,
  );
}
