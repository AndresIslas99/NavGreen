import { useState, useCallback } from 'react'

interface Props {
  visible: boolean
}

export function CameraFeed({ visible }: Props) {
  const [expanded, setExpanded] = useState(false)
  const [hasError, setHasError] = useState(false)
  const [depthError, setDepthError] = useState(false)
  const [showDepth, setShowDepth] = useState(true)

  const handleSnapshot = useCallback(async (e: React.MouseEvent) => {
    e.stopPropagation()
    try {
      const res = await fetch('/api/camera/snapshot')
      if (!res.ok) return
      const blob = await res.blob()
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      a.href = url
      a.download = `agv_snapshot_${Date.now()}.jpg`
      a.click()
      URL.revokeObjectURL(url)
    } catch { /* ignore */ }
  }, [])

  if (!visible) return null

  const cameraUrl = `${location.protocol}//${location.host}/api/camera/stream`
  const depthUrl = `${location.protocol}//${location.host}/api/depth/stream`

  return (
    <>
      {/* Camera PIP */}
      <div
        className={`camera-pip ${expanded ? 'camera-pip-expanded' : ''}`}
        onClick={() => setExpanded(!expanded)}
      >
        {hasError ? (
          <div className="camera-pip-placeholder">
            <span>No camera</span>
          </div>
        ) : (
          <img
            src={cameraUrl}
            alt="Camera"
            className="camera-pip-img"
            onError={() => setHasError(true)}
          />
        )}
        {!hasError && (
          <button className="camera-snapshot-btn" onClick={handleSnapshot} title="Save snapshot">
            &#128247;
          </button>
        )}
      </div>

      {/* Depth heatmap PIP */}
      {showDepth && (
        <div
          className="depth-pip"
          onClick={() => setShowDepth(false)}
          title="Depth heatmap (click to hide)"
        >
          {depthError ? (
            <div className="camera-pip-placeholder">
              <span>No depth</span>
            </div>
          ) : (
            <img
              src={depthUrl}
              alt="Depth"
              className="camera-pip-img"
              onError={() => setDepthError(true)}
            />
          )}
          <span className="depth-label">Depth</span>
        </div>
      )}
    </>
  )
}
