import { useState, useCallback } from 'react'

interface Props {
  visible: boolean
  expanded?: boolean
}

export function CameraFeed({ visible, expanded: forceExpanded }: Props) {
  const [expanded, setExpanded] = useState(false)
  const [depthExpanded, setDepthExpanded] = useState(false)
  const isExpanded = forceExpanded || expanded
  const [hasError, setHasError] = useState(false)
  const [depthError, setDepthError] = useState(false)

  const handleSnapshot = useCallback(async (e: React.MouseEvent) => {
    e.stopPropagation()
    try {
      const res = await fetch(`${location.protocol}//${location.hostname}:8091/camera/snapshot`)
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

  // C++ image server on port 8091 (OpenCV JPEG, 3-5x faster than Python PIL)
  const imageHost = `${location.protocol}//${location.hostname}:8091`
  const cameraUrl = `${imageHost}/camera/stream`
  const depthUrl = `${imageHost}/depth/stream`

  if (forceExpanded) {
    // Full panel mode during mapping — stacked camera + depth
    return (
      <div className="camera-panel-full">
        <div className="camera-panel-stream">
          {hasError ? (
            <div className="camera-pip-placeholder"><span>No camera</span></div>
          ) : (
            <img src={cameraUrl} alt="Camera" className="camera-panel-img" onError={() => setHasError(true)} />
          )}
          {!hasError && (
            <button className="camera-snapshot-btn" onClick={handleSnapshot} title="Save snapshot">
              &#128247;
            </button>
          )}
        </div>
        <div className="camera-panel-stream">
          {depthError ? (
            <div className="camera-pip-placeholder"><span>No depth</span></div>
          ) : (
            <img src={depthUrl} alt="Depth" className="camera-panel-img" onError={() => setDepthError(true)} />
          )}
          <span className="depth-label">Depth</span>
        </div>
      </div>
    )
  }

  return (
    <>
      {/* Camera PIP */}
      <div
        className={`camera-pip ${isExpanded ? 'camera-pip-expanded' : ''}`}
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
      <div
        className={`depth-pip ${depthExpanded ? 'depth-pip-expanded' : ''}`}
        onClick={() => setDepthExpanded(!depthExpanded)}
        title="Depth heatmap (click to expand)"
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
    </>
  )
}
