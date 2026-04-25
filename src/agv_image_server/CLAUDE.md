# agv_image_server

C++17 HTTP server that streams camera and depth images as MJPEG. Uses raw TCP sockets
with OpenCV JPEG encoding (libjpeg-turbo with NEON acceleration on ARM). 3-5x faster
than Python PIL equivalent.

## Nodes

- **image_server_node** (C++17): HTTP server accepting MJPEG stream and snapshot requests

## HTTP Endpoints

- `GET /camera/stream` — MJPEG multipart stream of RGB camera (100ms frame delay)
- `GET /camera/snapshot` — Single JPEG frame
- `GET /depth/stream` — MJPEG heatmap of depth data (200ms frame delay)
- `GET /depth/snapshot` — Single depth heatmap JPEG

## Topics

**Subscribed:**
- Camera topic (sensor_msgs/Image, BGR8) — Default: `/zed/zed_node/right/image_rect_color`
- Depth topic (sensor_msgs/Image, 32FC1) — Default: `/zed/zed_node/depth/depth_registered`

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `port` | `8091` | HTTP server port |
| `camera_topic` | `/zed/zed_node/right/image_rect_color` | RGB camera topic |
| `depth_topic` | `/zed/zed_node/depth/depth_registered` | Depth image topic |
| `jpeg_quality` | `70` | JPEG compression quality (1-100) |
| `max_width` | `640` | Maximum output width (pixels) |
| `max_concurrent_streams` | `4` | Cap on simultaneous MJPEG streams. Excess clients receive HTTP 503 Retry-After:2. Snapshots are not capped (synchronous, fast). Sprint 1 Fase A1 (2026-04-24). |

## Key Implementation Details

- Raw TCP socket HTTP server (no HTTP library dependency)
- MJPEG clients are bounded by `max_concurrent_streams` (default 4). Each
  active stream runs in a detached thread; an `std::atomic<int>
  active_streams_` counter enforces the cap before spawning. Excess clients
  get HTTP 503 + `Retry-After: 2` so the browser/UI backs off cleanly.
- Depth processing: 4x downsampling -> normalize 0.3-10m -> inverted JET colormap (red=near, blue=far)
- NaN depth values mapped to 10m (max range)
- MJPEG framing: HTTP multipart/x-mixed-replace with boundary markers

## Dependencies

- OpenCV (imgcodecs, imgproc), rclcpp, sensor_msgs

## Improvement Opportunities

- Add TASK.yaml (only package without one)
- ~~Add connection limit~~ — done in Sprint 1 Fase A1 (`max_concurrent_streams`)
- Add unit tests for image encoding pipeline
- Consider replacing raw sockets with lightweight HTTP library for robustness
- Add configurable frame rate per endpoint
- Sprint 2: replace MJPEG with H.264 NVENC via `isaac_ros_h264_encoder` to free CUDA cores and reduce bandwidth ~10-20× over WiFi.
