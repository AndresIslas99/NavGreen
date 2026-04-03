"""
Camera and depth image handler — converts ROS Image to JPEG for streaming.
Extracted from teleop_server.py for modularity.
"""

import io
import threading
import numpy as np
from PIL import Image as PILImage


class CameraHandler:
    def __init__(self):
        self.camera_jpeg = None
        self.camera_lock = threading.Lock()
        self.depth_jpeg = None
        self.depth_lock = threading.Lock()

    def handle_image(self, msg):
        """Convert raw Image (RGB8/BGR8/RGBA8) to JPEG."""
        try:
            if msg.encoding in ('rgb8', 'bgr8', 'rgba8'):
                channels = 4 if msg.encoding == 'rgba8' else 3
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
                if msg.encoding == 'bgr8':
                    img = img[:, :, ::-1]
                elif msg.encoding == 'rgba8':
                    img = img[:, :, :3]
            else:
                return

            pil_img = PILImage.fromarray(img)
            if pil_img.width > 640:
                scale = 640 / pil_img.width
                pil_img = pil_img.resize((640, int(pil_img.height * scale)))

            buf = io.BytesIO()
            pil_img.save(buf, format='JPEG', quality=70)
            with self.camera_lock:
                self.camera_jpeg = buf.getvalue()
        except Exception:
            pass

    def handle_depth(self, msg):
        """Convert 32FC1 depth to colorized heatmap JPEG."""
        try:
            if msg.encoding != '32FC1':
                return
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            depth_small = depth[::4, ::4]

            valid = np.isfinite(depth_small)
            depth_clamped = np.clip(depth_small, 0.3, 10.0)
            depth_clamped[~valid] = 10.0
            norm = ((depth_clamped - 0.3) / 9.7).clip(0, 1)

            r = np.where(norm < 0.5, 255, (255 * (1 - norm) * 2).clip(0, 255)).astype(np.uint8)
            g = np.where(norm < 0.5, (255 * norm * 2).clip(0, 255),
                         (255 * (1 - norm) * 2).clip(0, 255)).astype(np.uint8)
            b = np.where(norm > 0.5, (255 * (norm - 0.5) * 2).clip(0, 255), 0).astype(np.uint8)
            r[~valid] = 30; g[~valid] = 30; b[~valid] = 30

            rgb = np.stack([r, g, b], axis=-1)
            pil_img = PILImage.fromarray(rgb)

            buf = io.BytesIO()
            pil_img.save(buf, format='JPEG', quality=60)
            with self.depth_lock:
                self.depth_jpeg = buf.getvalue()
        except Exception:
            pass
