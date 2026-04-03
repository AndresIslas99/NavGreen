import * as http from 'http';
import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps, node: any): void {
  const CAMERA_TOPIC = process.env.AGV_CAMERA_TOPIC ||
    `/${deps.config.namespace}/zed/left/image_rect_color/compressed`;
  let cameraJpeg: Buffer | null = null;
  const cameraClients = new Set<http.ServerResponse>();
  let cameraSubCreated = false;

  function ensureCameraSub() {
    if (cameraSubCreated) return;
    cameraSubCreated = true;
    try {
      node.createSubscription('sensor_msgs/msg/CompressedImage', CAMERA_TOPIC, (msg: any) => {
        cameraJpeg = Buffer.from(msg.data);
        for (const client of cameraClients) {
          try {
            client.write(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${cameraJpeg.length}\r\n\r\n`);
            client.write(cameraJpeg);
            client.write('\r\n');
          } catch {
            cameraClients.delete(client);
          }
        }
      });
    } catch { /* topic may not exist */ }
  }

  app.get('/api/camera/stream', (req, res) => {
    ensureCameraSub();
    res.writeHead(200, {
      'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
      'Cache-Control': 'no-cache',
      'Connection': 'keep-alive',
    });
    cameraClients.add(res);
    req.on('close', () => cameraClients.delete(res));
  });

  app.get('/api/camera/snapshot', (_req, res) => {
    ensureCameraSub();
    if (cameraJpeg) {
      res.type('image/jpeg').send(cameraJpeg);
    } else {
      res.status(404).json({ error: 'No camera frame available' });
    }
  });
}
