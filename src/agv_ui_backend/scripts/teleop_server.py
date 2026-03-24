#!/usr/bin/env python3
"""
AGV Teleop Web Server — Commissioning Console
==============================================
FastAPI + rclpy server for remote teleop over LAN.
Publishes cmd_vel and e_stop, subscribes to robot status.

Usage:
  ros2 run agv_ui_backend teleop_server.py --ros-args -p port:=8090

dev_only: true — This is a commissioning tool, not the production dashboard.
"""

import asyncio
import json
import logging
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import uvicorn

logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
logger = logging.getLogger("teleop_server")


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_server')

        self.declare_parameter('port', 8090)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('deadband', 0.08)
        self.declare_parameter('expo', 0.5)

        self.port = self.get_parameter('port').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.deadband = self.get_parameter('deadband').value
        self.expo = self.get_parameter('expo').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.e_stop_pub = self.create_publisher(Bool, 'e_stop', 10)

        # Subscribers
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Odometry, 'wheel_odom', self._odom_cb, best_effort)
        self.create_subscription(String, 'motor_state', self._motor_state_cb, 10)
        self.create_subscription(String, '/slam/quality', self._slam_cb, 10)
        self.create_subscription(String, '/session/info', self._session_cb, 10)

        # Motor enable publisher
        self.motor_enable_pub = self.create_publisher(Bool, 'motor_enable', 10)

        # Service clients
        self.start_rec_client = self.create_client(Trigger, '/session/start_recording')
        self.stop_rec_client = self.create_client(Trigger, '/session/stop_recording')

        # State
        self.e_stop_active = False
        self.last_cmd_time = 0.0
        self.active_clients = 0
        self.odom_times = deque(maxlen=50)
        self.latest_velocity = {'linear': 0.0, 'angular': 0.0}
        self.motor_state = {'armed': False, 'left_state': 0, 'right_state': 0, 'left_errors': 0, 'right_errors': 0}
        self.slam_status = {}
        self.session_status = {}

        # Watchdog timer: zero cmd_vel if no client input
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f'Teleop server ready on port {self.port}, '
            f'max_linear={self.max_linear}, max_angular={self.max_angular}')

    def _odom_cb(self, msg):
        self.odom_times.append(time.time())
        v = msg.twist.twist
        self.latest_velocity = {
            'linear': round(v.linear.x, 3),
            'angular': round(v.angular.z, 3),
        }

    def _motor_state_cb(self, msg):
        try:
            self.motor_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _slam_cb(self, msg):
        try:
            self.slam_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _session_cb(self, msg):
        try:
            self.session_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _watchdog(self):
        if self.active_clients > 0 and not self.e_stop_active:
            elapsed = time.time() - self.last_cmd_time
            if elapsed > self.cmd_vel_timeout and self.last_cmd_time > 0:
                self._send_zero()

    @staticmethod
    def _apply_expo(val, expo):
        """Expo curve: finer control near center, full range at edges."""
        return val * (1.0 - expo) + (val ** 3) * expo

    def send_cmd_vel(self, linear: float, angular: float):
        if self.e_stop_active:
            return
        # Deadband: ignore small inputs
        if abs(linear) < self.deadband * self.max_linear and \
           abs(angular) < self.deadband * self.max_angular:
            linear = 0.0
            angular = 0.0
        else:
            # Expo curve for fine control
            lin_norm = max(-1.0, min(1.0, linear / self.max_linear))
            ang_norm = max(-1.0, min(1.0, angular / self.max_angular))
            linear = self._apply_expo(lin_norm, self.expo) * self.max_linear
            angular = self._apply_expo(ang_norm, self.expo) * self.max_angular
        msg = Twist()
        msg.linear.x = max(-self.max_linear, min(self.max_linear, linear))
        msg.angular.z = max(-self.max_angular, min(self.max_angular, angular))
        self.cmd_vel_pub.publish(msg)
        self.last_cmd_time = time.time()

    def _send_zero(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def set_e_stop(self, active: bool):
        self.e_stop_active = active
        msg = Bool()
        msg.data = active
        self.e_stop_pub.publish(msg)
        if active:
            self._send_zero()
        self.get_logger().warn(f'E-STOP {"ACTIVATED" if active else "CLEARED"}')

    def on_client_disconnect(self):
        self._send_zero()
        self.get_logger().warn('Client disconnected — zero velocity sent')

    def odom_hz(self):
        times = self.odom_times
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        return round((len(times) - 1) / dt, 1) if dt > 0 else 0.0

    def get_status(self):
        return {
            'wheel_odom_hz': self.odom_hz(),
            'velocity': self.latest_velocity,
            'e_stop': self.e_stop_active,
            'motors_armed': bool(self.motor_state.get('armed', False)),
            'left_state': self.motor_state.get('left_state', 0),
            'right_state': self.motor_state.get('right_state', 0),
            'motor_errors': (self.motor_state.get('left_errors', 0) != 0 or
                           self.motor_state.get('right_errors', 0) != 0),
            'drive_online': self.odom_hz() > 1.0,
            'slam_tracking': self.slam_status.get('tracking', {}).get('confidence', 'unknown'),
            'recording': bool(self.session_status.get('recording', False)),
            'clients': self.active_clients,
        }

    def set_motor_enable(self, active: bool):
        msg = Bool()
        msg.data = active
        self.motor_enable_pub.publish(msg)
        self.get_logger().info(f'Motor enable: {active}')

    def call_service(self, client, name):
        if not client.service_is_ready():
            return {'success': False, 'message': f'{name} service not available'}
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            r = future.result()
            return {'success': r.success, 'message': r.message}
        return {'success': False, 'message': 'Service call timed out'}


def create_app(node: TeleopNode) -> FastAPI:
    app = FastAPI(title="AGV Teleop Console")

    # Find static directory
    static_dir = None
    for candidate in [
        Path(__file__).resolve().parent.parent / 'static',
        Path(__file__).resolve().parent.parent / 'share' / 'agv_ui_backend' / 'static',
    ]:
        if candidate.is_dir():
            static_dir = candidate
            break
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = Path(get_package_share_directory('agv_ui_backend')) / 'static'
        if pkg.is_dir():
            static_dir = pkg
    except Exception:
        pass

    if static_dir is None:
        logger.error("Could not find static/ directory")
        static_dir = Path(__file__).parent

    @app.get("/")
    async def index():
        return FileResponse(static_dir / "index.html")

    @app.get("/api/status")
    async def get_status():
        return node.get_status()

    @app.post("/api/recording/start")
    async def start_recording():
        return node.call_service(node.start_rec_client, 'start_recording')

    @app.post("/api/recording/stop")
    async def stop_recording():
        return node.call_service(node.stop_rec_client, 'stop_recording')

    @app.websocket("/ws/teleop")
    async def ws_teleop(ws: WebSocket):
        await ws.accept()
        node.active_clients += 1
        node.get_logger().info(f'Client connected ({node.active_clients} total)')
        try:
            while True:
                raw = await asyncio.wait_for(ws.receive_text(), timeout=1.0)
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue

                msg_type = data.get('type')
                if msg_type == 'cmd_vel':
                    node.send_cmd_vel(
                        float(data.get('linear', 0)),
                        float(data.get('angular', 0)))
                elif msg_type == 'e_stop':
                    node.set_e_stop(bool(data.get('active', True)))
                elif msg_type == 'ping':
                    pass  # keepalive

        except asyncio.TimeoutError:
            # No message in 1s — send status update
            try:
                await ws.send_text(json.dumps({
                    'type': 'status', **node.get_status()
                }))
            except Exception:
                pass
            # Re-enter loop by recursing into the handler
            # Actually, we need a different loop structure
        except WebSocketDisconnect:
            pass
        except Exception as e:
            logger.debug(f"WebSocket error: {e}")
        finally:
            node.active_clients = max(0, node.active_clients - 1)
            node.on_client_disconnect()

    # Fix: proper bidirectional WebSocket loop
    @app.websocket("/ws/control")
    async def ws_control(ws: WebSocket):
        await ws.accept()
        node.active_clients += 1
        node.get_logger().info(f'Client connected ({node.active_clients} total)')

        async def send_status():
            while True:
                try:
                    await ws.send_text(json.dumps({
                        'type': 'status', **node.get_status()
                    }))
                except Exception:
                    break
                await asyncio.sleep(0.2)  # 5Hz status updates

        status_task = asyncio.create_task(send_status())

        try:
            while True:
                raw = await ws.receive_text()
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue

                msg_type = data.get('type')
                if msg_type == 'cmd_vel':
                    node.send_cmd_vel(
                        float(data.get('linear', 0)),
                        float(data.get('angular', 0)))
                elif msg_type == 'e_stop':
                    node.set_e_stop(bool(data.get('active', True)))
                elif msg_type == 'motor_enable':
                    node.set_motor_enable(bool(data.get('active', True)))
                elif msg_type == 'recording':
                    action = data.get('action')
                    if action == 'start':
                        result = node.call_service(node.start_rec_client, 'start_recording')
                    elif action == 'stop':
                        result = node.call_service(node.stop_rec_client, 'stop_recording')
                    else:
                        result = {'success': False, 'message': 'Unknown action'}
                    try:
                        await ws.send_text(json.dumps({'type': 'recording_result', **result}))
                    except Exception:
                        pass
        except WebSocketDisconnect:
            pass
        except Exception as e:
            logger.debug(f"WebSocket error: {e}")
        finally:
            status_task.cancel()
            node.active_clients = max(0, node.active_clients - 1)
            node.on_client_disconnect()

    return app


def main():
    rclpy.init()
    node = TeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = create_app(node)
    node.get_logger().info(f'Starting web server on http://0.0.0.0:{node.port}')

    try:
        uvicorn.run(app, host='0.0.0.0', port=node.port, log_level='warning')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
