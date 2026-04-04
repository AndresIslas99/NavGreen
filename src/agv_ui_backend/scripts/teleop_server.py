#!/usr/bin/env python3
"""
AGV Operator Backend — Mission Control Server
==============================================
FastAPI + rclpy server for full operator workflow over LAN.

SIMOVE-inspired improvements (v2):
  1. Persistent event log with backend-generated events
  2. Backend-authoritative state machine with transition enforcement
  3. Subsystem health monitoring with diagnostics
  4. Enhanced mission model (nodes/edges/pause/speed)
  5. Speed-adaptive navigation per mission edge

Usage:
  ros2 run agv_ui_backend teleop_server.py --ros-args -p port:=8090

The React dashboard is served at /dashboard (production build).
The legacy teleop console remains at / (static/index.html).
"""

import asyncio
import base64
import io
import json
import logging
import math
import os
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

# Extracted modules
import sys, importlib
_mod_dir = str(Path(__file__).resolve().parent.parent / 'agv_ui_backend')
if _mod_dir not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from agv_ui_backend.py_state_machine import derive_state, allowed_actions
from agv_ui_backend.py_event_log import EventLog
from agv_ui_backend.py_scan_accumulator import ScanAccumulator
from agv_ui_backend.py_camera_handler import CameraHandler

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, JSONResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
logger = logging.getLogger("teleop_server")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

VALID_STATES = (
    'offline', 'idle', 'ready', 'mapping', 'navigating',
    'executing_mission', 'blocked', 'e_stop', 'fault',
)

MAX_EVENTS = 500
EVENTS_FILE_NAME = 'events.jsonl'

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def occupancy_grid_to_png(grid_msg):
    w, h = grid_msg.info.width, grid_msg.info.height
    data = np.array(grid_msg.data, dtype=np.int8).reshape((h, w))
    img = np.full((h, w), 205, dtype=np.uint8)
    img[data == 0] = 254
    img[data == 100] = 0
    img = np.flipud(img)
    pil_img = PILImage.fromarray(img, mode='L')
    buf = io.BytesIO()
    pil_img.save(buf, format='PNG')
    return buf.getvalue()


def get_maps_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = Path(get_package_share_directory('agv_navigation')) / 'maps'
    except Exception:
        d = Path.home() / 'maps'
    d.mkdir(parents=True, exist_ok=True)
    return d


def get_data_dir():
    """Writable data directory for events, missions, etc."""
    d = get_maps_dir().parent
    d.mkdir(parents=True, exist_ok=True)
    return d


def get_missions_path():
    d = get_data_dir() / 'missions'
    d.mkdir(parents=True, exist_ok=True)
    p = d / 'missions.json'
    if not p.exists():
        p.write_text('[]')
    return p


# ---------------------------------------------------------------------------
# ROS2 Node with State Machine + Event Log + Health
# ---------------------------------------------------------------------------

class OperatorNode(Node):
    def __init__(self):
        super().__init__('teleop_server')

        # Parameters
        self.declare_parameter('port', 8090)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.3)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('deadband', 0.08)
        self.declare_parameter('expo', 0.5)

        self.port = self.get_parameter('port').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.deadband = self.get_parameter('deadband').value
        self.expo = self.get_parameter('expo').value

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.e_stop_pub = self.create_publisher(Bool, 'e_stop', 10)
        self.motor_enable_pub = self.create_publisher(Bool, 'motor_enable', 10)

        # --- QoS profiles ---
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        transient_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # --- Subscribers ---
        self.create_subscription(Odometry, 'wheel_odom', self._odom_cb, best_effort)
        self.create_subscription(String, 'motor_state', self._motor_state_cb, 10)
        self.create_subscription(String, '/slam/quality', self._slam_cb, 10)
        self.create_subscription(String, '/session/info', self._session_cb, 10)
        self.create_subscription(Odometry, 'odometry/global', self._global_odom_cb, best_effort)
        self.create_subscription(NavPath, 'plan', self._plan_cb, 10)
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, transient_local)
        self.create_subscription(OccupancyGrid, 'live_map', self._live_map_cb, transient_local)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, best_effort)

        # Camera/depth image processing REMOVED — moved to C++ agv_image_server
        # (PIL JPEG encoding was consuming 36% CPU, starving EKF and causing drift)

        # --- Action client ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Service clients ---
        self.start_rec_client = self.create_client(Trigger, '/session/start_recording')
        self.stop_rec_client = self.create_client(Trigger, '/session/stop_recording')
        self.load_map_client = self.create_client(LoadMap, 'map_server/load_map')
        self.clear_map_pub = self.create_publisher(Bool, 'clear_map', 10)

        # =====================================================================
        # State: raw sensor data
        # =====================================================================
        self.e_stop_active = False
        self.last_cmd_time = 0.0
        self.active_clients = 0
        self.odom_times = deque(maxlen=50)
        self.imu_times = deque(maxlen=50)
        self.latest_velocity = {'linear': 0.0, 'angular': 0.0}
        self.motor_state = {
            'armed': False, 'left_state': 0, 'right_state': 0,
            'left_errors': 0, 'right_errors': 0,
        }
        self.slam_status = {}
        self.session_status = {}
        self.current_mode = 'teleop'
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.nav_path = []
        self.map_png = None
        self.map_meta = None
        self.map_changed = False
        self.live_map_png = None
        self.live_map_meta = None
        self.live_map_changed = False
        self.scan_points = []

        # =====================================================================
        # Improvement 1: Persistent Event Log
        # =====================================================================
        self.event_log = []
        self._events_path = get_data_dir() / EVENTS_FILE_NAME
        self._load_events()
        self._pending_ws_events = []  # events to push to WS clients

        # =====================================================================
        # Improvement 2: Backend State Machine
        # =====================================================================
        self.robot_state = 'offline'
        self._prev_robot_state = 'offline'

        # =====================================================================
        # Improvement 3: Subsystem Health
        # =====================================================================
        self.health = {
            'drive': {'status': 'unknown', 'detail': 'waiting', 'updated': 0},
            'imu': {'status': 'unknown', 'detail': 'waiting', 'updated': 0},
            'slam': {'status': 'unknown', 'detail': 'waiting', 'updated': 0},
            'nav': {'status': 'unknown', 'detail': 'waiting', 'updated': 0},
            'network': {'status': 'ok', 'detail': '', 'updated': time.time()},
        }

        # =====================================================================
        # Improvement 4: Enhanced Missions
        # =====================================================================
        self.nav_goal_handle = None
        self.nav_state = {'active': False, 'distance_remaining': 0.0, 'status': 'idle'}
        self._mission_cancel = False
        self._mission_pause = False
        self.mission_progress = None  # {mission_id, current_node, total_nodes, status}

        # Scan accumulation removed — C++ scan_grid_mapper handles occupancy grid.
        # Stubs for WS broadcast compat (acc_map messages no longer sent from Python).
        self._acc_changed = False
        self._acc_png = None

        # --- Timers ---
        self.create_timer(0.1, self._watchdog)
        self.create_timer(1.0, self._update_health)

        self.emit_event('info', 'SYSTEM', 'Operator backend started')
        self.get_logger().info(
            f'Operator backend ready on port {self.port}, '
            f'max_linear={self.max_linear}, max_angular={self.max_angular}')

    # =====================================================================
    # Event Log (Improvement 1)
    # =====================================================================

    def _load_events(self):
        """Load last N events from disk on startup."""
        try:
            if self._events_path.exists():
                lines = self._events_path.read_text().strip().splitlines()
                for line in lines[-MAX_EVENTS:]:
                    self.event_log.append(json.loads(line))
        except Exception:
            pass

    def emit_event(self, severity: str, subsystem: str, text: str):
        """Create an event, persist to disk, queue for WS broadcast."""
        entry = {
            'timestamp': time.time(),
            'severity': severity,
            'subsystem': subsystem,
            'text': text,
        }
        self.event_log.append(entry)
        if len(self.event_log) > MAX_EVENTS:
            self.event_log = self.event_log[-MAX_EVENTS:]
        self._pending_ws_events.append(entry)
        # Persist (append)
        try:
            with open(self._events_path, 'a') as f:
                f.write(json.dumps(entry) + '\n')
        except Exception:
            pass

    # =====================================================================
    # State Machine (Improvement 2)
    # =====================================================================

    def _update_state(self):
        """Recompute robot_state from current sensor data. Emit events on transitions."""
        old = self.robot_state
        # Priority-ordered checks
        if self.e_stop_active:
            new = 'e_stop'
        elif (self.motor_state.get('left_errors', 0) != 0
              or self.motor_state.get('right_errors', 0) != 0):
            new = 'fault'
        elif self.odom_hz() < 1.0 and not self.motor_state.get('armed', False):
            new = 'idle'
        elif self.current_mode == 'mapping':
            new = 'mapping'
        elif self.mission_progress and self.mission_progress.get('status') == 'running':
            new = 'executing_mission'
        elif self.nav_state.get('active') and self.nav_state.get('status') == 'active':
            new = 'navigating'
        elif self.motor_state.get('armed', False):
            new = 'ready'
        else:
            new = 'idle'

        self.robot_state = new
        if old != new:
            self.emit_event(
                'crit' if new in ('e_stop', 'fault') else 'info',
                'SYSTEM',
                f'State: {old} → {new}')
            self._prev_robot_state = old

    def allowed_actions(self):
        s = self.robot_state
        return {
            'canTeleop': s in ('ready', 'mapping'),
            'canStartMapping': s in ('ready', 'idle'),
            'canStopMapping': s == 'mapping',
            'canSendGoal': s in ('ready', 'navigating'),
            'canExecuteMission': s == 'ready',
            'canSaveMap': s not in ('offline', 'fault'),
            'canLoadMap': s in ('idle', 'ready'),
            'canMotorEnable': s in ('idle', 'ready', 'e_stop'),
            'canPauseMission': s == 'executing_mission',
            'canCancelNav': s in ('navigating', 'executing_mission'),
        }

    # =====================================================================
    # Subsystem Health (Improvement 3)
    # =====================================================================

    def _update_health(self):
        """Called at 1 Hz to compute subsystem health from sensor data."""
        now = time.time()

        # Drive
        hz = self.odom_hz()
        if hz > 10:
            self.health['drive'] = {'status': 'ok', 'detail': f'{hz:.0f} Hz', 'updated': now}
        elif hz > 1:
            self.health['drive'] = {'status': 'warn', 'detail': f'{hz:.0f} Hz (low)', 'updated': now}
        else:
            self.health['drive'] = {'status': 'error', 'detail': 'No odom', 'updated': now,
                                    'action': 'Check CAN connection and ODrive'}

        # IMU
        imu_hz = self._calc_hz(self.imu_times)
        if imu_hz > 30:
            self.health['imu'] = {'status': 'ok', 'detail': f'{imu_hz:.0f} Hz', 'updated': now}
        elif imu_hz > 5:
            self.health['imu'] = {'status': 'warn', 'detail': f'{imu_hz:.0f} Hz (low)', 'updated': now}
        else:
            self.health['imu'] = {'status': 'error', 'detail': 'No IMU', 'updated': now,
                                  'action': 'Check ZED camera or IMU bridge'}

        # SLAM
        conf = self.slam_status.get('tracking', {}).get('confidence', 'unknown')
        if conf == 'good':
            self.health['slam'] = {'status': 'ok', 'detail': 'Tracking: good', 'updated': now}
        elif conf in ('fair', 'low'):
            self.health['slam'] = {'status': 'warn', 'detail': f'Tracking: {conf}', 'updated': now,
                                   'action': 'Move to area with visual features'}
        else:
            self.health['slam'] = {'status': 'error', 'detail': f'Tracking: {conf}', 'updated': now,
                                   'action': 'Check cuVSLAM or visual_slam topic'}

        # Nav
        nav_ready = self.nav_client.server_is_ready() if self.nav_client else False
        if nav_ready:
            self.health['nav'] = {'status': 'ok', 'detail': 'Nav2 active', 'updated': now}
        else:
            self.health['nav'] = {'status': 'warn', 'detail': 'Nav2 not ready', 'updated': now,
                                  'action': 'Check Nav2 lifecycle nodes'}

        # Network
        self.health['network'] = {
            'status': 'ok', 'detail': f'{self.active_clients} client(s)', 'updated': now}

        # Update state machine
        self._update_state()

    @staticmethod
    def _calc_hz(times_deque):
        if len(times_deque) < 2:
            return 0.0
        dt = times_deque[-1] - times_deque[0]
        return round((len(times_deque) - 1) / dt, 1) if dt > 0 else 0.0

    # _update_acc_png and clear_accumulated_map removed — C++ scan_grid_mapper handles this.

    # =====================================================================
    # Callbacks
    # =====================================================================

    def _odom_cb(self, msg):
        self.odom_times.append(time.time())
        v = msg.twist.twist
        self.latest_velocity = {
            'linear': round(v.linear.x, 3),
            'angular': round(v.angular.z, 3),
        }

    def _motor_state_cb(self, msg):
        try:
            prev_armed = self.motor_state.get('armed', False)
            self.motor_state = json.loads(msg.data)
            new_armed = self.motor_state.get('armed', False)
            if prev_armed != new_armed:
                self.emit_event('info', 'DRIVE',
                                'Motors armed' if new_armed else 'Motors disarmed')
                self._update_state()
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

    def _global_odom_cb(self, msg):
        p = msg.pose.pose
        self.robot_pose = {
            'x': round(p.position.x, 4),
            'y': round(p.position.y, 4),
            'theta': round(yaw_from_quaternion(p.orientation), 4),
        }
        self.imu_times.append(time.time())  # reuse for global odom Hz tracking

    def _plan_cb(self, msg):
        self.nav_path = [
            {'x': round(ps.pose.position.x, 3), 'y': round(ps.pose.position.y, 3)}
            for ps in msg.poses
        ]

    def _map_cb(self, msg):
        try:
            self.map_png = occupancy_grid_to_png(msg)
            info = msg.info
            self.map_meta = {
                'resolution': info.resolution,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'width': info.width,
                'height': info.height,
            }
            self.map_changed = True
            self.get_logger().info(
                f'Map received: {info.width}x{info.height} @ {info.resolution}m/px')
        except Exception as e:
            self.get_logger().error(f'Map conversion failed: {e}')

    def _live_map_cb(self, msg):
        """Receive live occupancy grid from scan_grid_mapper during commissioning."""
        try:
            self.live_map_png = occupancy_grid_to_png(msg)
            info = msg.info
            self.live_map_meta = {
                'resolution': info.resolution,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'width': info.width,
                'height': info.height,
            }
            self.live_map_changed = True
        except Exception:
            pass

    def _scan_cb(self, msg):
        """Extract world-frame scan points for dashboard red dots.
        Grid accumulation removed — handled by C++ scan_grid_mapper_node."""
        try:
            pose = self.robot_pose
            px, py, pt = pose['x'], pose['y'], pose['theta']
            cos_t = math.cos(pt)
            sin_t = math.sin(pt)
            points = []

            angle = msg.angle_min
            ray_idx = 0
            for r in msg.ranges:
                ray_idx += 1
                if msg.range_min < r < msg.range_max and ray_idx % 2 == 0:
                    lx = r * math.cos(angle)
                    ly = r * math.sin(angle)
                    mx = px + cos_t * lx - sin_t * ly
                    my = py + sin_t * lx + cos_t * ly
                    points.append({'x': round(mx, 3), 'y': round(my, 3)})
                angle += msg.angle_increment

            self.scan_points = points
        except Exception:
            pass

    # _raycast_free removed — C++ scan_grid_mapper handles Bresenham raycast.

    # _camera_cb and _depth_cb REMOVED — C++ agv_image_server handles JPEG encoding
    # (was consuming 36% CPU with PIL, starving EKF and causing drift)

    # =====================================================================
    # Watchdog + Teleop
    # =====================================================================

    def _watchdog(self):
        if self.active_clients > 0 and not self.e_stop_active:
            elapsed = time.time() - self.last_cmd_time
            if elapsed > self.cmd_vel_timeout and self.last_cmd_time > 0:
                self._send_zero()

    @staticmethod
    def _apply_expo(val, expo):
        return val * (1.0 - expo) + (val ** 3) * expo

    def send_cmd_vel(self, linear: float, angular: float):
        if self.e_stop_active:
            return
        if self.current_mode not in ('teleop', 'mapping'):
            return
        if (abs(linear) < self.deadband * self.max_linear and
                abs(angular) < self.deadband * self.max_angular):
            linear = 0.0
            angular = 0.0
        else:
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
            self.cancel_nav_goal()
        self.emit_event('crit' if active else 'info', 'SAFETY',
                        'E-STOP ACTIVATED' if active else 'E-stop cleared')
        self._update_state()

    def set_motor_enable(self, active: bool):
        msg = Bool()
        msg.data = active
        self.motor_enable_pub.publish(msg)
        self.get_logger().info(f'Motor enable: {active}')

    def on_client_disconnect(self):
        self._send_zero()

    # =====================================================================
    # Mode
    # =====================================================================

    def set_mode(self, mode: str):
        if mode not in ('teleop', 'mapping', 'nav'):
            return False
        old = self.current_mode
        if mode != 'nav' and self.nav_state['active']:
            self.cancel_nav_goal()
        if mode != 'teleop':
            self._send_zero()
        self.current_mode = mode
        if old != mode:
            self.emit_event('info', 'SYSTEM', f'Mode: {old} → {mode}')
            self._update_state()
        return True

    # =====================================================================
    # Navigation
    # =====================================================================

    def send_nav_goal(self, x: float, y: float, theta: float = 0.0):
        if self.current_mode != 'nav':
            return {'success': False, 'message': 'Not in nav mode'}
        if not self.nav_client.server_is_ready():
            return {'success': False, 'message': 'Nav2 action server not available'}

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb)
        future.add_done_callback(self._nav_goal_response_cb)
        self.nav_state = {'active': True, 'distance_remaining': 0.0, 'status': 'sending'}
        self.emit_event('info', 'NAV', f'Goal sent: ({x:.2f}, {y:.2f})')
        self._update_state()
        return {'success': True, 'message': 'Goal sent'}

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.nav_state = {'active': False, 'distance_remaining': 0.0, 'status': 'rejected'}
            self.emit_event('warn', 'NAV', 'Goal rejected')
            self._update_state()
            return
        self.nav_goal_handle = goal_handle
        self.nav_state['status'] = 'active'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.nav_state['distance_remaining'] = round(fb.distance_remaining, 3)
        self.nav_state['status'] = 'active'

    def _nav_result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.nav_state = {'active': False, 'distance_remaining': 0.0, 'status': 'succeeded'}
            self.emit_event('info', 'NAV', 'Goal reached')
        elif status == 5:
            self.nav_state = {'active': False, 'distance_remaining': 0.0, 'status': 'canceled'}
            self.emit_event('info', 'NAV', 'Goal canceled')
        else:
            self.nav_state = {'active': False, 'distance_remaining': 0.0, 'status': 'aborted'}
            self.emit_event('warn', 'NAV', 'Goal aborted')
        self.nav_goal_handle = None
        self.nav_path = []
        self._update_state()

    def cancel_nav_goal(self):
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_state['status'] = 'canceling'
        self._mission_cancel = True

    # =====================================================================
    # Status (includes state machine + health + allowed actions)
    # =====================================================================

    def odom_hz(self):
        return self._calc_hz(self.odom_times)

    def get_status(self):
        return {
            'robot_state': self.robot_state,
            'allowed_actions': self.allowed_actions(),
            'wheel_odom_hz': self.odom_hz(),
            'velocity': self.latest_velocity,
            'e_stop': self.e_stop_active,
            'motors_armed': bool(self.motor_state.get('armed', False)),
            'left_state': self.motor_state.get('left_state', 0),
            'right_state': self.motor_state.get('right_state', 0),
            'motor_errors': (self.motor_state.get('left_errors', 0) != 0
                             or self.motor_state.get('right_errors', 0) != 0),
            'drive_online': self.odom_hz() > 1.0,
            'slam_tracking': self.slam_status.get('tracking', {}).get('confidence', 'unknown'),
            'recording': bool(self.session_status.get('recording', False)),
            'clients': self.active_clients,
            'mode': self.current_mode,
            'pose': self.robot_pose,
            'nav_state': self.nav_state,
            'health': self.health,
            'mission_progress': self.mission_progress,
        }

    # =====================================================================
    # Service helpers (non-blocking)
    # =====================================================================

    async def call_service_async(self, client, name):
        if not client.service_is_ready():
            return {'success': False, 'message': f'{name} service not available'}
        req = Trigger.Request()
        future = client.call_async(req)
        for _ in range(50):
            if future.done():
                break
            await asyncio.sleep(0.1)
        if future.done() and future.result() is not None:
            r = future.result()
            return {'success': r.success, 'message': r.message}
        return {'success': False, 'message': 'Service call timed out'}

    async def load_map_file_async(self, yaml_path: str):
        if not self.load_map_client.service_is_ready():
            return {'success': False, 'message': 'map_server/load_map not available'}
        req = LoadMap.Request()
        req.map_url = yaml_path
        future = self.load_map_client.call_async(req)
        for _ in range(100):
            if future.done():
                break
            await asyncio.sleep(0.1)
        if future.done() and future.result() is not None:
            r = future.result()
            return {'success': r.result == 0, 'message': f'result={r.result}'}
        return {'success': False, 'message': 'LoadMap timed out'}


# ---------------------------------------------------------------------------
# FastAPI Application
# ---------------------------------------------------------------------------

def create_app(node: OperatorNode) -> FastAPI:
    app = FastAPI(title="AGV Mission Control")

    # --- Locate directories ---
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
        static_dir = Path(__file__).parent

    dashboard_dir = None
    for candidate in [
        Path(__file__).resolve().parent.parent.parent.parent / 'web' / 'agv_dashboard' / 'dist',
        Path(__file__).resolve().parent.parent / 'share' / 'agv_ui_backend' / 'dashboard',
    ]:
        if candidate.is_dir():
            dashboard_dir = candidate
            break
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dash = Path(get_package_share_directory('agv_ui_backend')) / 'dashboard'
        if pkg_dash.is_dir():
            dashboard_dir = pkg_dash
    except Exception:
        pass

    maps_dir = get_maps_dir()
    missions_path = get_missions_path()

    # =======================================================================
    # Legacy
    # =======================================================================

    @app.get("/")
    async def index():
        # Redirect root to React dashboard (production UI)
        if dashboard_dir and dashboard_dir.is_dir():
            from fastapi.responses import RedirectResponse
            return RedirectResponse(url="/dashboard")
        return FileResponse(static_dir / "index.html")

    # =======================================================================
    # Status + Health + Events
    # =======================================================================

    @app.get("/api/status")
    async def api_status():
        return node.get_status()

    @app.get("/api/health")
    async def api_health():
        return node.health

    @app.get("/api/acc_map/image")
    async def get_acc_map_image():
        # Scan accumulator removed — use /agv/live_map from scan_grid_mapper
        return JSONResponse({'error': 'Use /agv/live_map from scan_grid_mapper'}, 404)

    @app.delete("/api/acc_map")
    async def clear_acc_map():
        """Clear the live scan grid map via topic."""
        node.clear_map_pub.publish(Bool(data=True))
        return {'success': True}

    @app.get("/api/events")
    async def api_events(limit: int = 100, offset: int = 0):
        events = list(reversed(node.event_log))  # newest first
        return events[offset:offset + limit]

    @app.delete("/api/events")
    async def clear_events():
        node.event_log.clear()
        try:
            node._events_path.write_text('')
        except Exception:
            pass
        return {'success': True}

    # =======================================================================
    # Recording
    # =======================================================================

    @app.post("/api/recording/start")
    async def start_recording():
        return await node.call_service_async(node.start_rec_client, 'start_recording')

    @app.post("/api/recording/stop")
    async def stop_recording():
        return await node.call_service_async(node.stop_rec_client, 'stop_recording')

    # =======================================================================
    # Mode
    # =======================================================================

    @app.get("/api/mode")
    async def get_mode():
        return {'mode': node.current_mode}

    @app.put("/api/mode")
    async def set_mode(body: dict):
        mode = body.get('mode', '')
        if node.set_mode(mode):
            return {'success': True, 'mode': node.current_mode}
        return JSONResponse({'success': False, 'message': f'Invalid mode: {mode}'}, 400)

    # =======================================================================
    # Maps
    # =======================================================================

    @app.get("/api/maps")
    async def list_maps():
        result = []
        for f in sorted(maps_dir.glob('*.yaml')):
            try:
                text = f.read_text(errors='ignore')
                if 'image:' not in text:
                    continue
            except Exception:
                continue
            result.append({'name': f.stem, 'modified': f.stat().st_mtime})
        return result

    @app.get("/api/maps/{name}/image")
    async def get_map_image(name: str):
        yaml_path = maps_dir / f'{name}.yaml'
        if not yaml_path.exists():
            return JSONResponse({'error': 'Map not found'}, 404)
        pgm_name = None
        for line in yaml_path.read_text().splitlines():
            if line.strip().startswith('image:'):
                pgm_name = line.split(':', 1)[1].strip()
                break
        if not pgm_name:
            return JSONResponse({'error': 'No image in map YAML'}, 400)
        pgm_path = maps_dir / pgm_name
        if not pgm_path.exists():
            return JSONResponse({'error': f'PGM not found: {pgm_name}'}, 404)
        img = PILImage.open(pgm_path)
        buf = io.BytesIO()
        img.save(buf, format='PNG')
        return Response(content=buf.getvalue(), media_type='image/png')

    @app.post("/api/maps/save")
    async def save_map(body: dict):
        name = body.get('name', '').strip()
        if not name or '/' in name or '..' in name:
            return JSONResponse({'error': 'Invalid name'}, 400)
        out_path = str(maps_dir / name)
        proc = await asyncio.create_subprocess_exec(
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', out_path, '-t', '/agv/map',
            '--ros-args', '-p', 'save_map_timeout:=10.0',
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=15.0)
        if proc.returncode == 0:
            node.emit_event('info', 'MAPPING', f'Map "{name}" saved')
            return {'success': True, 'name': name}
        node.emit_event('warn', 'MAPPING', f'Map save failed: {name}')
        return JSONResponse({
            'success': False,
            'message': stderr.decode(errors='ignore').strip() or 'map_saver failed',
        }, 500)

    @app.post("/api/maps/load")
    async def load_map(body: dict):
        name = body.get('name', '').strip()
        yaml_path = maps_dir / f'{name}.yaml'
        if not yaml_path.exists():
            return JSONResponse({'error': 'Map not found'}, 404)
        result = await node.load_map_file_async(str(yaml_path))
        if result.get('success'):
            node.emit_event('info', 'MAPPING', f'Map "{name}" loaded')
        return result

    # =======================================================================
    # Missions (Improvement 4: Enhanced model)
    # =======================================================================

    def _read_missions():
        try:
            return json.loads(missions_path.read_text())
        except Exception:
            return []

    def _write_missions(missions):
        missions_path.write_text(json.dumps(missions, indent=2))

    def _normalize_mission(m):
        """Convert old waypoints[] format to nodes[] if needed."""
        if 'nodes' not in m and 'waypoints' in m:
            m['nodes'] = [
                {'id': f'n{i}', 'type': 'waypoint', 'action': 'none', **wp}
                for i, wp in enumerate(m['waypoints'])
            ]
            m['edges'] = []
        return m

    @app.get("/api/missions")
    async def list_missions():
        return [_normalize_mission(m) for m in _read_missions()]

    @app.post("/api/missions")
    async def create_mission(body: dict):
        missions = _read_missions()
        # Accept both old (waypoints) and new (nodes) format
        nodes = body.get('nodes', [])
        if not nodes and body.get('waypoints'):
            nodes = [
                {'id': f'n{i}', 'type': 'waypoint', 'action': 'none', **wp}
                for i, wp in enumerate(body['waypoints'])
            ]
        mission = {
            'id': body.get('id', str(uuid.uuid4())[:8]),
            'name': body.get('name', 'Untitled'),
            'nodes': nodes,
            'edges': body.get('edges', []),
            'repeat': body.get('repeat', False),
            'created': time.time(),
        }
        missions.append(mission)
        _write_missions(missions)
        node.emit_event('info', 'MISSION', f'Mission "{mission["name"]}" created ({len(nodes)} nodes)')
        return mission

    @app.delete("/api/missions/{mission_id}")
    async def delete_mission(mission_id: str):
        missions = _read_missions()
        name = next((m.get('name', '?') for m in missions if m.get('id') == mission_id), '?')
        missions = [m for m in missions if m.get('id') != mission_id]
        _write_missions(missions)
        node.emit_event('info', 'MISSION', f'Mission "{name}" deleted')
        return {'success': True}

    @app.post("/api/missions/{mission_id}/execute")
    async def execute_mission(mission_id: str):
        missions = _read_missions()
        mission = next((m for m in missions if m.get('id') == mission_id), None)
        if not mission:
            return JSONResponse({'error': 'Mission not found'}, 404)
        mission = _normalize_mission(mission)
        if node.current_mode != 'nav':
            return JSONResponse({'error': 'Not in nav mode'}, 400)
        nodes = mission.get('nodes', [])
        if not nodes:
            return JSONResponse({'error': 'Mission has no nodes'}, 400)

        edges = {(e['from'], e['to']): e for e in mission.get('edges', [])}
        node._mission_cancel = False
        node._mission_pause = False
        node.mission_progress = {
            'mission_id': mission_id,
            'mission_name': mission.get('name', ''),
            'current_node': 0,
            'total_nodes': len(nodes),
            'status': 'running',
        }
        node.emit_event('info', 'MISSION', f'Mission "{mission["name"]}" started ({len(nodes)} nodes)')
        node._update_state()

        async def run_mission():
            for i, nd in enumerate(nodes):
                if node._mission_cancel:
                    node.mission_progress['status'] = 'canceled'
                    node.emit_event('info', 'MISSION', 'Mission canceled')
                    break

                # Pause support
                while node._mission_pause:
                    await asyncio.sleep(0.5)
                    if node._mission_cancel:
                        break

                node.mission_progress['current_node'] = i
                node.mission_progress['status'] = 'running'

                # Get edge speed if available
                if i > 0:
                    prev_id = nodes[i - 1].get('id', '')
                    curr_id = nd.get('id', '')
                    edge = edges.get((prev_id, curr_id), {})
                    # Future: set Nav2 desired_linear_vel from edge.get('max_speed')

                # Send nav goal
                node.send_nav_goal(
                    float(nd.get('x', 0)),
                    float(nd.get('y', 0)),
                    float(nd.get('theta', 0)))

                # Wait for completion
                while node.nav_state.get('active', False):
                    await asyncio.sleep(0.5)
                    if node._mission_cancel:
                        node.cancel_nav_goal()
                        break

                if node.nav_state.get('status') != 'succeeded':
                    node.mission_progress['status'] = 'failed'
                    node.emit_event('warn', 'MISSION',
                                    f'Mission failed at node {i}: {node.nav_state.get("status")}')
                    break

                # Node action
                action = nd.get('action', 'none')
                if action == 'pause':
                    pause_sec = float(nd.get('pause_sec', 3))
                    node.emit_event('info', 'MISSION', f'Pausing {pause_sec}s at node {i}')
                    await asyncio.sleep(pause_sec)
                elif action == 'signal':
                    node.emit_event('info', 'MISSION', f'Signal at node {i}')

            else:
                # All nodes completed
                node.mission_progress['status'] = 'completed'
                node.emit_event('info', 'MISSION',
                                f'Mission "{mission["name"]}" completed')

            node._update_state()

        asyncio.create_task(run_mission())
        return {'success': True, 'nodes': len(nodes)}

    @app.post("/api/missions/pause")
    async def pause_mission():
        node._mission_pause = True
        node.emit_event('info', 'MISSION', 'Mission paused')
        return {'success': True}

    @app.post("/api/missions/resume")
    async def resume_mission():
        node._mission_pause = False
        node.emit_event('info', 'MISSION', 'Mission resumed')
        return {'success': True}

    # =======================================================================
    # Navigation (direct goal)
    # =======================================================================

    @app.post("/api/nav/goal")
    async def send_goal(body: dict):
        return node.send_nav_goal(
            float(body.get('x', 0)),
            float(body.get('y', 0)),
            float(body.get('theta', 0)))

    @app.post("/api/nav/cancel")
    async def cancel_goal():
        node.cancel_nav_goal()
        return {'success': True}

    # =======================================================================
    # Analytics stubs (full implementation in TypeScript backend)
    # =======================================================================

    @app.get("/api/analytics/summary")
    async def analytics_summary(period: str = '24h'):
        """Stub — returns zeros. Use TypeScript backend for real telemetry."""
        return {
            'uptime_pct': 0, 'distance_m': 0, 'mission_success_rate': 0,
            'mission_count': 0, 'avg_mission_duration_s': 0,
            'avg_odom_hz': 0, 'min_odom_hz': 0, 'max_odom_hz': 0,
            'slam_good_pct': 0,
        }

    @app.get("/api/analytics/timeseries")
    async def analytics_timeseries(metric: str = 'odom_hz'):
        return []

    @app.get("/api/analytics/missions")
    async def analytics_missions():
        return []

    @app.get("/api/replay/samples")
    async def replay_samples():
        return []

    @app.get("/api/replay/events")
    async def replay_events():
        return []

    @app.get("/api/auth/status")
    async def auth_status():
        return {'enabled': False}

    @app.post("/api/auth/login")
    async def auth_login(body: dict):
        """Stub login — auth disabled in Python backend. Returns dummy token."""
        return {'token': 'dev-token', 'username': body.get('username', 'dev'), 'role': 'engineer'}

    # =======================================================================
    # Camera stream (MJPEG)
    # =======================================================================

    # Camera/depth endpoints REMOVED — served by C++ agv_image_server on port 8091
    # Dashboard CameraFeed.tsx points directly to http://${host}:8091/camera/stream

    # =======================================================================
    # WebSocket — legacy teleop
    # =======================================================================

    @app.websocket("/ws/teleop")
    async def ws_teleop(ws: WebSocket):
        await ws.accept()
        node.active_clients += 1
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
        except (asyncio.TimeoutError, WebSocketDisconnect):
            pass
        except Exception as e:
            logger.debug(f"WS teleop error: {e}")
        finally:
            node.active_clients = max(0, node.active_clients - 1)
            node.on_client_disconnect()

    # =======================================================================
    # WebSocket — full dashboard control
    # =======================================================================

    @app.websocket("/ws/control")
    async def ws_control(ws: WebSocket):
        await ws.accept()
        node.active_clients += 1
        node.get_logger().info(f'Dashboard client connected ({node.active_clients})')

        last_path = []

        async def send_updates():
            nonlocal last_path
            while True:
                try:
                    # 5Hz status (includes robot_state, allowed_actions, health)
                    await ws.send_text(json.dumps({
                        'type': 'status', **node.get_status()
                    }))

                    # Path updates
                    if node.nav_path != last_path:
                        last_path = list(node.nav_path)
                        await ws.send_text(json.dumps({
                            'type': 'path', 'points': last_path
                        }))

                    # Map update (push once)
                    if node.map_changed and node.map_png:
                        node.map_changed = False
                        await ws.send_text(json.dumps({
                            'type': 'map_update',
                            'png_base64': base64.b64encode(node.map_png).decode(),
                            **node.map_meta,
                        }))

                    # Live map from scan_grid_mapper (commissioning)
                    if node.live_map_changed and node.live_map_png:
                        node.live_map_changed = False
                        await ws.send_text(json.dumps({
                            'type': 'map_update',
                            'png_base64': base64.b64encode(node.live_map_png).decode(),
                            **node.live_map_meta,
                        }))

                    # Scan points
                    if node.scan_points:
                        await ws.send_text(json.dumps({
                            'type': 'scan', 'points': node.scan_points
                        }))

                    # Accumulated map (live mapping visualization)
                    if node._acc_png and node._acc_changed:
                        await ws.send_text(json.dumps({
                            'type': 'acc_map',
                            'png_base64': base64.b64encode(node._acc_png).decode(),
                            **node._acc_meta,
                        }))
                        node._acc_changed = False

                    # Pending events (Improvement 1: real-time event push)
                    while node._pending_ws_events:
                        evt = node._pending_ws_events.pop(0)
                        await ws.send_text(json.dumps({
                            'type': 'event', **evt
                        }))

                except Exception:
                    break
                await asyncio.sleep(0.2)

        update_task = asyncio.create_task(send_updates())

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
                elif msg_type == 'mode':
                    node.set_mode(data.get('mode', 'teleop'))
                elif msg_type == 'nav_goal':
                    node.send_nav_goal(
                        float(data.get('x', 0)),
                        float(data.get('y', 0)),
                        float(data.get('theta', 0)))
                elif msg_type == 'nav_cancel':
                    node.cancel_nav_goal()
                elif msg_type == 'recording':
                    action = data.get('action')
                    if action == 'start':
                        result = await node.call_service_async(
                            node.start_rec_client, 'start_recording')
                    elif action == 'stop':
                        result = await node.call_service_async(
                            node.stop_rec_client, 'stop_recording')
                    else:
                        result = {'success': False, 'message': 'Unknown'}
                    try:
                        await ws.send_text(json.dumps(
                            {'type': 'recording_result', **result}))
                    except Exception:
                        pass

        except WebSocketDisconnect:
            pass
        except Exception as e:
            logger.debug(f"WS control error: {e}")
        finally:
            update_task.cancel()
            node.active_clients = max(0, node.active_clients - 1)
            node.on_client_disconnect()

    # --- Mount dashboard ---
    if dashboard_dir and dashboard_dir.is_dir():
        app.mount("/dashboard", StaticFiles(directory=str(dashboard_dir), html=True))
        logger.info(f"Dashboard mounted at /dashboard from {dashboard_dir}")
    else:
        logger.info("No dashboard build found — /dashboard not available")

    return app


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = OperatorNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = create_app(node)
    node.get_logger().info(f'Starting web server on http://0.0.0.0:{node.port}')

    # Retry port binding (previous instance may still be releasing port)
    import time as _time
    for attempt in range(5):
        try:
            uvicorn.run(app, host='0.0.0.0', port=node.port, log_level='warning')
            break
        except OSError as e:
            if 'Address already in use' in str(e) and attempt < 4:
                node.get_logger().warn(f'Port {node.port} busy, retrying in 3s (attempt {attempt+1}/5)')
                _time.sleep(3)
            else:
                raise
        except KeyboardInterrupt:
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
