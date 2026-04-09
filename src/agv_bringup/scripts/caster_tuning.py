#!/usr/bin/env python3
"""
Caster Compensation Tuning Monitor

Real-time visualization and tuning assistant for caster wheel compensation
parameters. Monitors the drive_debug topic and wheel odometry covariance
to help calibrate: tau, multiplier, and angular_accel_threshold.

Usage:
  ./caster_tuning.py                     # Monitor mode (live display)
  ./caster_tuning.py --log output.csv    # Monitor + log to CSV
  ./caster_tuning.py --set tau=0.8       # Change parameter live
  ./caster_tuning.py --set multiplier=15 # Change parameter live
  ./caster_tuning.py --set threshold=0.5 # Change parameter live

Interpreting the output:
  disturbance: 0.0 = no caster effect, 1.0 = max caster disturbance
  cov_angular: Current angular covariance from wheel_odom (inflated during disturbance)

Tuning guide:
  - disturbance triggers too late → reduce threshold (1.0 → 0.5)
  - odometry still bad during settling → increase multiplier (10 → 15) or tau (0.5 → 0.8)
  - disturbance triggers on normal turns → increase threshold (1.0 → 1.5)
  - wet floor → consider multiplier 15+ and reducing local EKF x,y process noise
"""

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters


class CasterTuningNode(Node):
    """Real-time caster compensation monitor and tuner."""

    def __init__(self, log_path=None):
        super().__init__('caster_tuning_monitor')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # State
        self.disturbance = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.left_meas = 0.0
        self.right_meas = 0.0
        self.cov_angular = 0.0
        self.cov_linear = 0.0
        self.armed = False
        self.last_update = 0.0
        self.event_count = 0  # disturbance > 0.1 events
        self.peak_disturbance = 0.0
        self.log_writer = None
        self.log_file = None

        if log_path:
            self.log_file = open(log_path, 'w', newline='')
            fieldnames = [
                'timestamp', 'disturbance', 'cmd_linear', 'cmd_angular',
                'left_meas', 'right_meas', 'cov_linear', 'cov_angular',
                'armed', 'event_count',
            ]
            self.log_writer = csv.DictWriter(self.log_file, fieldnames=fieldnames)
            self.log_writer.writeheader()

        # Subscribe to drive_debug (JSON in String)
        self.create_subscription(
            String, '/agv/drive_debug', self._on_drive_debug, qos)

        # Subscribe to wheel_odom for covariance
        self.create_subscription(
            Odometry, '/agv/wheel_odom', self._on_wheel_odom, qos)

        # Set parameter service client
        self.set_param_client = self.create_client(
            SetParameters, '/agv/odrive_can_node/set_parameters')

        # Display timer (4Hz)
        self.create_timer(0.25, self._display)

    def _on_drive_debug(self, msg):
        try:
            data = json.loads(msg.data)
            self.disturbance = data.get('caster_disturbance', 0.0)
            self.cmd_linear = data.get('cmd_linear', 0.0)
            self.cmd_angular = data.get('cmd_angular', 0.0)
            self.left_meas = data.get('left_meas', 0.0)
            self.right_meas = data.get('right_meas', 0.0)
            self.armed = data.get('armed', False)
            self.last_update = time.time()

            if self.disturbance > 0.1:
                self.event_count += 1
            self.peak_disturbance = max(self.peak_disturbance, self.disturbance)

        except json.JSONDecodeError:
            pass

    def _on_wheel_odom(self, msg):
        # Extract diagonal covariance: linear (x) and angular (yaw)
        self.cov_linear = msg.pose.covariance[0]     # x variance
        self.cov_angular = msg.pose.covariance[35]    # yaw variance

    def _display(self):
        if time.time() - self.last_update > 2.0:
            sys.stdout.write('\r  [NO DATA] Waiting for /agv/drive_debug...       ')
            sys.stdout.flush()
            return

        # Build bar for disturbance level
        bar_len = 30
        filled = int(self.disturbance * bar_len)
        bar = '█' * filled + '░' * (bar_len - filled)

        # Color: green=0, yellow=0.3, red=0.7+
        if self.disturbance < 0.3:
            level = 'OK '
        elif self.disturbance < 0.7:
            level = 'MED'
        else:
            level = 'HI '

        line = (
            f'\r  [{level}] |{bar}| {self.disturbance:.2f}  '
            f'cmd=({self.cmd_linear:+.2f}, {self.cmd_angular:+.2f})  '
            f'wheels=({self.left_meas:+.2f}, {self.right_meas:+.2f})  '
            f'cov=(L:{self.cov_linear:.4f} A:{self.cov_angular:.4f})  '
            f'events={self.event_count} peak={self.peak_disturbance:.2f}  '
        )
        sys.stdout.write(line)
        sys.stdout.flush()

        if self.log_writer:
            self.log_writer.writerow({
                'timestamp': datetime.now().isoformat(),
                'disturbance': round(self.disturbance, 4),
                'cmd_linear': round(self.cmd_linear, 3),
                'cmd_angular': round(self.cmd_angular, 3),
                'left_meas': round(self.left_meas, 3),
                'right_meas': round(self.right_meas, 3),
                'cov_linear': round(self.cov_linear, 6),
                'cov_angular': round(self.cov_angular, 6),
                'armed': self.armed,
                'event_count': self.event_count,
            })
            self.log_file.flush()

    def set_parameter(self, name, value):
        """Set a caster compensation parameter on the odrive node."""
        param_map = {
            'tau': 'caster_settling_tau',
            'multiplier': 'caster_covariance_multiplier',
            'threshold': 'caster_angular_accel_threshold',
        }

        ros_name = param_map.get(name, name)

        if not self.set_param_client.wait_for_service(timeout_sec=2.0):
            print(f'\n  ERROR: odrive_can_node parameter service not available')
            return False

        param = Parameter()
        param.name = ros_name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(value)

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.set_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result():
            result = future.result().results[0]
            if result.successful:
                print(f'\n  SET {ros_name} = {value} (OK)')
                return True
            else:
                print(f'\n  SET {ros_name} = {value} FAILED: {result.reason}')
                return False
        else:
            print(f'\n  SET {ros_name} = {value} TIMEOUT')
            return False

    def destroy_node(self):
        if self.log_file:
            self.log_file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description='Caster compensation tuning monitor')
    parser.add_argument('--log', type=str, default=None,
                        help='Log to CSV file')
    parser.add_argument('--set', type=str, default=None,
                        help='Set parameter: tau=0.8, multiplier=15, threshold=0.5')
    args = parser.parse_args()

    rclpy.init()
    node = CasterTuningNode(log_path=args.log)

    # If --set is provided, set the parameter and exit
    if args.set:
        name, value = args.set.split('=', 1)
        node.set_parameter(name.strip(), float(value.strip()))
        node.destroy_node()
        rclpy.shutdown()
        return

    print('=== Caster Compensation Tuning Monitor ===')
    print('  Current params: check /agv/odrive_can_node params')
    print('  Ctrl+C to stop. Use --set to change params live.')
    print()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n\n  Session: {node.event_count} disturbance events, '
              f'peak={node.peak_disturbance:.2f}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
