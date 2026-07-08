#!/usr/bin/env python3
"""
Wheel odometry calibration tool for differential-drive AGV.
dev_only: true — commissioning/calibration tool, not part of robot runtime.

Two-step calibration procedure:
  1. calibrate_odom.py radius       — calibrate wheel_radius from straight-line drives
  2. calibrate_odom.py trackwidth   — calibrate track_width from spin-in-place tests

Run radius first, then trackwidth with the corrected radius.
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class OdomCalibrator(Node):
    """ROS2 node that tracks wheel odometry for calibration purposes.

    Uses twist integration for heading (avoids quaternion wrapping on multi-rotation spins)
    and pose position for linear distance.
    """

    def __init__(self, namespace: str):
        super().__init__('odom_calibrator')
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.theta_accumulated = 0.0
        self.last_stamp = None
        self.received = False

        odom_topic = f'/{namespace}/wheel_odom'
        cmd_topic = f'/{namespace}/cmd_vel'
        self.sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.get_logger().info(f'Subscribing to {odom_topic}, publishing to {cmd_topic}')

    def _odom_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vz = msg.twist.twist.angular.z

        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            if self.last_stamp is not None:
                dt = stamp - self.last_stamp
                if 0.0 < dt < 1.0:
                    self.theta_accumulated += vz * dt
            self.last_stamp = stamp
            self.received = True

    def get_pose(self):
        with self.lock:
            return self.x, self.y, self.theta_accumulated

    def is_ready(self):
        with self.lock:
            return self.received

    def reset_theta(self):
        with self.lock:
            self.theta_accumulated = 0.0

    def send_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.send_cmd(0.0, 0.0)


def wait_for_odom(node):
    """Block until at least one odometry message is received."""
    print('Waiting for wheel_odom...')
    while rclpy.ok() and not node.is_ready():
        time.sleep(0.1)
    print('  Odometry active.')


def input_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print('  Invalid number, try again.')


def run_radius_calibration(node, current_radius: float, num_trials: int):
    print('\n=== WHEEL RADIUS CALIBRATION ===')
    print(f'Current wheel_radius: {current_radius:.4f} m')
    print()
    print('Instructions:')
    print('  1. Place robot at a starting mark on the floor')
    print('  2. Press Enter to zero the reference')
    print('  3. Drive the robot STRAIGHT (teleop at low speed)')
    print('  4. Press Enter at the end, then type the tape-measured distance')
    print(f'  Repeat {num_trials} trials. Use >= 2 meters for accuracy.\n')

    corrections = []

    for trial in range(1, num_trials + 1):
        input(f'--- Trial {trial}/{num_trials}: Press Enter at START ---')
        x0, y0, _ = node.get_pose()

        input('  Drive straight, then press Enter at END...')
        x1, y1, _ = node.get_pose()

        odom_dist = math.hypot(x1 - x0, y1 - y0)
        print(f'  Odometry distance: {odom_dist:.4f} m')

        if odom_dist < 0.05:
            print('  WARNING: distance too small (<5cm), skipping.')
            continue

        actual_dist = input_float('  Tape-measured distance (m): ')

        ratio = actual_dist / odom_dist
        corrected = current_radius * ratio
        corrections.append(corrected)
        print(f'  Ratio: {ratio:.4f}  ->  wheel_radius: {corrected:.5f} m\n')

    _print_result('wheel_radius', corrections, 'straight-line')


def run_trackwidth_calibration(node, current_track: float, spin_speed: float, num_trials: int):
    print('\n=== TRACK WIDTH CALIBRATION ===')
    print(f'Current track_width: {current_track:.4f} m')
    print(f'Spin speed: {spin_speed:.2f} rad/s')
    print()
    print('Instructions:')
    print('  1. Put a tape mark on the floor aligned with a feature on the robot')
    print('  2. Press Enter to start spinning')
    print('  3. Count COMPLETE rotations (mark-to-mark)')
    print('  4. Press Enter to stop, then type the count')
    print(f'  Aim for 5-10 rotations per trial. Repeat {num_trials} trials.\n')

    corrections = []

    for trial in range(1, num_trials + 1):
        input(f'--- Trial {trial}/{num_trials}: Press Enter to START spinning ---')
        node.reset_theta()

        # Publish cmd_vel at 10 Hz via a timer
        spinning = True

        def spin_cb():
            if spinning:
                node.send_cmd(0.0, spin_speed)

        timer = node.create_timer(0.1, spin_cb)

        input('  Counting rotations... Press Enter to STOP ---')
        spinning = False
        timer.cancel()
        node.destroy_timer(timer)
        node.stop()
        time.sleep(0.3)  # let final odom messages arrive

        _, _, theta_acc = node.get_pose()
        odom_rotations = abs(theta_acc) / (2.0 * math.pi)
        print(f'  Odometry: {math.degrees(abs(theta_acc)):.1f} deg ({odom_rotations:.2f} rotations)')

        actual_rotations = input_float('  Actual complete rotations counted: ')

        if actual_rotations < 0.5:
            print('  WARNING: too few rotations, skipping.')
            continue
        if odom_rotations < 0.1:
            print('  WARNING: odometry near-zero, skipping.')
            continue

        # If odom reports MORE rotations than actual -> track_width is too small
        ratio = odom_rotations / actual_rotations
        corrected = current_track * ratio
        corrections.append(corrected)
        print(f'  Ratio: {ratio:.4f}  ->  track_width: {corrected:.5f} m\n')

    _print_result('track_width', corrections, 'spin')


def _print_result(param_name: str, values: list, test_type: str):
    if not values:
        print('\nNo valid trials. Aborting.')
        return

    mean = sum(values) / len(values)
    if len(values) > 1:
        std = (sum((v - mean) ** 2 for v in values) / (len(values) - 1)) ** 0.5
    else:
        std = 0.0

    print(f'\n=== RESULT ===')
    print(f'  Trials:    {len(values)}')
    print(f'  Mean:      {mean:.5f} m')
    print(f'  Std dev:   {std:.5f} m')
    print(f'\n  YAML snippet:')
    print(f'    {param_name}: {mean:.5f}        # calibrated {len(values)}-trial {test_type} test')
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate differential-drive odometry parameters',
        epilog='Run "radius" first, then "trackwidth" with the corrected value.')
    parser.add_argument('--namespace', default='agv', help='Robot namespace (default: agv)')

    sub = parser.add_subparsers(dest='command', required=True)

    # Seed defaults MUST match the wheel_radius / track_width currently set in
    # config/odrive_params.yaml (the values the running driver integrates with),
    # otherwise the correction ratio is computed against the wrong baseline.
    p_rad = sub.add_parser('radius', help='Calibrate wheel_radius from straight-line drives')
    p_rad.add_argument('--current-radius', type=float, default=0.0781,
                       help='wheel_radius the driver is running with (default: 0.0781, '
                            'the calibrated value in config/odrive_params.yaml)')
    p_rad.add_argument('--trials', type=int, default=3, help='Number of trials (default: 3)')

    p_tw = sub.add_parser('trackwidth', help='Calibrate track_width from spin-in-place tests')
    p_tw.add_argument('--current-track', type=float, default=0.960,
                      help='track_width the driver is running with (default: 0.960, '
                           'the calibrated value in config/odrive_params.yaml)')
    p_tw.add_argument('--spin-speed', type=float, default=0.3,
                      help='Angular velocity in rad/s (default: 0.3)')
    p_tw.add_argument('--trials', type=int, default=3, help='Number of trials (default: 3)')

    args = parser.parse_args()
    rclpy.init()

    if args.command == 'radius':
        node = OdomCalibrator(args.namespace)
    elif args.command == 'trackwidth':
        node = OdomCalibrator(args.namespace)

    # Spin ROS2 in background thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    wait_for_odom(node)

    try:
        if args.command == 'radius':
            run_radius_calibration(node, args.current_radius, args.trials)
        elif args.command == 'trackwidth':
            run_trackwidth_calibration(node, args.current_track, args.spin_speed, args.trials)
    except KeyboardInterrupt:
        print('\nAborted.')
        node.stop()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
