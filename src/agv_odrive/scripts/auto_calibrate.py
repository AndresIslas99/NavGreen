#!/usr/bin/env python3
"""
Non-interactive odometry calibration tool.
Drives the robot automatically and prints results.
dev_only: true

Usage:
  auto_calibrate.py radius --distance 2.0 --speed 0.15 --trials 3
  auto_calibrate.py trackwidth --rotations 5 --spin-speed 0.3 --trials 3
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class AutoCalibrator(Node):
    def __init__(self, namespace: str):
        super().__init__('auto_calibrator')
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.theta_accumulated = 0.0
        self.vx = 0.0
        self.vz = 0.0
        self.last_stamp = None
        self.received = False

        odom_topic = f'/{namespace}/wheel_odom'
        cmd_topic = f'/{namespace}/cmd_vel_safe'
        self.sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.get_logger().info(f'Subscribing to {odom_topic}')

    def _odom_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vz = msg.twist.twist.angular.z

        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.vx = msg.twist.twist.linear.x
            self.vz = vz
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
        """Send zero velocity aggressively to ensure robot stops."""
        for _ in range(20):
            self.send_cmd(0.0, 0.0)
            time.sleep(0.02)


def wait_for_odom(node):
    print('Waiting for wheel_odom...')
    t0 = time.time()
    while rclpy.ok() and not node.is_ready():
        time.sleep(0.1)
        if time.time() - t0 > 10:
            print('ERROR: No odometry received in 10s')
            sys.exit(1)
    print('  Odometry active.')


def drive_straight(node, target_distance: float, speed: float):
    """Drive target_distance meters at given speed (negative=backward). Returns odom distance."""
    x0, y0, _ = node.get_pose()
    abs_speed = abs(speed)
    print(f'  Driving {target_distance:.2f}m at {speed:.2f} m/s...')

    t0 = time.time()
    timeout = target_distance / abs_speed * 3 + 5

    while rclpy.ok():
        x, y, _ = node.get_pose()
        traveled = math.hypot(x - x0, y - y0)

        if traveled >= target_distance:
            break
        if time.time() - t0 > timeout:
            print(f'  WARNING: Timeout after {timeout:.0f}s, traveled {traveled:.4f}m')
            break

        node.send_cmd(speed, 0.0)
        time.sleep(0.02)

    node.stop()
    time.sleep(0.5)

    x1, y1, _ = node.get_pose()
    odom_dist = math.hypot(x1 - x0, y1 - y0)
    return odom_dist


def spin_in_place(node, target_rotations: float, spin_speed: float):
    """Spin target_rotations full rotations. Returns odom rotations."""
    node.reset_theta()
    target_rad = target_rotations * 2.0 * math.pi
    print(f'  Spinning {target_rotations:.1f} rotations at {spin_speed:.2f} rad/s...')

    t0 = time.time()
    timeout = target_rad / abs(spin_speed) * 2 + 10  # generous timeout

    while rclpy.ok():
        _, _, theta = node.get_pose()
        if abs(theta) >= target_rad:
            break
        if time.time() - t0 > timeout:
            print(f'  WARNING: Timeout, accumulated {abs(theta)/(2*math.pi):.2f} rotations')
            break

        node.send_cmd(0.0, spin_speed)
        time.sleep(0.02)

    node.stop()
    time.sleep(0.5)

    _, _, theta_final = node.get_pose()
    return abs(theta_final) / (2.0 * math.pi)


def run_radius_auto(node, current_radius: float, target_dist: float,
                     speed: float, num_trials: int):
    print(f'\n=== AUTO WHEEL RADIUS CALIBRATION ===')
    print(f'Current: {current_radius:.4f} m')
    print(f'Target drive: {target_dist:.1f} m at {speed:.2f} m/s')
    print(f'Trials: {num_trials}')
    print(f'\nThe robot will drive forward {target_dist}m {num_trials} times.')
    print(f'Measure the ACTUAL distance with tape for each trial.\n')

    results = []
    for trial in range(1, num_trials + 1):
        # Alternate direction: odd trials backward, even trials forward
        direction = -1.0 if (trial % 2 == 1) else 1.0
        dir_name = "BACKWARD" if direction < 0 else "FORWARD"
        print(f'--- Trial {trial}/{num_trials} ({dir_name}) ---')
        time.sleep(3)  # pause between trials
        odom_dist = drive_straight(node, target_dist, speed * direction)
        print(f'  Odometry says: {odom_dist:.4f} m')
        print(f'  >>> MEASURE the actual distance with tape and note it <<<')
        results.append(odom_dist)
        time.sleep(1)

    print(f'\n=== RADIUS RESULTS ===')
    print(f'Odometry distances: {[f"{d:.4f}" for d in results]}')
    print(f'\nTo compute corrected radius:')
    print(f'  corrected = {current_radius} * (actual_measured / odom_distance)')
    print(f'  For each trial, divide your tape measurement by the odom value above,')
    print(f'  multiply by {current_radius}, and average.')
    print()
    return results


def run_trackwidth_auto(node, current_track: float, target_rotations: float,
                         spin_speed: float, num_trials: int):
    print(f'\n=== AUTO TRACK WIDTH CALIBRATION ===')
    print(f'Current: {current_track:.4f} m')
    print(f'Target: {target_rotations:.0f} rotations at {spin_speed:.2f} rad/s')
    print(f'Trials: {num_trials}')
    print(f'\nThe robot will spin {target_rotations:.0f} rotations {num_trials} times.')
    print(f'Count ACTUAL complete rotations for each trial.\n')

    results = []
    for trial in range(1, num_trials + 1):
        print(f'--- Trial {trial}/{num_trials} ---')
        time.sleep(3)  # pause to set up floor mark
        odom_rots = spin_in_place(node, target_rotations, spin_speed)
        print(f'  Odometry says: {odom_rots:.3f} rotations')
        print(f'  >>> COUNT the actual rotations you observed <<<')
        results.append(odom_rots)
        time.sleep(1)

    print(f'\n=== TRACKWIDTH RESULTS ===')
    print(f'Odometry rotations: {[f"{r:.3f}" for r in results]}')
    print(f'\nTo compute corrected track_width:')
    print(f'  corrected = {current_track} * (odom_rotations / actual_rotations)')
    print(f'  For each trial, divide the odom value by your counted rotations,')
    print(f'  multiply by {current_track}, and average.')
    print()
    return results


def main():
    parser = argparse.ArgumentParser(description='Auto odometry calibration')
    parser.add_argument('--namespace', default='agv')
    sub = parser.add_subparsers(dest='command', required=True)

    p_r = sub.add_parser('radius')
    p_r.add_argument('--current-radius', type=float, default=0.0625)
    p_r.add_argument('--distance', type=float, default=2.0, help='Target drive distance (m)')
    p_r.add_argument('--speed', type=float, default=0.15, help='Drive speed (m/s)')
    p_r.add_argument('--trials', type=int, default=3)

    p_t = sub.add_parser('trackwidth')
    p_t.add_argument('--current-track', type=float, default=0.735)
    p_t.add_argument('--rotations', type=float, default=5, help='Target rotations')
    p_t.add_argument('--spin-speed', type=float, default=0.3, help='Spin speed (rad/s)')
    p_t.add_argument('--trials', type=int, default=3)

    args = parser.parse_args()
    rclpy.init()
    node = AutoCalibrator(args.namespace)
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    wait_for_odom(node)

    try:
        if args.command == 'radius':
            run_radius_auto(node, args.current_radius, args.distance, args.speed, args.trials)
        elif args.command == 'trackwidth':
            run_trackwidth_auto(node, args.current_track, args.rotations, args.spin_speed, args.trials)
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        # Aggressively stop before shutdown to avoid runaway robot
        node.stop()
        time.sleep(0.5)
        node.stop()
        # Skip rclpy.shutdown() to avoid CycloneDDS segfault on Jetson
        # The process will exit and OS will clean up
        sys.exit(0)


if __name__ == '__main__':
    main()
