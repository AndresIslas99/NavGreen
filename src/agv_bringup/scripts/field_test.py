#!/usr/bin/env python3
"""
AGV Field Testing Script — Automated baseline & stress test runs.

Automates the Phase 3 field testing protocol:
  - Baseline data collection (3 routes × N reps × 3 times of day)
  - Stress tests (wet floor, condensation, direct sun, shadows, thermal)
  - Endurance testing (continuous loop)

Each run records a rosbag with key topics and logs metrics to CSV.

Usage:
  # Baseline run on route A, 5 repetitions
  ./field_test.py baseline --route A --reps 5

  # Stress test: wet floor
  ./field_test.py stress --condition wet_floor --reps 3

  # Endurance: 8-hour loop
  ./field_test.py endurance --duration-hours 8

  # Analyze collected data
  ./field_test.py analyze --session-dir /mnt/ssd/field_tests/2026-04-09_baseline

Requirements:
  - Robot running with agv_full.launch.py or agv_mapping.launch.py
  - /mnt/ssd/ mounted with sufficient free space (>20GB)
  - session_recorder.py available for bag recording
"""

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped


# Topics to record in each rosbag
BAG_TOPICS = [
    '/visual_slam/tracking/odometry',
    '/agv/odometry/global',
    '/agv/odometry/local',
    '/agv/zed/imu/data',
    '/agv/wheel_odom',
    '/visual_slam/status',
    '/diagnostics',
    '/agv/marker_pose',
    '/agv/drive_debug',
    '/agv/cuvslam_tracking_ok',
    '/agv/pose',
    '/tf',
    '/tf_static',
]

OUTPUT_BASE = '/mnt/ssd/field_tests'


class FieldTestNode(Node):
    """ROS2 node for collecting field test metrics in real-time."""

    def __init__(self):
        super().__init__('field_test_collector')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # Metrics accumulators
        self.tracking_losses = 0
        self.tracking_ok = True
        self.tracking_loss_start = None
        self.tracking_loss_durations = []
        self.ekf_rejections = 0
        self.max_pose_cov = 0.0
        self.cov_samples = []
        self.cuvslam_quality = 'unknown'
        self.poses = []
        self.start_time = time.time()

        # Subscriptions
        self.create_subscription(
            Bool, '/agv/cuvslam_tracking_ok', self._on_tracking_ok, qos)
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self._on_pose, qos)
        self.create_subscription(
            String, '/agv/drive_debug', self._on_drive_debug, qos)

    def _on_tracking_ok(self, msg):
        was_ok = self.tracking_ok
        self.tracking_ok = msg.data
        now = time.time()

        if not msg.data and was_ok:
            self.tracking_losses += 1
            self.tracking_loss_start = now
        elif msg.data and not was_ok and self.tracking_loss_start:
            duration = now - self.tracking_loss_start
            self.tracking_loss_durations.append(duration)
            self.tracking_loss_start = None

    def _on_diagnostics(self, msg):
        for status in msg.status:
            if 'cuVSLAM Tracking' in status.name:
                for kv in status.values:
                    if kv.key == 'quality_level':
                        self.cuvslam_quality = kv.value
                    elif kv.key == 'max_covariance':
                        try:
                            cov = float(kv.value)
                            self.cov_samples.append(cov)
                        except ValueError:
                            pass

    def _on_pose(self, msg):
        cov_diag = [
            abs(msg.pose.covariance[0]),   # x
            abs(msg.pose.covariance[7]),   # y
            abs(msg.pose.covariance[35]),  # yaw
        ]
        max_cov = max(cov_diag)
        self.max_pose_cov = max(self.max_pose_cov, max_cov)
        self.poses.append({
            'time': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'cov_max': max_cov,
        })

    def _on_drive_debug(self, msg):
        try:
            data = json.loads(msg.data)
            # Track caster disturbance events
            if data.get('caster_disturbance', 0) > 0.1:
                pass  # Could log caster events here
        except json.JSONDecodeError:
            pass

    def get_metrics(self):
        """Return current metrics as a dict."""
        elapsed = time.time() - self.start_time
        cov_mean = sum(self.cov_samples) / len(self.cov_samples) if self.cov_samples else 0
        cov_std = 0
        if len(self.cov_samples) > 1:
            cov_std = (sum((c - cov_mean)**2 for c in self.cov_samples)
                       / (len(self.cov_samples) - 1)) ** 0.5

        total_loss_time = sum(self.tracking_loss_durations)
        if self.tracking_loss_start:
            total_loss_time += time.time() - self.tracking_loss_start

        return {
            'elapsed_s': round(elapsed, 1),
            'tracking_losses': self.tracking_losses,
            'tracking_loss_total_s': round(total_loss_time, 1),
            'tracking_loss_max_s': round(max(self.tracking_loss_durations) if self.tracking_loss_durations else 0, 1),
            'max_pose_cov': round(self.max_pose_cov, 4),
            'cov_mean': round(cov_mean, 4),
            'cov_std': round(cov_std, 4),
            'cov_samples': len(self.cov_samples),
            'quality_level': self.cuvslam_quality,
            'pose_count': len(self.poses),
            'distance_m': self._total_distance(),
        }

    def _total_distance(self):
        d = 0.0
        for i in range(1, len(self.poses)):
            dx = self.poses[i]['x'] - self.poses[i-1]['x']
            dy = self.poses[i]['y'] - self.poses[i-1]['y']
            d += (dx*dx + dy*dy) ** 0.5
        return round(d, 2)

    def reset(self):
        self.tracking_losses = 0
        self.tracking_ok = True
        self.tracking_loss_start = None
        self.tracking_loss_durations = []
        self.ekf_rejections = 0
        self.max_pose_cov = 0.0
        self.cov_samples = []
        self.cuvslam_quality = 'unknown'
        self.poses = []
        self.start_time = time.time()


def start_bag_recording(output_dir, run_name):
    """Start ros2 bag recording in background."""
    bag_dir = os.path.join(output_dir, f'bag_{run_name}')
    cmd = ['ros2', 'bag', 'record', '-o', bag_dir] + BAG_TOPICS
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return proc


def stop_bag_recording(proc):
    """Stop ros2 bag recording gracefully."""
    if proc and proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=10)


def check_disk_space(path, min_gb=5.0):
    """Check if sufficient disk space is available."""
    st = os.statvfs(path)
    free_gb = (st.f_bavail * st.f_frsize) / (1024**3)
    if free_gb < min_gb:
        print(f'WARNING: Only {free_gb:.1f}GB free on {path} (need {min_gb}GB)')
        return False
    return True


def check_robot_ready():
    """Verify key nodes are running."""
    result = subprocess.run(
        ['ros2', 'node', 'list'],
        capture_output=True, text=True, timeout=5)
    nodes = result.stdout.strip().split('\n')
    required = ['/agv/odrive_can_node', '/agv/fusion_monitor']
    missing = [n for n in required if n not in nodes]
    if missing:
        print(f'ERROR: Missing nodes: {missing}')
        return False
    return True


def run_baseline(args):
    """Execute baseline field test runs."""
    session_name = f'{datetime.now():%Y-%m-%d_%H%M}_baseline_{args.route}'
    output_dir = os.path.join(OUTPUT_BASE, session_name)
    os.makedirs(output_dir, exist_ok=True)

    csv_path = os.path.join(output_dir, 'metrics.csv')
    fieldnames = [
        'run', 'route', 'time_of_day', 'elapsed_s', 'distance_m',
        'tracking_losses', 'tracking_loss_total_s', 'tracking_loss_max_s',
        'max_pose_cov', 'cov_mean', 'cov_std', 'cov_samples',
        'quality_level', 'pose_count',
    ]

    rclpy.init()
    node = FieldTestNode()

    print(f'=== Baseline Test: Route {args.route}, {args.reps} reps ===')
    print(f'Output: {output_dir}')

    if not check_disk_space(OUTPUT_BASE):
        rclpy.shutdown()
        return

    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for rep in range(1, args.reps + 1):
            run_name = f'route{args.route}_rep{rep:02d}'
            tod = datetime.now().strftime('%H:%M')
            print(f'\n--- Run {rep}/{args.reps} ({run_name}) @ {tod} ---')
            print('  Press ENTER to start run (position robot at start)...')
            input()

            node.reset()
            bag_proc = start_bag_recording(output_dir, run_name)
            print('  Recording... Press ENTER when run is complete.')

            # Spin in background while waiting for user input
            import threading
            spin_event = threading.Event()

            def spin_loop():
                while not spin_event.is_set():
                    rclpy.spin_once(node, timeout_sec=0.1)

            spinner = threading.Thread(target=spin_loop, daemon=True)
            spinner.start()
            input()
            spin_event.set()
            spinner.join(timeout=2)

            stop_bag_recording(bag_proc)
            metrics = node.get_metrics()
            metrics['run'] = run_name
            metrics['route'] = args.route
            metrics['time_of_day'] = tod
            writer.writerow(metrics)
            csvfile.flush()

            print(f'  Results: dist={metrics["distance_m"]}m, '
                  f'losses={metrics["tracking_losses"]}, '
                  f'max_cov={metrics["max_pose_cov"]:.4f}, '
                  f'quality={metrics["quality_level"]}')

    rclpy.shutdown()
    print(f'\n=== Baseline complete. Metrics: {csv_path} ===')


def run_stress(args):
    """Execute stress test runs."""
    session_name = f'{datetime.now():%Y-%m-%d_%H%M}_stress_{args.condition}'
    output_dir = os.path.join(OUTPUT_BASE, session_name)
    os.makedirs(output_dir, exist_ok=True)

    csv_path = os.path.join(output_dir, 'metrics.csv')
    fieldnames = [
        'run', 'condition', 'elapsed_s', 'distance_m',
        'tracking_losses', 'tracking_loss_total_s', 'tracking_loss_max_s',
        'max_pose_cov', 'cov_mean', 'cov_std', 'cov_samples',
        'quality_level', 'pose_count',
    ]

    rclpy.init()
    node = FieldTestNode()

    conditions = {
        'wet_floor': 'Run during active irrigation',
        'condensation': 'Run within 30min of greenhouse opening (morning)',
        'direct_sun': 'Run through sun-facing corridor at noon',
        'shadow_transition': 'Run during partly cloudy conditions',
        'thermal_soak': 'Run after 2+ hours continuous operation',
        'cold_boot': 'Run immediately after cold boot',
    }

    desc = conditions.get(args.condition, args.condition)
    print(f'=== Stress Test: {args.condition} ({desc}) ===')
    print(f'Output: {output_dir}')

    if not check_disk_space(OUTPUT_BASE):
        rclpy.shutdown()
        return

    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for rep in range(1, args.reps + 1):
            run_name = f'{args.condition}_rep{rep:02d}'
            print(f'\n--- Run {rep}/{args.reps} ({run_name}) ---')
            print(f'  Condition: {desc}')
            print('  Press ENTER to start run...')
            input()

            node.reset()
            bag_proc = start_bag_recording(output_dir, run_name)
            print('  Recording... Press ENTER when run is complete.')

            import threading
            spin_event = threading.Event()

            def spin_loop():
                while not spin_event.is_set():
                    rclpy.spin_once(node, timeout_sec=0.1)

            spinner = threading.Thread(target=spin_loop, daemon=True)
            spinner.start()
            input()
            spin_event.set()
            spinner.join(timeout=2)

            stop_bag_recording(bag_proc)
            metrics = node.get_metrics()
            metrics['run'] = run_name
            metrics['condition'] = args.condition
            writer.writerow(metrics)
            csvfile.flush()

            print(f'  Results: dist={metrics["distance_m"]}m, '
                  f'losses={metrics["tracking_losses"]}, '
                  f'max_cov={metrics["max_pose_cov"]:.4f}')

    rclpy.shutdown()
    print(f'\n=== Stress test complete. Metrics: {csv_path} ===')


def run_endurance(args):
    """Execute endurance test (continuous loop)."""
    session_name = f'{datetime.now():%Y-%m-%d_%H%M}_endurance_{args.duration_hours}h'
    output_dir = os.path.join(OUTPUT_BASE, session_name)
    os.makedirs(output_dir, exist_ok=True)

    duration_s = args.duration_hours * 3600
    csv_path = os.path.join(output_dir, 'metrics_periodic.csv')
    fieldnames = [
        'timestamp', 'elapsed_min', 'tracking_losses', 'tracking_loss_total_s',
        'max_pose_cov', 'cov_mean', 'quality_level', 'distance_m',
    ]

    rclpy.init()
    node = FieldTestNode()

    print(f'=== Endurance Test: {args.duration_hours}h ===')
    print(f'Output: {output_dir}')
    print(f'  Logging metrics every 5 minutes')
    print(f'  Pass criteria: <1 loss/hour, max loss <10s, Tj <85°C')

    if not check_disk_space(OUTPUT_BASE, min_gb=20.0):
        rclpy.shutdown()
        return

    bag_proc = start_bag_recording(output_dir, 'endurance')
    start = time.time()
    last_report = start

    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        try:
            while time.time() - start < duration_s:
                rclpy.spin_once(node, timeout_sec=0.5)

                if time.time() - last_report >= 300:  # every 5 min
                    metrics = node.get_metrics()
                    elapsed_min = round((time.time() - start) / 60, 1)
                    row = {
                        'timestamp': datetime.now().isoformat(),
                        'elapsed_min': elapsed_min,
                        'tracking_losses': metrics['tracking_losses'],
                        'tracking_loss_total_s': metrics['tracking_loss_total_s'],
                        'max_pose_cov': metrics['max_pose_cov'],
                        'cov_mean': metrics['cov_mean'],
                        'quality_level': metrics['quality_level'],
                        'distance_m': metrics['distance_m'],
                    }
                    writer.writerow(row)
                    csvfile.flush()

                    losses_per_hour = metrics['tracking_losses'] / max(elapsed_min / 60, 0.01)
                    print(f'  [{elapsed_min:.0f}min] dist={metrics["distance_m"]}m '
                          f'losses={metrics["tracking_losses"]} ({losses_per_hour:.1f}/h) '
                          f'max_cov={metrics["max_pose_cov"]:.4f} '
                          f'quality={metrics["quality_level"]}')
                    last_report = time.time()

        except KeyboardInterrupt:
            print('\n  Endurance test interrupted by user')

    stop_bag_recording(bag_proc)
    final = node.get_metrics()
    rclpy.shutdown()

    elapsed_h = final['elapsed_s'] / 3600
    losses_per_hour = final['tracking_losses'] / max(elapsed_h, 0.01)
    print(f'\n=== Endurance Summary ===')
    print(f'  Duration: {elapsed_h:.1f}h')
    print(f'  Distance: {final["distance_m"]}m')
    print(f'  Tracking losses: {final["tracking_losses"]} ({losses_per_hour:.1f}/h)')
    print(f'  Max loss duration: {final["tracking_loss_max_s"]}s')
    print(f'  Max pose covariance: {final["max_pose_cov"]:.4f}')
    pass_criteria = losses_per_hour < 1.0 and final['tracking_loss_max_s'] < 10.0
    print(f'  PASS: {"YES" if pass_criteria else "NO"}')
    print(f'  Metrics: {csv_path}')


def run_analyze(args):
    """Analyze collected field test data."""
    session_dir = Path(args.session_dir)
    csv_files = list(session_dir.glob('metrics*.csv'))

    if not csv_files:
        print(f'No metrics CSV found in {session_dir}')
        return

    for csv_path in csv_files:
        print(f'\n=== {csv_path.name} ===')
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        if not rows:
            print('  (empty)')
            continue

        # Aggregate metrics
        total_runs = len(rows)
        total_losses = sum(int(r.get('tracking_losses', 0)) for r in rows)
        total_dist = sum(float(r.get('distance_m', 0)) for r in rows)
        max_covs = [float(r.get('max_pose_cov', 0)) for r in rows if r.get('max_pose_cov')]
        max_loss_durs = [float(r.get('tracking_loss_max_s', 0)) for r in rows if r.get('tracking_loss_max_s')]

        print(f'  Runs: {total_runs}')
        print(f'  Total distance: {total_dist:.1f}m')
        print(f'  Total tracking losses: {total_losses} ({total_losses/max(total_runs,1):.1f}/run)')
        if max_covs:
            print(f'  Max pose covariance: {max(max_covs):.4f} (mean {sum(max_covs)/len(max_covs):.4f})')
        if max_loss_durs:
            print(f'  Max loss duration: {max(max_loss_durs):.1f}s')

        # Per-run table
        print(f'\n  {"Run":<30} {"Dist":>6} {"Losses":>7} {"MaxCov":>8} {"Quality":<10}')
        print(f'  {"-"*30} {"-"*6} {"-"*7} {"-"*8} {"-"*10}')
        for r in rows:
            name = r.get('run', r.get('timestamp', '?'))
            dist = r.get('distance_m', '?')
            losses = r.get('tracking_losses', '?')
            cov = r.get('max_pose_cov', '?')
            quality = r.get('quality_level', '?')
            print(f'  {name:<30} {dist:>6} {losses:>7} {cov:>8} {quality:<10}')


def main():
    parser = argparse.ArgumentParser(
        description='AGV Field Testing — automated baseline, stress, and endurance tests')
    subparsers = parser.add_subparsers(dest='command', required=True)

    # Baseline
    p_base = subparsers.add_parser('baseline', help='Run baseline test suite')
    p_base.add_argument('--route', required=True, choices=['A', 'B', 'C'],
                        help='Route: A=straight corridor, B=turn-heavy, C=full operational')
    p_base.add_argument('--reps', type=int, default=5, help='Repetitions per route')

    # Stress
    p_stress = subparsers.add_parser('stress', help='Run stress test')
    p_stress.add_argument('--condition', required=True,
                          choices=['wet_floor', 'condensation', 'direct_sun',
                                   'shadow_transition', 'thermal_soak', 'cold_boot'],
                          help='Environmental condition to test')
    p_stress.add_argument('--reps', type=int, default=3, help='Repetitions')

    # Endurance
    p_endur = subparsers.add_parser('endurance', help='Run endurance test')
    p_endur.add_argument('--duration-hours', type=float, default=8.0,
                         help='Test duration in hours')

    # Analyze
    p_anal = subparsers.add_parser('analyze', help='Analyze test results')
    p_anal.add_argument('--session-dir', required=True,
                        help='Path to session directory with metrics CSV')

    args = parser.parse_args()

    if args.command == 'baseline':
        run_baseline(args)
    elif args.command == 'stress':
        run_stress(args)
    elif args.command == 'endurance':
        run_endurance(args)
    elif args.command == 'analyze':
        run_analyze(args)


if __name__ == '__main__':
    main()
