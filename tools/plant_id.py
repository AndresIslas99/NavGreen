#!/usr/bin/env python3
"""Plant identification for HIL drive: step response at multiple cmd levels.

Publishes cmd_vel.linear.x = V to /agv/cmd_vel_armed (sim_drive_shaping
input, bypassing arbiter + smoother + safety gate). Records GT pose to
estimate actual robot velocity. Repeats at:
  V = 0.005, 0.010, 0.015, 0.020, 0.025, 0.030, 0.040, 0.050, 0.060, 0.080, 0.100, 0.150, 0.200, 0.300

For each step:
  - Apply cmd for 5 s
  - Sample GT pose at start and end
  - Estimate steady-state vel = (x_end - x_start) / dt (after 1 s startup)
  - Then apply 0 cmd for 2 s to stop

Outputs:
  - V_stiction_break: smallest V producing measurable motion (>1 mm/s)
  - Plant gain map (cmd → real_vel)
  - Time-constant estimate from rise time

NOTE: Robot must already be teleported to a clear path (e.g. (5.5, 0, π)
before running). Use sim_api /reset if needed.
"""
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped


class PlantId(Node):
    def __init__(self):
        super().__init__('plant_id')
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        # Publish through the FULL chain: cmd_vel → motor_gate →
        # drive_shaping → physics. This is what rail_approach sees as
        # the plant. Compare to direct shaped_cmd_vel test to isolate
        # drive_shaping's effects.
        self.pub = self.create_publisher(Twist, '/agv/cmd_vel', rel)
        self.gt = None
        self.gt_t = None
        self.create_subscription(
            PoseStamped, '/agv/sim/ground_truth/pose',
            self._on_gt, rel)

    def _on_gt(self, msg):
        self.gt = (msg.pose.position.x, msg.pose.position.y)
        self.gt_t = time.monotonic()

    def cmd(self, vx):
        m = Twist()
        m.linear.x = vx
        self.pub.publish(m)


def main():
    rclpy.init()
    node = PlantId()

    # Wait for first GT
    deadline = time.monotonic() + 10
    while node.gt is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.gt is None:
        print('ERROR: no GT pose received', file=sys.stderr)
        return 1

    print('# plant identification — step response')
    print('# %-7s %-10s %-12s %-9s %-9s' % ('cmd_vx', 'duration', 'displacement', 'avg_vel', 'efficiency'))

    voltages = [0.005, 0.010, 0.015, 0.020, 0.025, 0.030, 0.035, 0.040,
                0.050, 0.060, 0.080, 0.100, 0.150, 0.200, 0.300]
    settle_pause = 2.0
    step_dur = 4.0

    rate_hz = 100  # dominate over 20 Hz arbiter republishes
    period = 1.0 / rate_hz

    for V in voltages:
        # Settle 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < settle_pause:
            node.cmd(0.0)
            rclpy.spin_once(node, timeout_sec=period)
        # Snapshot start (after 0.5s of motion to skip stiction breakthrough)
        t_step_start = time.monotonic()
        # Skip first 0.8 s for transient
        while time.monotonic() - t_step_start < 0.8:
            node.cmd(V)
            rclpy.spin_once(node, timeout_sec=period)
        x_start, y_start = node.gt
        t_meas_start = time.monotonic()
        # Active step
        while time.monotonic() - t_step_start < step_dur:
            node.cmd(V)
            rclpy.spin_once(node, timeout_sec=period)
        x_end, y_end = node.gt
        t_meas_end = time.monotonic()
        dt = t_meas_end - t_meas_start
        disp = math.hypot(x_end - x_start, y_end - y_start)
        vel = disp / dt if dt > 1e-3 else 0.0
        eff = vel / V if V > 1e-6 else 0.0
        print('%-7.4f %-10.2f %-12.5f %-9.5f %-9.4f' % (V, dt, disp, vel, eff))
        # Stop briefly
        for _ in range(5):
            node.cmd(0.0)
            rclpy.spin_once(node, timeout_sec=period)

    # Final stop
    for _ in range(20):
        node.cmd(0.0)
        rclpy.spin_once(node, timeout_sec=period)

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
