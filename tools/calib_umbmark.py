#!/usr/bin/env python3
"""UMBmark — Borenstein & Feng (1996) bidirectional square calibration.

Reference:
  J. Borenstein & L. Feng, "Measurement and Correction of Systematic
  Odometry Errors in Mobile Robots," IEEE Trans. Robotics & Automation,
  Dec 1996. https://johnloomis.org/ece445/topics/odometry/borenstein/paper58.pdf

The protocol runs the robot through a square trajectory of side L,
once clockwise (CW) and once counter-clockwise (CCW), repeated N
times each direction. The CW and CCW closure errors carry independent
information about two systematic biases:

  Ed = (Dr_actual / Dl_actual)        # right/left wheel diameter ratio
  Eb = b_actual / b_nominal           # effective wheelbase scale

After Borenstein, with the centre of gravity of the CW endpoints at
(x_cg_cw, y_cg_cw) and CCW at (x_cg_ccw, y_cg_ccw):

  α  = ((x_cg_cw + x_cg_ccw) / (-4 L))          [rad, type-A error]
  β  = ((x_cg_cw - x_cg_ccw) / (-4 L))          [rad, type-B error]
  Ed = (90° + α) / (90° - α)
  Eb = (90°) / (90° - β)

These corrections are then applied to wheel_radius_left/right and
track_width, and a re-run of UMBmark validates the residual.

Two execution modes:

  --mode auto      — Script publishes cmd_vel via WebSocket to follow
                     the square. Uses /agv/odometry/global as feedback
                     for "did I close the side?" so operator can leave
                     the keyboard. Works only with motors_armed.

  --mode operator  — Operator drives each side with the joystick and
                     hits Enter at each corner. Slower but robust on
                     systems with the watchdog interfering.

For both modes, the AGV must start at a marked position with a known
heading (the script records the start pose; the operator measures the
end pose either via AprilTag or by tape-measure to floor marks).
This script will print the closure error for each of the N×2 runs and
then report the final Ed, Eb, and suggested wheel_radius / track_width
parameters.

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///tmp/agv_cyclonedds_runtime.xml
  python3 tools/calib_umbmark.py --side 4.0 --runs 5 --mode operator
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def angle_wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


@dataclass
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass
class RunResult:
    direction: str   # "CW" or "CCW"
    run_idx: int
    start: Pose2
    end: Pose2
    closure_x: float
    closure_y: float
    closure_yaw: float


class UmbmarkNode(Node):
    def __init__(self):
        super().__init__("calib_umbmark")
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.latest_pose: Pose2 | None = None
        # We use /agv/odometry/global because it fuses cuVSLAM + AprilTag
        # and is the closest thing the AGV has to "ground truth" without
        # an external Vicon/total-station setup. The bias UMBmark detects
        # therefore reflects the *residual* error after EKF correction —
        # which is what calibrating wheel_radius/track_width actually
        # affects (the EKF-fed wheel_odom).
        self.create_subscription(Odometry, "/agv/odometry/global",
                                 self.on_odom, be_qos)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_pose = Pose2(
            x=float(p.x), y=float(p.y),
            yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def wait_for_pose(self, timeout_s: float = 5.0) -> Pose2 | None:
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose is not None:
                return self.latest_pose
        return self.latest_pose


def prompt(msg: str) -> str:
    sys.stdout.write(msg)
    sys.stdout.flush()
    try:
        return input().strip().lower()
    except EOFError:
        return "q"


def run_operator_mode(node: UmbmarkNode, side_m: float,
                       direction: str, run_idx: int) -> RunResult | None:
    sign = +1 if direction == "CCW" else -1  # CCW = left turns
    print(f"\n══ Run {run_idx} — {direction} ══")
    print(f"  Posiciona el robot en el origen, mirando +X. ENTER cuando esté listo (q sale): ")
    if prompt("  > ") == "q": return None
    start = node.wait_for_pose()
    if start is None:
        print("  ERROR: sin lectura de odometría global."); return None
    print(f"  START: x={start.x:+.3f} y={start.y:+.3f} yaw={math.degrees(start.yaw):+.1f}°")

    for side in range(4):
        print(f"\n  Lado {side+1}/4 ({direction}): conduce {side_m:.2f} m forward.")
        print("    (Idealmente con cmd_vel constante y línea recta. Detén al final.)")
        if prompt("    ENTER al llegar: ") == "q": return None

        print(f"  Esquina {side+1}/4: gira {'90° izquierda' if sign>0 else '90° derecha'} en sitio.")
        print("    (cmd_vel solo angular, hasta que estés alineado al siguiente lado.)")
        if prompt("    ENTER al alinear: ") == "q": return None

    end = node.wait_for_pose()
    if end is None:
        print("  ERROR: sin lectura final."); return None
    print(f"  END:   x={end.x:+.3f} y={end.y:+.3f} yaw={math.degrees(end.yaw):+.1f}°")

    # Closure error: end pose vs start pose, in start frame
    dx = end.x - start.x
    dy = end.y - start.y
    cs, sn = math.cos(-start.yaw), math.sin(-start.yaw)
    closure_x = cs * dx - sn * dy
    closure_y = sn * dx + cs * dy
    closure_yaw = angle_wrap(end.yaw - start.yaw)
    print(f"  Closure: Δx={closure_x*100:+.1f}cm Δy={closure_y*100:+.1f}cm "
          f"Δθ={math.degrees(closure_yaw):+.2f}°")
    return RunResult(direction=direction, run_idx=run_idx,
                     start=start, end=end,
                     closure_x=closure_x, closure_y=closure_y,
                     closure_yaw=closure_yaw)


def analyze(results: list[RunResult], side_m: float, b_nominal: float,
            wheel_radius_nominal: float) -> dict:
    cw = [r for r in results if r.direction == "CW"]
    ccw = [r for r in results if r.direction == "CCW"]
    if not cw or not ccw:
        return {}

    # Centre of gravity of endpoint clusters (Borenstein eq. 13-16)
    cg = lambda group, axis: statistics.mean(getattr(r, f"closure_{axis}") for r in group)
    x_cg_cw = cg(cw, "x");  y_cg_cw = cg(cw, "y")
    x_cg_ccw = cg(ccw, "x"); y_cg_ccw = cg(ccw, "y")

    # In Borenstein's convention, after a CW square the y-error of the
    # cluster's CG carries Type-A (orientation) error and CCW carries
    # the opposite sign. The exact formulation:
    #   α = (x_cg_cw + x_cg_ccw) / (-4 * L)   # rad
    #   β = (x_cg_cw - x_cg_ccw) / (-4 * L)   # rad
    L = side_m
    alpha = (x_cg_cw + x_cg_ccw) / (-4.0 * L)
    beta = (x_cg_cw - x_cg_ccw) / (-4.0 * L)

    # Type-A: wheel diameter asymmetry → curved trajectory
    # Type-B: wheelbase mismeasurement → over/under-rotation in turns
    # Note 90° expressed in radians: π/2
    half_pi = math.pi / 2.0
    Ed = (half_pi + alpha) / (half_pi - alpha)
    Eb = half_pi / (half_pi - beta)

    # Suggested corrections:
    # If Ed > 1, the right wheel is effectively larger than the left.
    # Apply a multiplicative correction split between the two wheels:
    #   r_left  = r_nominal * (2 / (Ed + 1))
    #   r_right = r_nominal * (2 * Ed / (Ed + 1))
    r_left = wheel_radius_nominal * (2.0 / (Ed + 1.0))
    r_right = wheel_radius_nominal * (2.0 * Ed / (Ed + 1.0))
    b_corrected = b_nominal * Eb

    return {
        "n_cw": len(cw), "n_ccw": len(ccw),
        "x_cg_cw_m": x_cg_cw, "y_cg_cw_m": y_cg_cw,
        "x_cg_ccw_m": x_cg_ccw, "y_cg_ccw_m": y_cg_ccw,
        "alpha_rad": alpha, "alpha_deg": math.degrees(alpha),
        "beta_rad": beta, "beta_deg": math.degrees(beta),
        "Ed": Ed, "Eb": Eb,
        "wheel_radius_nominal": wheel_radius_nominal,
        "wheel_radius_left_suggested": r_left,
        "wheel_radius_right_suggested": r_right,
        "track_width_nominal": b_nominal,
        "track_width_suggested": b_corrected,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--side", type=float, default=4.0,
                        help="square side length in meters (default 4.0)")
    parser.add_argument("--runs", type=int, default=5,
                        help="number of runs per direction (CW + CCW). "
                             "Borenstein recommends ≥ 5.")
    parser.add_argument("--mode", choices=["operator"], default="operator",
                        help="execution mode (currently only operator is "
                             "implemented; auto-mode requires bypassing "
                             "the backend stale-cmd watchdog).")
    parser.add_argument("--track-width", type=float, default=0.960,
                        help="nominal wheelbase b_nominal (default reads "
                             "current odrive_params.yaml value)")
    parser.add_argument("--wheel-radius", type=float, default=0.0781,
                        help="nominal wheel radius (default reads current "
                             "odrive_params.yaml value)")
    parser.add_argument("--out-dir", type=str,
                        default=str(Path(__file__).parent / "calib_runs"),
                        help="directory to write CSV (default: tools/calib_runs)")
    args = parser.parse_args(argv)

    rclpy.init()
    node = UmbmarkNode()

    # Sanity check: we need /agv/odometry/global to be live
    print("Esperando primera lectura de /agv/odometry/global…")
    pose = node.wait_for_pose(timeout_s=5.0)
    if pose is None:
        print("ERROR: no llega odometry/global. ¿Está corriendo el stack?")
        node.destroy_node(); rclpy.shutdown()
        return 1

    print(f"OK. side={args.side} m, runs={args.runs} per direction.")
    print(f"Nominal: wheel_radius={args.wheel_radius:.4f}, "
          f"track_width={args.track_width:.4f}")
    print()
    print("Protocol:")
    print("  Drive a square of side L, alternating CW and CCW.")
    print("  Operator decides exact sub-trajectory (joystick); the script")
    print("  only records the start and end pose for closure analysis.")
    print()

    results: list[RunResult] = []
    try:
        for i in range(1, args.runs + 1):
            for direction in ["CW", "CCW"]:
                r = run_operator_mode(node, args.side, direction, i)
                if r is None:
                    raise KeyboardInterrupt
                results.append(r)
    except KeyboardInterrupt:
        print("\nInterrumpido — analizando lo que tenemos.")

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%dT%H%M%S")
    csv_path = Path(args.out_dir) / f"umbmark_{ts}.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["direction", "run", "start_x", "start_y", "start_yaw_deg",
                     "end_x", "end_y", "end_yaw_deg",
                     "closure_x", "closure_y", "closure_yaw_deg"])
        for r in results:
            w.writerow([
                r.direction, r.run_idx,
                f"{r.start.x:.4f}", f"{r.start.y:.4f}",
                f"{math.degrees(r.start.yaw):.2f}",
                f"{r.end.x:.4f}", f"{r.end.y:.4f}",
                f"{math.degrees(r.end.yaw):.2f}",
                f"{r.closure_x:.4f}", f"{r.closure_y:.4f}",
                f"{math.degrees(r.closure_yaw):.2f}",
            ])
    print(f"\nCSV: {csv_path}")

    if results:
        a = analyze(results, args.side, args.track_width, args.wheel_radius)
        if a:
            print("\n══ UMBmark analysis (Borenstein & Feng 1996) ══")
            print(f"  N_CW={a['n_cw']}, N_CCW={a['n_ccw']}")
            print(f"  CW  endpoint CG: ({a['x_cg_cw_m']*100:+.2f}, {a['y_cg_cw_m']*100:+.2f}) cm")
            print(f"  CCW endpoint CG: ({a['x_cg_ccw_m']*100:+.2f}, {a['y_cg_ccw_m']*100:+.2f}) cm")
            print(f"  α (Type-A wheel-Ø asymmetry) = {a['alpha_deg']:+.4f}°")
            print(f"  β (Type-B wheelbase error)   = {a['beta_deg']:+.4f}°")
            print(f"  Ed = {a['Ed']:.6f}  →  r_left  = {a['wheel_radius_left_suggested']:.5f} m")
            print(f"                       r_right = {a['wheel_radius_right_suggested']:.5f} m")
            print(f"  Eb = {a['Eb']:.6f}  →  track_width = {a['track_width_suggested']:.5f} m")
            print()
            # Residual estimate: if we apply these corrections, what % bias is
            # left? Crude proxy: max(|Ed-1|, |Eb-1|).
            resid_pct = 100 * max(abs(a["Ed"] - 1.0), abs(a["Eb"] - 1.0))
            print(f"  Residual estimate after correction: {resid_pct:.2f}% (wheel/wheelbase)")
            print()
            if resid_pct > 5.0:
                print("  ⚠️  Residual > 5%: do NOT simply apply these corrections.")
                print("      Likely a non-geometric source dominates (caster slip,")
                print("      surface, payload). Re-run on a different surface or")
                print("      consider Phase 2 (slip detector) before adjusting geometry.")
            else:
                print("  ✓ Residual < 5%: corrections likely worth applying.")
                print("    Edit src/agv_odrive/config/odrive_params.yaml:")
                print(f"      wheel_radius:  {(a['wheel_radius_left_suggested']+a['wheel_radius_right_suggested'])/2:.5f}  # was {a['wheel_radius_nominal']}")
                print(f"      track_width:   {a['track_width_suggested']:.5f}  # was {a['track_width_nominal']}")
                print("    Then re-run UMBmark to validate residual ≤ 1%.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
