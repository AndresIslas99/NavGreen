#!/usr/bin/env python3
"""sim_obstacle_relay — HIL-only collision_monitor_state publisher.

Why this exists
---------------
`rail_driver` and `mode_arbiter` both subscribe to
`/agv/collision_monitor_state` expecting `std_msgs/msg/String` with
content "stop" / "slowdown" / "clear". In production the Nav2
`collision_monitor` + a safety translator publishes that string. In
HIL we run Nav2's collision_monitor (which publishes
`nav2_msgs/msg/CollisionMonitorState`, a different type) and no
translator, so the topic has no String publisher — rail_driver never
sees a halt signal and marches straight into the crates in the USD
gap (e.g. Crate1 at (3.5, -2.0) directly on the c5_drive_in path).

This relay bridges the gap using the sim's oracle:
  - `/agv/sim/ground_truth/obstacles` (latched JSON list of static
    obstacles baked into the USD, emitted once at sim startup).
  - `/agv/sim/ground_truth/pose` (10 Hz robot pose).

On every pose tick it computes robot-frame distance to each obstacle
AABB, clips to the forward half-plane (robot cannot back-collide in
HIL validation scenarios), and publishes `stop` if any obstacle edge
is within `stop_distance_m` ahead, else `clear`. This mirrors what a
real 2D LIDAR collision_monitor would produce, minus the FoV noise.

IMPORTANT: this is HIL-only. Real deployments keep Nav2's collision
chain on real LIDAR scans. Gated via `agv_hil_full.launch.py` + TASK
`dev_only: true`.
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class SimObstacleRelay(Node):
    def __init__(self) -> None:
        super().__init__(
            "sim_obstacle_relay",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )
        self.declare_parameter("obstacles_topic", "/agv/sim/ground_truth/obstacles")
        self.declare_parameter("pose_topic", "/agv/sim/ground_truth/pose")
        self.declare_parameter("state_topic", "/agv/collision_monitor_state")
        self.declare_parameter("stop_distance_m", 0.50)
        self.declare_parameter("slowdown_distance_m", 1.00)
        self.declare_parameter("robot_half_width_m", 0.42)
        self.declare_parameter("publish_rate_hz", 10.0)

        self._stop_d = float(self.get_parameter("stop_distance_m").value)
        self._slow_d = float(self.get_parameter("slowdown_distance_m").value)
        self._half_w = float(self.get_parameter("robot_half_width_m").value)

        self._obstacles: list[tuple[float, float, float, float]] = []
        self._robot: tuple[float, float, float] | None = None

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            String,
            self.get_parameter("obstacles_topic").value,
            self._on_obstacles, latched,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose, rel,
        )
        self._pub = self.create_publisher(
            String,
            self.get_parameter("state_topic").value, rel,
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(
            f"sim_obstacle_relay up: stop<{self._stop_d:.2f}m, "
            f"slow<{self._slow_d:.2f}m, half_w={self._half_w:.2f}m, "
            f"rate={rate:.1f}Hz")

    def _on_obstacles(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("obstacles payload is not valid JSON")
            return
        obstacles = []
        for o in payload.get("static_obstacles", []):
            # Sim oracle format: `pose: [x, y, z]`, `bbox_m: [sx, sy, sz]`
            # (FULL dimensions, not half-extents), `yaw_deg`.
            pose = o.get("pose")
            bbox = o.get("bbox_m")
            if not (isinstance(pose, list) and isinstance(bbox, list) and
                    len(pose) >= 2 and len(bbox) >= 2):
                continue
            x = float(pose[0])
            y = float(pose[1])
            hx = float(bbox[0]) / 2.0
            hy = float(bbox[1]) / 2.0
            # Yawed-rectangle → tight AABB in world frame:
            # half_ext_x = |hx·cos| + |hy·sin|
            # half_ext_y = |hx·sin| + |hy·cos|
            yaw = math.radians(float(o.get("yaw_deg", 0.0)))
            c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
            sx = hx * c + hy * s
            sy = hx * s + hy * c
            obstacles.append((x, y, sx, sy))
        self._obstacles = obstacles
        self.get_logger().info(f"loaded {len(obstacles)} obstacle AABBs from oracle")

    def _on_pose(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        # Iter-41 guard: drop frames with NaN/inf — sim unstick events and
        # mid-teleport ticks can emit non-finite components that propagate
        # into rx_rel/ry_rel and NaN-short-circuit the skip check, falsely
        # triggering "stop" for tens of seconds.
        if not all(math.isfinite(v) for v in (x, y, qz, qw)):
            return
        yaw = 2.0 * math.atan2(qz, qw)
        self._robot = (x, y, yaw)

    def _on_tick(self) -> None:
        if self._robot is None or not self._obstacles:
            # No pose or no obstacles known: default to "clear" so the
            # rail_driver doesn't freeze waiting for a signal it will
            # never receive in a degraded state.
            self._pub.publish(String(data="clear"))
            return
        rx, ry, ryaw = self._robot
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)
        nearest_forward = float("inf")
        for (ox, oy, sx, sy) in self._obstacles:
            # Expand AABB by robot half-width so the check is corner-
            # inclusive.
            sxe = sx + self._half_w
            sye = sy + self._half_w
            # Translate into robot frame.
            dx = ox - rx
            dy = oy - ry
            rx_rel = dx * cos_y + dy * sin_y
            ry_rel = -dx * sin_y + dy * cos_y
            # Only care about obstacles ahead (rx_rel > 0) and within
            # the lateral corridor.
            if abs(ry_rel) > sye:
                continue
            # Obstacle x-range in robot frame: [rx_rel-sxe, rx_rel+sxe].
            # Three cases:
            #   fully behind (rx_rel + sxe <= 0): skip — can't collide going forward.
            #   overlapping origin (rx_rel - sxe <= 0 < rx_rel + sxe): robot inside
            #     expanded AABB → stop now.
            #   fully ahead (rx_rel - sxe > 0): distance = rx_rel - sxe.
            if rx_rel + sxe <= 0.0:
                continue
            if rx_rel - sxe <= 0.0:
                nearest_forward = 0.0
                break
            forward_edge = rx_rel - sxe
            if forward_edge < nearest_forward:
                nearest_forward = forward_edge
        if nearest_forward <= self._stop_d:
            state = "stop"
        elif nearest_forward <= self._slow_d:
            state = "slowdown"
        else:
            state = "clear"
        self._pub.publish(String(data=state))


def main() -> None:
    rclpy.init()
    node = SimObstacleRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
