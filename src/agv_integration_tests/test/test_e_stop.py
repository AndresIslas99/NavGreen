#!/usr/bin/env python3
"""Integration test: E-stop propagation through the ROS graph.

Publishes true on /<ns>/e_stop and asserts the ODrive driver acknowledges
it — drive_debug JSON reports e_stop:true with zeroed wheel targets —
within the acceptance.yaml command-to-stop budget plus an observation
allowance, then that measured wheel speed reaches zero.

Observability note: drive_debug publishes at 10 Hz, so the measured
latency includes up to one 100 ms reporting period plus DDS transport
time on top of the actual command-to-stop time. The strict 0.2 s
command-to-physical-stop gate (specs/acceptance.yaml
hardware.pass_condition) is validated on hardware/HIL with external
timing instrumentation, not here.

Safety: the test releases the e-stop only if it latched it itself; if an
e-stop is already active it skips rather than clear an operator's stop.
Running this against a live robot stops it — that is the point — so it
only runs when AGV_STACK_TEST=1 is set with the full stack running.
"""
import json
import os
import time

import pytest

try:
    import rclpy
    from std_msgs.msg import Bool, String
except ImportError:
    pytest.skip("rclpy not available", allow_module_level=True)

if os.environ.get("AGV_STACK_TEST") != "1":
    pytest.skip(
        "stack-required test: set AGV_STACK_TEST=1 with the full stack "
        "running (agv_full.launch.py)", allow_module_level=True)

NS = os.environ.get("AGV_NAMESPACE", "agv")

DISCOVERY_TIMEOUT_S = 10.0
# acceptance.yaml hardware gate: "E-stop command-to-stop <= 0.2 s".
E_STOP_BUDGET_S = 0.2
# One 10 Hz drive_debug reporting period + DDS/executor margin.
OBSERVATION_ALLOWANCE_S = 0.3
# Measured wheel speed below this counts as stopped (motor turns/s).
STOP_SPEED_TURNS_S = 0.05
WHEEL_STOP_EXTRA_S = 1.0


def _spin_until(node, deadline, predicate):
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_e_stop_propagation():
    rclpy.init()
    node = rclpy.create_node('test_e_stop_probe')
    debug = {}

    def on_debug(msg):
        try:
            debug.update(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    # Keep the subscription handle alive for the duration of the test.
    debug_sub = node.create_subscription(
        String, f'/{NS}/drive_debug', on_debug, 10)
    pub = node.create_publisher(Bool, f'/{NS}/e_stop', 10)
    we_latched = False
    try:
        deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
        assert _spin_until(node, deadline,
                           lambda: pub.get_subscription_count() > 0), \
            f"no subscriber discovered on /{NS}/e_stop (is agv_odrive running?)"
        assert _spin_until(node, deadline, lambda: 'e_stop' in debug), \
            f"no /{NS}/drive_debug traffic (is agv_odrive running?)"

        if debug['e_stop']:
            pytest.skip("e-stop already latched; release it before running "
                        "this test (refusing to clear an operator stop)")

        t0 = time.monotonic()
        we_latched = True
        pub.publish(Bool(data=True))

        acked = _spin_until(
            node, t0 + E_STOP_BUDGET_S + OBSERVATION_ALLOWANCE_S,
            lambda: debug.get('e_stop') is True
            and abs(debug.get('left_target', 1.0)) < 1e-6
            and abs(debug.get('right_target', 1.0)) < 1e-6)
        latency = time.monotonic() - t0
        assert acked, (
            f"e_stop not acknowledged with zeroed wheel targets within "
            f"{E_STOP_BUDGET_S} s budget + {OBSERVATION_ALLOWANCE_S} s "
            f"observation allowance (waited {latency:.2f} s): {debug}")
        print(f"e_stop acknowledged in {latency:.3f} s")

        stopped = _spin_until(
            node, time.monotonic() + WHEEL_STOP_EXTRA_S,
            lambda: abs(debug.get('left_meas', 1.0)) < STOP_SPEED_TURNS_S
            and abs(debug.get('right_meas', 1.0)) < STOP_SPEED_TURNS_S)
        assert stopped, f"wheels still moving after e_stop: {debug}"
    finally:
        if we_latched:
            pub.publish(Bool(data=False))
            # Spin briefly so the release is delivered before shutdown.
            release_deadline = time.monotonic() + 0.5
            while time.monotonic() < release_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_subscription(debug_sub)
        node.destroy_node()
        rclpy.shutdown()
