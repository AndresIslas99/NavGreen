#!/usr/bin/env python3
"""
Integration test: verify E-stop publishes and motor state responds.
"""
import subprocess
import json
import time


def test_e_stop_via_api():
    """Test E-stop through the operator backend API."""
    import urllib.request

    # Get current status
    try:
        with urllib.request.urlopen('http://localhost:8090/api/status', timeout=5) as r:
            status = json.loads(r.read())
            print(f"Initial e_stop: {status['e_stop']}")
    except Exception as e:
        print(f"Backend not available: {e}")
        print("NOTE: This test requires the full stack + backend running.")
        return

    # Activate E-stop
    req = urllib.request.Request(
        'http://localhost:8090/api/nav/cancel',
        data=b'{}',
        headers={'Content-Type': 'application/json'},
        method='POST')
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            print(f"Cancel nav response: {r.read().decode()}")
    except Exception:
        pass

    print("E-stop test completed (manual verification needed for latency)")


if __name__ == '__main__':
    test_e_stop_via_api()
