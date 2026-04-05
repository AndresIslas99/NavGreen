"""agv_odrive — Development and commissioning tools for ODrive S1 motor controllers.

This Python package contains dev-only tools (not part of the production robot runtime).
The production driver is the C++17 odrive_can_node.

Modules:
    odrive_can_node: Python CAN validation node for commissioning.
    odrive_gui: ImGui diagnostic GUI for real-time motor tuning and monitoring.
    validate: Hardware validation script (CAN bus, OpenGL, ROS2, ODrive connectivity).
    simple_test: Basic CAN connectivity smoke test.

Note:
    Marked dev_only: true in TASK.yaml. These tools must be replaced with C++17
    equivalents before production deployment per workspace engineering rules.
"""
