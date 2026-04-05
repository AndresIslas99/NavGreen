"""agv_ui_backend — Legacy Python modules for the AGV operator dashboard backend.

These modules have been superseded by the TypeScript implementation in src/.
They remain as reference but are not used in the active backend server.

Legacy modules:
    py_state_machine: Robot state derivation and action guards.
    py_event_log: Persistent event logging (JSONL format).
    py_camera_handler: Image processing utilities (offloaded to agv_image_server C++ node).
    py_scan_accumulator: LaserScan buffering for live map rendering.

Active backend:
    The production UI backend is TypeScript (src/index.ts) using Express + rclnodejs.
    See src/agv_ui_backend/src/ for the current implementation.
"""
