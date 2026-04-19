"""
Validation script - checks all dependencies and subsystems before launch.
Run: python3 -m agv_odrive.validate
"""
import sys
import subprocess


def check(name, func):
    try:
        result = func()
        if result is True or result is None:
            print(f"  [OK]  {name}")
            return True
        else:
            print(f"  [FAIL] {name}: {result}")
            return False
    except Exception as e:
        print(f"  [FAIL] {name}: {e}")
        return False


def check_python_module(module_name):
    def _check():
        __import__(module_name)
        return True
    return _check


def check_can_interface():
    result = subprocess.run(['ip', 'link', 'show', 'can0'], capture_output=True, text=True)
    if result.returncode != 0:
        return "can0 interface not found"
    if 'UP' not in result.stdout:
        return "can0 is DOWN. Run: sudo ip link set can0 up type can bitrate 250000"
    return True


def check_can_bitrate():
    result = subprocess.run(
        ['ip', '-details', 'link', 'show', 'can0'], capture_output=True, text=True
    )
    if 'bitrate 250000' in result.stdout:
        return True
    return f"Unexpected bitrate. Expected 250000. Output: {result.stdout.strip()}"


def check_ros2():
    try:
        import rclpy
        rclpy.init()
        rclpy.shutdown()
        return True
    except Exception as e:
        return str(e)


def check_glfw_window():
    import glfw
    if not glfw.init():
        return "GLFW init failed"
    window = glfw.create_window(100, 100, "test", None, None)
    if not window:
        glfw.terminate()
        return "Cannot create GLFW window (no display?)"
    glfw.destroy_window(window)
    glfw.terminate()
    return True


def check_imgui_api():
    import imgui
    required = [
        'create_context', 'new_frame', 'begin', 'end', 'render',
        'button', 'slider_float', 'text', 'text_colored',
        'separator', 'spacing', 'same_line', 'columns', 'next_column',
        'push_style_color', 'pop_style_color', 'plot_lines',
        'set_next_window_position', 'set_next_window_size',
        'get_content_region_available', 'get_style',
    ]
    missing = [f for f in required if not hasattr(imgui, f)]
    if missing:
        return f"Missing imgui functions: {missing}"
    return True


def check_can_send_recv():
    """Try to open CAN socket (non-destructive)."""
    try:
        import can
        bus = can.interface.Bus(channel='can0', interface='socketcan')
        msg = bus.recv(timeout=0.5)
        bus.shutdown()
        if msg:
            node_id = msg.arbitration_id >> 5
            cmd_id = msg.arbitration_id & 0x1F
            return True  # Got a message (likely heartbeat)
        return "No CAN messages received in 0.5s. Is ODrive powered on?"
    except Exception as e:
        return str(e)


def main():
    print("\n=== AGV ODrive Validation ===\n")
    all_ok = True

    print("[1/7] Python modules:")
    for mod in ['can', 'rclpy', 'imgui', 'glfw', 'OpenGL']:
        if not check(mod, check_python_module(mod)):
            all_ok = False

    print("\n[2/7] ROS 2:")
    if not check("rclpy init/shutdown", check_ros2):
        all_ok = False

    print("\n[3/7] CAN interface:")
    if not check("can0 UP", check_can_interface):
        all_ok = False
    else:
        check("can0 bitrate 250000", check_can_bitrate)

    print("\n[4/7] CAN communication:")
    if not check("Receive CAN message from ODrive", check_can_send_recv):
        all_ok = False

    print("\n[5/7] OpenGL / GLFW:")
    if not check("Create GLFW window", check_glfw_window):
        all_ok = False

    print("\n[6/7] ImGui API:")
    if not check("Required imgui functions", check_imgui_api):
        all_ok = False

    print("\n[7/7] ROS 2 package:")
    result = subprocess.run(
        ['bash', '-c', 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 pkg list | grep agv_odrive'],
        capture_output=True, text=True
    )
    if not check("agv_odrive installed", lambda: True if 'agv_odrive' in result.stdout else "Package not found. Run: colcon build --packages-select agv_odrive"):
        all_ok = False

    print("\n" + "=" * 35)
    if all_ok:
        print("ALL CHECKS PASSED - Ready to launch!")
        print("\n  ros2 launch agv_odrive odrive_gui.launch.py\n")
    else:
        print("SOME CHECKS FAILED - Fix issues above before launching.")
    print("=" * 35 + "\n")

    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
