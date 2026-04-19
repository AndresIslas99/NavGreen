"""
ODrive OpenGL GUI - Dear ImGui interface for motor control.
Provides sliders, buttons, and real-time feedback visualization.
"""
import array
import collections
import threading
import time

import glfw
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String


def section_header(label):
    """separator_text replacement for older imgui."""
    imgui.spacing()
    imgui.separator()
    imgui.text(label)
    imgui.separator()
    imgui.spacing()


class ODriveGUINode(Node):
    def __init__(self):
        super().__init__('odrive_gui')

        # Publishers
        self.pub_vel = self.create_publisher(Float32, 'odrive/cmd_vel', 10)
        self.pub_state = self.create_publisher(Float32, 'odrive/set_state', 10)
        self.pub_clear = self.create_publisher(Float32, 'odrive/clear_errors', 10)

        # Subscribers
        self.create_subscription(Float32, 'odrive/velocity', self.vel_cb, 10)
        self.create_subscription(Float32, 'odrive/position', self.pos_cb, 10)
        self.create_subscription(Float32, 'odrive/vbus_voltage', self.vbus_cb, 10)
        self.create_subscription(Int32, 'odrive/axis_state', self.state_cb, 10)
        self.create_subscription(Int32, 'odrive/error_code', self.error_code_cb, 10)
        self.create_subscription(Int32, 'odrive/disarm_reason', self.disarm_cb, 10)
        self.create_subscription(String, 'odrive/errors', self.errors_cb, 10)

        # State
        self.vel_feedback = 0.0
        self.pos_feedback = 0.0
        self.vbus_voltage = 0.0
        self.cmd_vel = 0.0
        self.motor_enabled = False
        self.axis_state = 0
        self.axis_state_name = "UNKNOWN"
        self.error_code = 0
        self.disarm_reason = 0
        self.error_names = []
        self.disarm_names = []

        # History for plots
        self.max_history = 200
        self.vel_history = collections.deque([0.0] * self.max_history, maxlen=self.max_history)
        self.pos_history = collections.deque([0.0] * self.max_history, maxlen=self.max_history)

    def vel_cb(self, msg):
        self.vel_feedback = msg.data
        self.vel_history.append(msg.data)

    def pos_cb(self, msg):
        self.pos_feedback = msg.data
        self.pos_history.append(msg.data)

    def vbus_cb(self, msg):
        self.vbus_voltage = msg.data

    def state_cb(self, msg):
        self.axis_state = msg.data

    def error_code_cb(self, msg):
        self.error_code = msg.data

    def disarm_cb(self, msg):
        self.disarm_reason = msg.data

    def errors_cb(self, msg):
        # Parse "errors:X,Y|disarm:A,B|state:NAME"
        parts = msg.data.split('|')
        for part in parts:
            if part.startswith('errors:'):
                self.error_names = part[7:].split(',')
            elif part.startswith('disarm:'):
                self.disarm_names = part[7:].split(',')
            elif part.startswith('state:'):
                self.axis_state_name = part[6:]

    def send_vel(self, vel):
        self.cmd_vel = vel
        msg = Float32()
        msg.data = vel
        self.pub_vel.publish(msg)

    def send_state(self, state):
        msg = Float32()
        msg.data = float(state)
        self.pub_state.publish(msg)

    def clear_errors(self):
        msg = Float32()
        msg.data = 0.0
        self.pub_clear.publish(msg)

    def enable_motor(self):
        self.motor_enabled = True
        self.send_state(8)  # CLOSED_LOOP_CONTROL

    def disable_motor(self):
        self.motor_enabled = False
        self.send_vel(0.0)
        self.send_state(1)  # IDLE

    def emergency_stop(self):
        self.motor_enabled = False
        self.cmd_vel = 0.0
        self.send_vel(0.0)
        self.send_state(1)


def run_gui(node):
    if not glfw.init():
        raise RuntimeError("Could not initialize GLFW")

    window = glfw.create_window(780, 700, "ODrive Control Panel", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("Could not create GLFW window")

    glfw.make_context_current(window)
    imgui.create_context()
    impl = GlfwRenderer(window)

    # Style
    style = imgui.get_style()
    style.window_rounding = 6.0
    style.frame_rounding = 4.0
    style.grab_rounding = 4.0
    style.colors[imgui.COLOR_WINDOW_BACKGROUND] = (0.12, 0.12, 0.14, 1.0)
    style.colors[imgui.COLOR_TITLE_BACKGROUND_ACTIVE] = (0.2, 0.4, 0.7, 1.0)
    style.colors[imgui.COLOR_BUTTON] = (0.2, 0.4, 0.7, 1.0)
    style.colors[imgui.COLOR_BUTTON_HOVERED] = (0.3, 0.5, 0.8, 1.0)
    style.colors[imgui.COLOR_SLIDER_GRAB] = (0.3, 0.6, 1.0, 1.0)

    vel_slider = [0.0]

    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        imgui.new_frame()

        w, h = glfw.get_window_size(window)
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(w, h)

        imgui.begin(
            "ODrive Motor Control",
            flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_COLLAPSE,
        )

        # === Status / Errors Section ===
        section_header("ODrive Status")

        has_errors = node.error_code != 0
        has_disarm = node.disarm_reason != 0

        # Axis state
        imgui.text("Axis State: ")
        imgui.same_line()
        if node.axis_state == 8:
            imgui.text_colored(node.axis_state_name, 0.2, 0.9, 0.2, 1.0)
        elif node.axis_state == 1:
            imgui.text_colored(node.axis_state_name, 0.9, 0.9, 0.2, 1.0)
        else:
            imgui.text_colored(node.axis_state_name, 0.6, 0.6, 0.6, 1.0)

        # Active errors
        if has_errors:
            imgui.text_colored(
                f"ACTIVE ERRORS (0x{node.error_code:08X}):",
                1.0, 0.2, 0.2, 1.0,
            )
            for err_name in node.error_names:
                if err_name != "NONE":
                    imgui.same_line()
                    imgui.text_colored(f"  [{err_name}]", 1.0, 0.4, 0.4, 1.0)
        else:
            imgui.text_colored("No active errors", 0.2, 0.9, 0.2, 1.0)

        # Disarm reason
        if has_disarm:
            imgui.text_colored(
                f"DISARM REASON (0x{node.disarm_reason:08X}):",
                1.0, 0.5, 0.2, 1.0,
            )
            for reason in node.disarm_names:
                if reason != "NONE":
                    imgui.same_line()
                    imgui.text_colored(f"  [{reason}]", 1.0, 0.6, 0.3, 1.0)

        # Clear errors button
        if has_errors or has_disarm:
            imgui.spacing()
            imgui.push_style_color(imgui.COLOR_BUTTON, 0.6, 0.4, 0.0, 1.0)
            imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.8, 0.6, 0.1, 1.0)
            if imgui.button("CLEAR ERRORS", width=160, height=30):
                node.clear_errors()
            imgui.pop_style_color(2)

        # === Motor State Section ===
        section_header("Motor Control")

        if not node.motor_enabled:
            if imgui.button("ENABLE MOTOR", width=200, height=40):
                node.enable_motor()
        else:
            imgui.push_style_color(imgui.COLOR_BUTTON, 0.7, 0.2, 0.2, 1.0)
            imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.9, 0.3, 0.3, 1.0)
            if imgui.button("DISABLE MOTOR", width=200, height=40):
                node.disable_motor()
                vel_slider[0] = 0.0
            imgui.pop_style_color(2)

        imgui.same_line(spacing=20)

        # Emergency stop
        imgui.push_style_color(imgui.COLOR_BUTTON, 0.8, 0.0, 0.0, 1.0)
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 1.0, 0.1, 0.1, 1.0)
        if imgui.button("EMERGENCY STOP", width=200, height=40):
            node.emergency_stop()
            vel_slider[0] = 0.0
        imgui.pop_style_color(2)

        imgui.same_line(spacing=20)
        if node.motor_enabled:
            imgui.text_colored("ENABLED", 0.2, 0.9, 0.2, 1.0)
        else:
            imgui.text_colored("DISABLED", 0.6, 0.6, 0.6, 1.0)

        imgui.spacing()

        # === Velocity Control ===
        section_header("Velocity Control")

        changed, vel_slider[0] = imgui.slider_float(
            "Velocity (turns/s)", vel_slider[0], -10.0, 10.0, "%.1f"
        )
        if changed and node.motor_enabled:
            node.send_vel(vel_slider[0])

        # Preset buttons
        presets = [-5.0, -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0, 5.0]
        for i, p in enumerate(presets):
            if i > 0:
                imgui.same_line()
            label = "STOP" if p == 0.0 else f"{p:+.1f}"
            if p == 0.0:
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.6, 0.5, 0.0, 1.0)
                imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.8, 0.7, 0.1, 1.0)
            if imgui.button(label, width=60):
                vel_slider[0] = p
                if node.motor_enabled:
                    node.send_vel(p)
            if p == 0.0:
                imgui.pop_style_color(2)

        imgui.spacing()

        # === Feedback ===
        section_header("Feedback")

        imgui.columns(4)
        imgui.text("Command")
        imgui.text_colored(f"{node.cmd_vel:+.2f} turns/s", 0.3, 0.7, 1.0, 1.0)
        imgui.next_column()
        imgui.text("Velocity")
        imgui.text_colored(f"{node.vel_feedback:+.2f} turns/s", 0.3, 1.0, 0.5, 1.0)
        imgui.next_column()
        imgui.text("Position")
        imgui.text_colored(f"{node.pos_feedback:+.2f} turns", 1.0, 0.8, 0.3, 1.0)
        imgui.next_column()
        imgui.text("Vbus")
        imgui.text_colored(f"{node.vbus_voltage:.1f} V", 1.0, 0.5, 0.5, 1.0)
        imgui.columns(1)

        imgui.spacing()

        # === Plots ===
        section_header("Velocity Plot")
        vel_data = array.array('f', node.vel_history)
        imgui.plot_lines(
            "##vel_plot",
            vel_data,
            overlay_text=f"vel: {node.vel_feedback:.2f}",
            scale_min=-10.0,
            scale_max=10.0,
            graph_size=(imgui.get_content_region_available()[0], 80),
        )

        section_header("Position Plot")
        pos_data = array.array('f', node.pos_history)
        pos_min = min(pos_data) - 1.0 if len(pos_data) > 0 else -1.0
        pos_max = max(pos_data) + 1.0 if len(pos_data) > 0 else 1.0
        imgui.plot_lines(
            "##pos_plot",
            pos_data,
            overlay_text=f"pos: {node.pos_feedback:.2f}",
            scale_min=pos_min,
            scale_max=pos_max,
            graph_size=(imgui.get_content_region_available()[0], 80),
        )

        imgui.end()

        # Render
        gl.glClearColor(0.1, 0.1, 0.12, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = ODriveGUINode()

    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
