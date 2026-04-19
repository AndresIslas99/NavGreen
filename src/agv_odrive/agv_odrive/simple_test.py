"""
Simple test node for ODrive - keyboard-like control via terminal.
Publishes velocity commands and state changes.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import termios
import tty

INSTRUCTIONS = """
=== ODrive Simple Test ===
Controls:
  e : Enable motor (closed loop)
  d : Disable motor (idle)
  w : Increase velocity (+0.5 turns/s)
  s : Decrease velocity (-0.5 turns/s)
  x : Stop (velocity = 0)
  q : Quit
==========================
"""


class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('odrive_simple_test')
        self.pub_vel = self.create_publisher(Float32, 'odrive/cmd_vel', 10)
        self.pub_state = self.create_publisher(Float32, 'odrive/set_state', 10)

        # Subscribe to feedback
        self.create_subscription(Float32, 'odrive/velocity', self.vel_fb_callback, 10)
        self.create_subscription(Float32, 'odrive/position', self.pos_fb_callback, 10)

        self.current_vel_cmd = 0.0
        self.vel_step = 0.5
        self.last_vel_fb = 0.0
        self.last_pos_fb = 0.0

    def vel_fb_callback(self, msg):
        self.last_vel_fb = msg.data

    def pos_fb_callback(self, msg):
        self.last_pos_fb = msg.data

    def send_vel(self, vel):
        msg = Float32()
        msg.data = vel
        self.pub_vel.publish(msg)

    def send_state(self, state):
        msg = Float32()
        msg.data = float(state)
        self.pub_state.publish(msg)

    def print_status(self):
        print(
            f'\r  Cmd: {self.current_vel_cmd:+.1f} turns/s | '
            f'FB vel: {self.last_vel_fb:+.2f} | '
            f'FB pos: {self.last_pos_fb:+.2f}    ',
            end='', flush=True
        )


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTestNode()

    print(INSTRUCTIONS)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            key = None
            # Non-blocking key read
            import select
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = get_key()

            if key == 'e':
                print('\n>> Enabling motor (closed loop)')
                node.send_state(8)
            elif key == 'd':
                print('\n>> Disabling motor (idle)')
                node.current_vel_cmd = 0.0
                node.send_vel(0.0)
                node.send_state(1)
            elif key == 'w':
                node.current_vel_cmd += node.vel_step
                node.send_vel(node.current_vel_cmd)
            elif key == 's':
                node.current_vel_cmd -= node.vel_step
                node.send_vel(node.current_vel_cmd)
            elif key == 'x':
                node.current_vel_cmd = 0.0
                node.send_vel(0.0)
                print('\n>> STOP')
            elif key == 'q':
                print('\n>> Quitting...')
                node.send_vel(0.0)
                node.send_state(1)
                break

            node.print_status()

    except KeyboardInterrupt:
        node.send_vel(0.0)
        node.send_state(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
