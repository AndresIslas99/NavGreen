"""
ODrive CAN Node - Simple driver for ODrive over CAN bus.
Publishes encoder feedback, subscribes to velocity commands.
Uses ODrive CAN protocol (flat endpoint messages).
"""
import struct
import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
from geometry_msgs.msg import Twist


# ODrive CAN command IDs (5-bit, combined as (node_id << 5) | cmd_id)
class ODriveCmdID:
    HEARTBEAT = 0x01
    ESTOP = 0x02
    GET_ERROR = 0x03
    SET_AXIS_STATE = 0x07
    GET_ENCODER_ESTIMATES = 0x09
    SET_INPUT_VEL = 0x0D
    GET_TEMPERATURE = 0x15
    GET_VBUS_VOLTAGE = 0x17
    CLEAR_ERRORS = 0x18
    REBOOT = 0x16


class AxisState:
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8

    NAMES = {
        0: "UNDEFINED",
        1: "IDLE",
        2: "STARTUP_SEQUENCE",
        3: "FULL_CALIBRATION",
        4: "MOTOR_CALIBRATION",
        7: "ENCODER_CALIBRATION",
        8: "CLOSED_LOOP",
    }


# ODrive active error flags (from ODrive firmware)
ODRIVE_ERRORS = {
    0x00000001: "INITIALIZING",
    0x00000002: "SYSTEM_ERROR",
    0x00000004: "TIMING_ERROR",
    0x00000008: "MISSING_ESTIMATE",
    0x00000010: "BAD_CONFIG",
    0x00000020: "DRV_FAULT",
    0x00000040: "MISSING_INPUT",
    0x00000080: "DC_BUS_OVER_VOLTAGE",
    0x00000100: "DC_BUS_UNDER_VOLTAGE",
    0x00000200: "DC_BUS_OVER_CURRENT",
    0x00000400: "DC_BUS_OVER_REGEN_CURRENT",
    0x00000800: "CURRENT_LIMIT_VIOLATION",
    0x00001000: "MOTOR_OVER_TEMP",
    0x00002000: "INVERTER_OVER_TEMP",
    0x00004000: "VELOCITY_LIMIT_VIOLATION",
    0x00008000: "POSITION_LIMIT_VIOLATION",
    0x01000000: "WATCHDOG_TIMER_EXPIRED",
    0x02000000: "ESTOP_REQUESTED",
    0x04000000: "SPINOUT_DETECTED",
    0x08000000: "BRAKE_RESISTOR_DISARMED",
    0x10000000: "THERMISTOR_DISCONNECTED",
    0x40000000: "CALIBRATION_ERROR",
}

DISARM_REASONS = {
    0x00000001: "ESTOP",
    0x00000002: "PROCEDURE_FAILED",
    0x00000004: "WATCHDOG",
    0x00000008: "BRAKE_RESISTOR_DISARMED",
    0x00000010: "BAD_CONFIG",
    0x00000020: "MISSING_ESTIMATE",
    0x00000040: "MISSING_INPUT",
    0x00000080: "DC_BUS_OVER_VOLTAGE",
    0x00000100: "DC_BUS_UNDER_VOLTAGE",
    0x00000200: "DC_BUS_OVER_CURRENT",
    0x00000400: "DC_BUS_OVER_REGEN_CURRENT",
    0x00000800: "CURRENT_LIMIT_VIOLATION",
    0x00001000: "MOTOR_OVER_TEMP",
    0x00002000: "INVERTER_OVER_TEMP",
    0x00004000: "VELOCITY_LIMIT_VIOLATION",
    0x00008000: "POSITION_LIMIT_VIOLATION",
    0x01000000: "SPINOUT_DETECTED",
    0x40000000: "CALIBRATION_ERROR",
}


def decode_flags(value, flag_dict):
    """Decode a bitmask into a list of flag names."""
    if value == 0:
        return ["NONE"]
    flags = []
    for bit, name in flag_dict.items():
        if value & bit:
            flags.append(name)
    unknown = value & ~sum(flag_dict.keys())
    if unknown:
        flags.append(f"UNKNOWN(0x{unknown:08X})")
    return flags


class ODriveCANNode(Node):
    def __init__(self):
        super().__init__('odrive_can_node')

        # Parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('node_id', 0)
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('can_retry_interval', 2.0)
        self.declare_parameter('can_max_retries', 15)

        self.can_interface = self.get_parameter('can_interface').value
        self.node_id = self.get_parameter('node_id').value
        self.retry_interval = self.get_parameter('can_retry_interval').value
        self.max_retries = self.get_parameter('can_max_retries').value

        # CAN bus - connect with retries
        self.bus = None
        self.connected = False
        self._connect_can()

        # Publishers
        self.pub_velocity = self.create_publisher(Float32, 'odrive/velocity', 10)
        self.pub_position = self.create_publisher(Float32, 'odrive/position', 10)
        self.pub_vbus = self.create_publisher(Float32, 'odrive/vbus_voltage', 10)
        self.pub_state = self.create_publisher(Int32, 'odrive/axis_state', 10)
        self.pub_errors = self.create_publisher(String, 'odrive/errors', 10)
        self.pub_error_code = self.create_publisher(Int32, 'odrive/error_code', 10)
        self.pub_disarm_reason = self.create_publisher(Int32, 'odrive/disarm_reason', 10)

        # Subscribers
        self.create_subscription(Float32, 'odrive/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.create_subscription(Float32, 'odrive/set_state', self.set_state_callback, 10)
        self.create_subscription(Float32, 'odrive/clear_errors', self.clear_errors_callback, 10)

        # Timers
        self.create_timer(0.02, self.read_can_messages)  # 50Hz read
        self.create_timer(0.5, self.request_encoder)      # 2Hz encoder request

        # Track last error to avoid spamming
        self._last_error = 0
        self._last_disarm = 0

        # Reconnect timer if not connected
        if not self.connected:
            self._reconnect_attempts = 0
            self.reconnect_timer = self.create_timer(self.retry_interval, self._retry_connect)

    def _connect_can(self):
        try:
            self.bus = can.interface.Bus(
                channel=self.can_interface,
                interface='socketcan',
            )
            self.connected = True
            self.get_logger().info(
                f'Connected to CAN bus: {self.can_interface}, ODrive node_id: {self.node_id}'
            )
            self.get_logger().info('ODrive CAN node ready. Motor is IDLE.')
        except Exception as e:
            self.connected = False
            self.get_logger().warn(
                f'CAN bus not available yet ({self.can_interface}): {e}. Will retry...'
            )

    def _retry_connect(self):
        if self.connected:
            self.reconnect_timer.cancel()
            return
        self._reconnect_attempts += 1
        self.get_logger().info(
            f'Retry CAN connection {self._reconnect_attempts}/{self.max_retries}...'
        )
        self._connect_can()
        if self.connected:
            self.reconnect_timer.cancel()
        elif self._reconnect_attempts >= self.max_retries:
            self.get_logger().error(
                f'Failed to connect to CAN after {self.max_retries} attempts. Shutting down.'
            )
            self.reconnect_timer.cancel()
            raise SystemExit(1)

    def _make_can_id(self, cmd_id):
        return (self.node_id << 5) | cmd_id

    def _send_can(self, cmd_id, data, rtr=False):
        if not self.connected:
            return
        msg = can.Message(
            arbitration_id=self._make_can_id(cmd_id),
            data=data,
            is_extended_id=False,
            is_remote_frame=rtr,
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(f'CAN send error: {e}')

    def set_state_callback(self, msg):
        state = int(msg.data)
        self.get_logger().info(f'Setting axis state to {state}')
        data = struct.pack('<I', state) + b'\x00\x00\x00\x00'
        self._send_can(ODriveCmdID.SET_AXIS_STATE, data)

    def clear_errors_callback(self, msg):
        self.get_logger().info('Clearing ODrive errors')
        self._send_can(ODriveCmdID.CLEAR_ERRORS, b'\x00' * 8)

    def cmd_vel_callback(self, msg):
        vel = msg.data
        torque_ff = 0.0
        data = struct.pack('<ff', vel, torque_ff)
        self._send_can(ODriveCmdID.SET_INPUT_VEL, data)

    def twist_callback(self, msg):
        """Accept cmd_vel Twist - uses linear.x as velocity in turns/s."""
        vel = msg.linear.x
        torque_ff = 0.0
        data = struct.pack('<ff', vel, torque_ff)
        self._send_can(ODriveCmdID.SET_INPUT_VEL, data)

    def request_encoder(self):
        if not self.connected:
            return
        self._send_can(ODriveCmdID.GET_ENCODER_ESTIMATES, b'', rtr=True)

    def read_can_messages(self):
        if not self.connected:
            return
        try:
            msg = self.bus.recv(timeout=0.005)
        except can.CanError as e:
            self.get_logger().error(f'CAN recv error: {e}')
            return
        if msg is None:
            return

        cmd_id = msg.arbitration_id & 0x1F
        recv_node_id = msg.arbitration_id >> 5

        if recv_node_id != self.node_id:
            return

        if cmd_id == ODriveCmdID.HEARTBEAT and len(msg.data) >= 8:
            # Heartbeat: [active_errors(4)] [axis_state(1)] [procedure_result(1)] [traj_done(1)] [padding(1)]
            active_errors, axis_state, procedure_result, traj_done = struct.unpack('<IBBB', msg.data[:7])

            # Also decode disarm_reason from second 4 bytes
            disarm_reason = struct.unpack('<I', msg.data[4:8])[0]

            # Publish axis state
            state_msg = Int32()
            state_msg.data = axis_state
            self.pub_state.publish(state_msg)

            # Publish error code
            err_msg = Int32()
            err_msg.data = active_errors
            self.pub_error_code.publish(err_msg)

            # Publish disarm reason
            disarm_msg = Int32()
            disarm_msg.data = disarm_reason
            self.pub_disarm_reason.publish(disarm_msg)

            # Build error string and publish
            error_names = decode_flags(active_errors, ODRIVE_ERRORS)
            disarm_names = decode_flags(disarm_reason, DISARM_REASONS)
            error_str = String()
            error_str.data = f"errors:{','.join(error_names)}|disarm:{','.join(disarm_names)}|state:{AxisState.NAMES.get(axis_state, f'UNKNOWN({axis_state})')}"
            self.pub_errors.publish(error_str)

            # Log on change
            if active_errors != self._last_error:
                if active_errors != 0:
                    self.get_logger().warn(
                        f'ODrive errors: {", ".join(error_names)} (0x{active_errors:08X})'
                    )
                else:
                    self.get_logger().info('ODrive errors cleared')
                self._last_error = active_errors

            if disarm_reason != self._last_disarm:
                if disarm_reason != 0:
                    self.get_logger().warn(
                        f'ODrive disarm: {", ".join(disarm_names)} (0x{disarm_reason:08X})'
                    )
                self._last_disarm = disarm_reason

        elif cmd_id == ODriveCmdID.GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
            pos, vel = struct.unpack('<ff', msg.data)
            pos_msg = Float32()
            pos_msg.data = pos
            self.pub_position.publish(pos_msg)
            vel_msg = Float32()
            vel_msg.data = vel
            self.pub_velocity.publish(vel_msg)

        elif cmd_id == ODriveCmdID.GET_VBUS_VOLTAGE and len(msg.data) >= 4:
            vbus = struct.unpack('<f', msg.data[:4])[0]
            vbus_msg = Float32()
            vbus_msg.data = vbus
            self.pub_vbus.publish(vbus_msg)

    def destroy_node(self):
        if self.connected and self.bus:
            self.get_logger().info('Stopping motor...')
            data = struct.pack('<ff', 0.0, 0.0)
            self._send_can(ODriveCmdID.SET_INPUT_VEL, data)
            data = struct.pack('<I', AxisState.IDLE) + b'\x00\x00\x00\x00'
            self._send_can(ODriveCmdID.SET_AXIS_STATE, data)
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ODriveCANNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
