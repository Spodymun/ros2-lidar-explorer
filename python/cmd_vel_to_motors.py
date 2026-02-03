#!/usr/bin/env python3
import math
import sys
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToMotors(Node):
    def __init__(self, serial_port='/dev/ttyACM0'):
        super().__init__('cmd_vel_to_motors')

        try:
            self.serial_conn = serial.Serial(serial_port, 57600, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial_conn = None

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Robot parameters
        self.wheel_separation = 0.505   # m
        self.wheel_radius = 0.072       # m
        self.ticks_per_rev = 1600       # ticks per wheel revolution (measured after gearbox)

        # Arduino PID rate
        self.control_rate_hz = 30.0
        self.dt = 1.0 / self.control_rate_hz

        # cmd_vel limits
        self.max_linear = 1.0     # m/s
        self.max_angular = 2.0    # rad/s

        # ticks/frame safety clamp (adjust if needed)
        self.max_ticks_frame = 2000

        self._log_counter = 0

    @staticmethod
    def _ensure_min_ticks(ticks_frame: int, wheel_v_mps: float) -> int:
        # Avoid rounding to zero for small non-zero velocities
        if abs(wheel_v_mps) > 1e-3 and ticks_frame == 0:
            return 1 if wheel_v_mps > 0 else -1
        return ticks_frame

    def cmd_vel_callback(self, msg: Twist):
        if self.serial_conn is None:
            return

        v_x = float(msg.linear.x)
        w_z = float(msg.angular.z)

        # Clamp cmd_vel
        v_x = max(-self.max_linear, min(self.max_linear, v_x))
        w_z = max(-self.max_angular, min(self.max_angular, w_z))

        # Differential drive wheel linear speeds (m/s)
        v_left = v_x - w_z * (self.wheel_separation / 2.0)
        v_right = v_x + w_z * (self.wheel_separation / 2.0)

        # Linear -> angular (rad/s)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # rad/s -> ticks/s
        ticks_left_per_sec = omega_left * self.ticks_per_rev / (2.0 * math.pi)
        ticks_right_per_sec = omega_right * self.ticks_per_rev / (2.0 * math.pi)

        # ticks/s -> ticks/frame
        ticks_left_frame = int(round(ticks_left_per_sec * self.dt))
        ticks_right_frame = int(round(ticks_right_per_sec * self.dt))

        # Prevent small values rounding to 0
        ticks_left_frame = self._ensure_min_ticks(ticks_left_frame, v_left)
        ticks_right_frame = self._ensure_min_ticks(ticks_right_frame, v_right)

        # Clamp
        ticks_left_frame = max(-self.max_ticks_frame, min(self.max_ticks_frame, ticks_left_frame))
        ticks_right_frame = max(-self.max_ticks_frame, min(self.max_ticks_frame, ticks_right_frame))

        # Send
        command = f"v {ticks_left_frame} {ticks_right_frame}\r\n"
        try:
            self.serial_conn.write(command.encode('ascii', errors='ignore'))
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            return

        self._log_counter = (self._log_counter + 1) % 10
        if self._log_counter == 0:
            self.get_logger().info(
                f"cmd_vel: vx={v_x:.3f} wz={w_z:.3f} -> ticks/frame L={ticks_left_frame} R={ticks_right_frame}"
            )


def main(args=None):
    rclpy.init(args=args)

    serial_port = '/dev/ttyACM0'
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    node = CmdVelToMotors(serial_port=serial_port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
