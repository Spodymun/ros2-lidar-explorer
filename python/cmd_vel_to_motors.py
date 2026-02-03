#!/usr/bin/env python3

import math
import sys
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToMotors(Node):
    def __init__(self, serial_port='/dev/ttyACM1'):
        super().__init__('cmd_vel_to_motors')

        # --- Serial ---
        try:
            self.serial_conn = serial.Serial(serial_port, 57600, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial_conn = None

        # --- Subscribe to cmd_vel ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # --- Robot parameters (DEINE WERTE) ---
        self.wheel_separation = 0.505   # m
        self.wheel_radius = 0.072       # m
        self.ticks_per_rev = 1600       # ticks / wheel revolution (nach Getriebe gemessen)

        # Control update rate on Arduino PID loop
        self.control_rate_hz = 30.0
        self.dt = 1.0 / self.control_rate_hz

        # Optional safety limits (depends on your robot)
        self.max_linear = 1.0     # m/s (limit cmd_vel)
        self.max_angular = 2.0    # rad/s (limit cmd_vel)

        # Logging throttle
        self._log_counter = 0

    def cmd_vel_callback(self, msg: Twist):
        if self.serial_conn is None:
            return

        v_x = float(msg.linear.x)
        w_z = float(msg.angular.z)

        # --- Clamp inputs (optional but recommended) ---
        v_x = max(-self.max_linear, min(self.max_linear, v_x))
        w_z = max(-self.max_angular, min(self.max_angular, w_z))

        # --- Differential drive: wheel linear speeds (m/s) ---
        v_left = v_x - w_z * (self.wheel_separation / 2.0)
        v_right = v_x + w_z * (self.wheel_separation / 2.0)

        # --- Convert linear wheel speed to wheel angular speed (rad/s) ---
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # --- Convert to ticks per second ---
        ticks_left_per_sec = omega_left * self.ticks_per_rev / (2.0 * math.pi)
        ticks_right_per_sec = omega_right * self.ticks_per_rev / (2.0 * math.pi)

        # --- Convert to ticks per frame (ticks per 1/30s) ---
        ticks_left_frame = int(round(ticks_left_per_sec * self.dt))
        ticks_right_frame = int(round(ticks_right_per_sec * self.dt))

        # Send speed targets (closed-loop)
        command = f"s {ticks_left_frame} {ticks_right_frame}\r"

        try:
            self.serial_conn.write(command.encode('ascii', errors='ignore'))
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            return

        # Throttled debug logging (every ~10 messages)
        self._log_counter = (self._log_counter + 1) % 10
        if self._log_counter == 0:
            self.get_logger().info(
                f"cmd_vel: vx={v_x:.3f} wz={w_z:.3f} -> ticks/frame L={ticks_left_frame} R={ticks_right_frame}"
            )


def main(args=None):
    rclpy.init(args=args)

    serial_port = '/dev/ttyACM1'
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