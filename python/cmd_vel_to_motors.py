#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelToMotors(Node):
    def __init__(self, serial_port='/dev/ttyACM0'):
        super().__init__('cmd_vel_to_motors')
        
        # Connect to Arduino
        try:
            self.serial_conn = serial.Serial(serial_port, 57600, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial_conn = None
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Robot parameters
        self.wheel_separation = 0.23  # Distance between left and right wheels (m)
        self.wheel_radius = 0.048     # Wheel radius (m) - kalibriert (9,6cm Durchmesser)
        self.max_speed = 1.0          # Max linear speed (m/s)
        self.max_angular = 2.0        # Max angular speed (rad/s)
        
    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to motor PWM commands"""
        if self.serial_conn is None:
            return
        
        # Extract linear and angular velocity
        v_x = msg.linear.x   # Linear velocity (m/s)
        w_z = msg.angular.z  # Angular velocity (rad/s)
        
        # Differential drive kinematics
        # v_left = v_x - w_z * (wheel_separation / 2)
        # v_right = v_x + w_z * (wheel_separation / 2)
        v_left = v_x - w_z * (self.wheel_separation / 2)
        v_right = v_x + w_z * (self.wheel_separation / 2)
        
        # Normalize to [-1, 1] range
        max_v = max(abs(v_left), abs(v_right), self.max_speed)
        if max_v > self.max_speed:
            v_left = (v_left / max_v) * self.max_speed
            v_right = (v_right / max_v) * self.max_speed
        
        # Convert velocity to PWM (-255 to 255)
        # Assuming max_speed corresponds to PWM = 255
        pwm_left = int((v_left / self.max_speed) * 255) if self.max_speed > 0 else 0
        pwm_right = int((v_right / self.max_speed) * 255) if self.max_speed > 0 else 0
        
        # Clamp to [-255, 255]
        pwm_left = max(-255, min(255, pwm_left))
        pwm_right = max(-255, min(255, pwm_right))
        
        # Send to Arduino
        # Format: "m <pwm_left> <pwm_right>\r"
        command = f"m {pwm_left} {pwm_right}\r"
        
        try:
            self.serial_conn.write(command.encode())
            self.get_logger().debug(f'Sent: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    # Get serial port from command line if provided
    serial_port = '/dev/ttyACM0'
    if len(__import__('sys').argv) > 1:
        serial_port = __import__('sys').argv[1]
    
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
