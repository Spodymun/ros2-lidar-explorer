#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32MultiArray
import serial
from math import sin, cos, pi, atan2
import sys

class MotorPublisher(Node):
    def __init__(self, serial_port=None):
        super().__init__('motor_publisher')
        
        # Serielle Verbindung
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        
        if serial_port:
            final_serial_port = serial_port
        else:
            final_serial_port = self.get_parameter('serial_port').value
        
        baud_rate = self.get_parameter('baud_rate').value
        
        try:
            self.serial_conn = serial.Serial(final_serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {final_serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.motor_data_pub = self.create_publisher(Float32MultiArray, '/motor_data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot parameters (KALIBRIERUNG)
        self.wheel_radius = 0.048          # Meter (9,6cm Durchmesser = 4,8cm Radius)
        self.wheel_separation = 0.23       # Meter (Abstand links-rechts)
        self.wheel_offset_x = 0.174        # Meter (Abstand vorne-hinten)
        self.TICKS_PER_REV = 1600          # ← KALIBRIERT: 1600 ticks pro Umdrehung (Motor 1 gemessen)
        self.MAX_PWM = 255
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        self.first_reading = True
        
        # Wheel angles for joint_states
        self.wheel_angles = {
            'left_front_wheel_joint': 0.0,
            'left_rear_wheel_joint': 0.0,
            'right_front_wheel_joint': 0.0,
            'right_rear_wheel_joint': 0.0
        }
        
        # Timer
        self.create_timer(0.033, self.read_and_publish)  # ~30Hz
        
        self.get_logger().info("Motor Publisher started - single source of truth!")
    
    def read_and_publish(self):
        """Read Arduino encoder data once, publish both /odom and /joint_states"""
        timestamp = self.get_clock().now().to_msg()
        
        motor1_pwm = 0
        motor2_pwm = 0
        motor3_pwm = 0
        motor4_pwm = 0
        encoder_left = self.last_encoder_left
        encoder_right = self.last_encoder_right
        
        try:
            # Read from Arduino
            self.serial_conn.write(b'f\r')
            self.serial_conn.flush()
            
            line = None
            for _ in range(5):
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        break
            
            if line:
                try:
                    parts = line.split(',')
                    if len(parts) >= 8:
                        encoder_1 = int(parts[0])
                        encoder_2 = int(parts[1])
                        encoder_3 = int(parts[2])
                        encoder_4 = int(parts[3])
                        motor1_pwm = int(parts[4])
                        motor2_pwm = int(parts[5])
                        motor3_pwm = int(parts[6])
                        motor4_pwm = int(parts[7])
                        
                        # Average encoders
                        # NOTE: Right side motors are mirrored, so negate their encoder values
                        encoder_left = (encoder_1 + encoder_2) // 2
                        encoder_right = -(encoder_3 + encoder_4) // 2  # Negated for mirrored motor config
                        
                        if self.first_reading:
                            self.last_encoder_left = encoder_left
                            self.last_encoder_right = encoder_right
                            self.first_reading = False
                        else:
                            # === SINGLE CALCULATION POINT ===
                            # Convert encoder ticks to distance
                            delta_left = (encoder_left - self.last_encoder_left) / self.TICKS_PER_REV * (2 * pi * self.wheel_radius)
                            delta_right = (encoder_right - self.last_encoder_right) / self.TICKS_PER_REV * (2 * pi * self.wheel_radius)
                            
                            # Update position using differential drive kinematics
                            avg_distance = (delta_left + delta_right) / 2.0
                            delta_theta = (delta_left - delta_right) / self.wheel_separation
                            
                            if abs(delta_theta) > 0.01:
                                self.x += avg_distance * cos(self.theta + delta_theta / 2)
                                self.y += avg_distance * sin(self.theta + delta_theta / 2)
                            else:
                                self.x += avg_distance * cos(self.theta)
                                self.y += avg_distance * sin(self.theta)
                            
                            self.theta = (self.theta + delta_theta + pi) % (2 * pi) - pi
                            
                            # Update wheel angles based on actual wheel distances (for joint_states)
                            # Use delta_left and delta_right directly for accurate wheel rotation
                            angle_delta_left = delta_left / self.wheel_radius
                            angle_delta_right = delta_right / self.wheel_radius  # Already negated from encoder reading
                            
                            # Update left wheels
                            for key in ['left_front_wheel_joint', 'left_rear_wheel_joint']:
                                self.wheel_angles[key] += angle_delta_left
                                self.wheel_angles[key] = atan2(
                                    sin(self.wheel_angles[key]),
                                    cos(self.wheel_angles[key])
                                )
                            
                            # Update right wheels (negated because of mirrored motor config)
                            for key in ['right_front_wheel_joint', 'right_rear_wheel_joint']:
                                self.wheel_angles[key] -= angle_delta_right
                                self.wheel_angles[key] = atan2(
                                    sin(self.wheel_angles[key]),
                                    cos(self.wheel_angles[key])
                                )
                            
                            self.last_encoder_left = encoder_left
                            self.last_encoder_right = encoder_right
                    else:
                        self.get_logger().debug(f'Invalid format: {line}')
                except (ValueError, IndexError) as e:
                    self.get_logger().debug(f'Parse error: {e}')
        except Exception as e:
            self.get_logger().debug(f'Serial error: {e}')
        
        # Velocities from PWM
        left_velocity = (((motor1_pwm + motor2_pwm) // 2) / self.MAX_PWM) * 0.5
        right_velocity = (((motor3_pwm + motor4_pwm) // 2) / self.MAX_PWM) * 0.5
        
        # === PUBLISH BOTH FROM SAME DATA ===
        self.publish_odom(timestamp, left_velocity, right_velocity)
        self.publish_joint_states(timestamp, left_velocity, right_velocity)
        self.publish_motor_data(timestamp, motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm)
        self.publish_tf(timestamp)
    
    def publish_odom(self, timestamp, vel_left, vel_right):
        """Publish odometry"""
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)
        
        avg_velocity = (vel_left + vel_right) / 2.0
        odom.twist.twist.linear.x = avg_velocity
        odom.twist.twist.angular.z = (vel_right - vel_left) / self.wheel_separation
        
        self.odom_pub.publish(odom)
    
    def publish_joint_states(self, timestamp, vel_left, vel_right):
        """Publish wheel joint states"""
        joint_state = JointState()
        joint_state.header.stamp = timestamp
        joint_state.name = list(self.wheel_angles.keys())
        joint_state.position = list(self.wheel_angles.values())
        joint_state.velocity = [vel_left, vel_left, vel_right, vel_right]
        
        self.joint_pub.publish(joint_state)
    
    def publish_motor_data(self, timestamp, pwm1, pwm2, pwm3, pwm4):
        """Publish motor data for debugging"""
        msg = Float32MultiArray()
        msg.data = [float(pwm1), float(pwm2), float(pwm3), float(pwm4),
                    self.x, self.y, self.theta]
        self.motor_data_pub.publish(msg)
    
    def publish_tf(self, timestamp):
        """Publish TF transform"""
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = sin(self.theta / 2)
        transform.transform.rotation.w = cos(self.theta / 2)
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    
    serial_port = None
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    
    node = MotorPublisher(serial_port=serial_port)
    
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
