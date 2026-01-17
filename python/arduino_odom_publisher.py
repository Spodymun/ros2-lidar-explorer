#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32MultiArray
import serial
from math import sin, cos, pi
import sys

class ArduinoOdomPublisher(Node):
    def __init__(self, serial_port=None):
        super().__init__('arduino_odom_publisher')
        
        # Parameter für serielle Verbindung
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('read_interval', 0.033)  # ~30Hz wie Arduino PID
        
        # Use command line argument if provided
        if serial_port:
            self.get_logger().info(f"Using serial port from command line: {serial_port}")
            final_serial_port = serial_port
        else:
            final_serial_port = self.get_parameter('serial_port').value
        
        baud_rate = self.get_parameter('baud_rate').value
        read_interval = self.get_parameter('read_interval').value
        
        # Serielle Verbindung zum Arduino
        try:
            self.serial_conn = serial.Serial(final_serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {final_serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {final_serial_port}: {e}')
            return
        
        # Publisher und TF Broadcaster
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.motor_data_publisher = self.create_publisher(Float32MultiArray, '/motor_data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Roboter-Parameter (anpassen an deine Konfiguration)
        self.wheel_radius = 0.032
        self.wheel_offset_x = 0.174
        self.TICKS_PER_REV = 23
        self.MAX_PWM = 255
        
        # Zustandsvariablen
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        self.first_reading = True
        
        # Motor-Zustandsvariablen (Live Daten vom Arduino)
        self.left_pwm = 0
        self.right_pwm = 0
        self.left_velocity = 0.0  # m/s berechnet aus PWM
        self.right_velocity = 0.0  # m/s berechnet aus PWM
        
        self.get_logger().info("Using LIVE MOTOR DATA mode (PWM + Encoder)")
        
        # Timer für regelmäßiges Lesen
        self.create_timer(read_interval, self.read_and_publish)
    
    def read_and_publish(self):
        """Liest Live Motor-Daten vom Arduino und publiziert Odometrie IMMER"""
        timestamp = self.get_clock().now().to_msg()
        motor1_pwm = 0
        motor2_pwm = 0
        motor3_pwm = 0
        motor4_pwm = 0
        encoder_left = self.last_encoder_left
        encoder_right = self.last_encoder_right
        
        try:
            # Sende READ_MOTOR_DATA Befehl an Arduino
            # Befehl 'f' sendet: enc1,enc2,enc3,enc4,pwm1,pwm2,pwm3,pwm4
            self.serial_conn.write(b'f\r')
            self.serial_conn.flush()
            
            # Versuche Antwort zu lesen (mit kurz Timeout)
            line = None
            for _ in range(5):  # Versuche bis zu 5x zu lesen
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        break
                    
            if line:  # Wenn wir eine Antwort bekamen
                try:
                    parts = line.split(',')
                    if len(parts) >= 8:
                        encoder_1 = int(parts[0])  # Motor 1 (left front)
                        encoder_2 = int(parts[1])  # Motor 2 (left rear)
                        encoder_3 = int(parts[2])  # Motor 3 (right front)
                        encoder_4 = int(parts[3])  # Motor 4 (right rear)
                        motor1_pwm = int(parts[4])  # Left front motor PWM
                        motor2_pwm = int(parts[5])  # Left rear motor PWM
                        motor3_pwm = int(parts[6])  # Right front motor PWM
                        motor4_pwm = int(parts[7])  # Right rear motor PWM
                        
                        # Combine encoders: left side (1+2)/2, right side (3+4)/2
                        encoder_left = (encoder_1 + encoder_2) // 2
                        encoder_right = (encoder_3 + encoder_4) // 2
                        
                        # Average PWM for each side
                        self.left_pwm = (motor1_pwm + motor2_pwm) // 2 if (motor1_pwm + motor2_pwm) != 0 else motor1_pwm
                        self.right_pwm = (motor3_pwm + motor4_pwm) // 2 if (motor3_pwm + motor4_pwm) != 0 else motor3_pwm
                        
                        if self.first_reading:
                            self.last_encoder_left = encoder_left
                            self.last_encoder_right = encoder_right
                            self.first_reading = False
                        else:
                            # Berechne Distanzen aus Encoder-Daten
                            delta_left = (encoder_left - self.last_encoder_left) / self.TICKS_PER_REV * (2 * 3.14159265 * self.wheel_radius)
                            delta_right = (encoder_right - self.last_encoder_right) / self.TICKS_PER_REV * (2 * 3.14159265 * self.wheel_radius)
                            
                            # Speichere aktuelle Encoder-Werte für nächsten Durchlauf
                            self.last_encoder_left = encoder_left
                            self.last_encoder_right = encoder_right
                            
                            # Berechne Position über Encoder-Deltas
                            avg_distance = (delta_left + delta_right) / 2.0
                            delta_theta = (delta_left - delta_right) / (2 * self.wheel_offset_x)
                            
                            # Update Position mit Differential Drive Kinematik
                            from math import sin, cos, pi
                            if abs(delta_theta) > 0.01:
                                self.x += avg_distance * cos(self.theta + delta_theta / 2)
                                self.y += avg_distance * sin(self.theta + delta_theta / 2)
                                self.theta = (self.theta + delta_theta + pi) % (2 * pi) - pi
                            else:
                                self.x += avg_distance * cos(self.theta)
                                self.y += avg_distance * sin(self.theta)
                    else:
                        self.get_logger().debug(f'Invalid motor data format: {line}')
                except (ValueError, IndexError) as e:
                    self.get_logger().debug(f'Error parsing motor data: {line}')
            else:
                # Kein Arduino-Response, aber weiterhin publizieren
                self.get_logger().debug('No response from Arduino, publishing last known state')
        
        except serial.SerialException as e:
            self.get_logger().warn(f'Error reading from Arduino: {e}')
        except Exception as e:
            self.get_logger().warn(f'Unexpected error: {e}')
        
        # ========== WICHTIG: IMMER Odometrie publizieren, auch wenn keine Bewegung ==========
        # PWM liegt zwischen -255 und +255
        max_velocity = 0.5  # m/s
        self.left_velocity = (self.left_pwm / self.MAX_PWM) * max_velocity if self.MAX_PWM != 0 else 0.0
        self.right_velocity = (self.right_pwm / self.MAX_PWM) * max_velocity if self.MAX_PWM != 0 else 0.0
        
        # Publiziere IMMER - das ist der Schlüssel!
        self.publish_odometry(timestamp)
        self.publish_motor_data(timestamp, motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm)
        self.publish_tf(timestamp)
    
    def publish_odometry(self, timestamp):
        """Publiziert Odometrie mit Live Motor-Velocitäten"""
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position aus Encoder-Integration
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)
        
        # ========== WICHTIG: Velocitäten aus LIVE Motor PWM-Daten ==========
        avg_velocity = (self.left_velocity + self.right_velocity) / 2.0
        odom.twist.twist.linear.x = avg_velocity
        odom.twist.twist.angular.z = (self.left_velocity - self.right_velocity) / (2 * self.wheel_offset_x)
        
        self.odom_publisher.publish(odom)
    
    def publish_motor_data(self, timestamp, motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm):
        """Publiziert rohe Live Motor-Daten für externe Nutzung / Debugging"""
        msg = Float32MultiArray()
        msg.data = [
            float(motor1_pwm),         # [0] Motor 1 PWM (left front)
            float(motor2_pwm),         # [1] Motor 2 PWM (left rear)
            float(motor3_pwm),         # [2] Motor 3 PWM (right front)
            float(motor4_pwm),         # [3] Motor 4 PWM (right rear)
            float(self.left_velocity), # [4] Calculated left velocity (m/s)
            float(self.right_velocity),# [5] Calculated right velocity (m/s)
            self.x,                    # [6] Current X position
            self.y,                    # [7] Current Y position
            self.theta                 # [8] Current orientation
        ]
        self.motor_data_publisher.publish(msg)
    
    def publish_tf(self, timestamp):
        """Publiziert TF Transform"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = sin(self.theta / 2)
        t.transform.rotation.w = cos(self.theta / 2)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    # Get serial port from command line if provided
    serial_port = sys.argv[1] if len(sys.argv) > 1 else None
    node = ArduinoOdomPublisher(serial_port=serial_port)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
