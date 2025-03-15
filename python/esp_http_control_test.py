import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
import json
import requests
from math import cos, sin
from urllib.parse import quote  # URL-Codierung
from collections import deque  # Für gleitenden Durchschnitt

# Beispiel-URL deines ESP
ESP_IP = "http://192.168.137.249"  # IP-Adresse des ESP

class ESPHttpControl(Node):
    def __init__(self):
        super().__init__('esp_http_control')
        
        self.velocity_topic = "/cmd_vel"
        self.vel_subscriber = self.create_subscription(
            Twist,
            self.velocity_topic,
            self.cmd_vel_callback,
            10
        )
        
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Anfangswerte für die Position und Orientierung
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Geschwindigkeiten (lineare und winkelige)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_time = self.get_clock().now()
        
        # Gleitender Durchschnitt der letzten 5 Positionen
        self.position_history = deque(maxlen=5)
        self.last_command_time = self.get_clock().now()
        
        # Timeout für die Reduzierung der Geschwindigkeit nach Inaktivität
        self.cmd_timeout = 0.5
        self.create_timer(0.02, self.update_odom)  # Update-Rate 50 Hz

        # Definition der Rad-Offsets (Position der Räder relativ zum Base-Link)
        self.wheel_offset_x = 0.226
        self.wheel_offset_y = 0.1485
        self.wheel_radius = 0.033
        self.wheel_thickness = 0.026

    def cmd_vel_callback(self, msg):
        """Callback, um Befehle zu verarbeiten und Geschwindigkeiten zu setzen."""
        self.linear_x = msg.linear.x * 5
        self.angular_z = msg.angular.z * 5
        self.last_command_time = self.get_clock().now()  # Zeitstempel aktualisieren

    def update_odom(self):
        """Aktualisiert die Odometrie und sendet Transformationsdaten."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # Wenn keine neuen Befehle kommen, Geschwindigkeit langsam reduzieren
        #time_since_last_cmd = (current_time - self.last_command_time).nanoseconds / 1e9
        #if time_since_last_cmd > self.cmd_timeout:
        #    self.linear_x *= 0.95  # Exponentielles Abbremsen
        #    self.angular_z *= 0.95
        #    if abs(self.linear_x) < 0.01:
        #        self.linear_x = 0.0
        #    if abs(self.angular_z) < 0.01:
        #        self.angular_z = 0.0

        # Berechnung der neuen Position des `base_link`
        new_x = self.x + self.linear_x * cos(self.theta) * dt
        new_y = self.y + self.linear_x * sin(self.theta) * dt
        new_theta = self.theta + self.angular_z * dt

        # Speichern der Positionen für den gleitenden Durchschnitt
        self.position_history.append((new_x, new_y, new_theta))
        avg_x = sum(p[0] for p in self.position_history) / len(self.position_history)
        avg_y = sum(p[1] for p in self.position_history) / len(self.position_history)
        avg_theta = sum(p[2] for p in self.position_history) / len(self.position_history)

        self.x, self.y, self.theta = avg_x, avg_y, avg_theta
        self.last_time = current_time

        # Odom-Nachricht: Aktualisierung der Position des base_link in der Odometrie
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)
        self.odom_publisher.publish(odom)

        # TF-Transformation: Transformation des base_link in das odom-Koordinatensystem
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = sin(self.theta / 2)
        t.transform.rotation.w = cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(t)

        # Jetzt die Radrotation berechnen (keine Änderung der Position der Räder)
        left_wheel_rotation = self.linear_x * dt / self.wheel_radius
        right_wheel_rotation = self.linear_x * dt / self.wheel_radius

        # Nur alle 100 ms an ESP senden
        if (current_time - self.last_command_time).nanoseconds / 1e9 >= 0.2:
            self.send_motor_command(self.linear_x, self.angular_z)
            self.last_command_time = current_time

    def send_motor_command(self, linear_x, angular_z):
        """Sendet Befehle an das ESP-Modul zur Steuerung des Motors."""
        scaling_factor_circel = 0.0627
        scaling_factor_straight = 0.09
        linear_x_scaled = linear_x * scaling_factor_straight
        angular_z_scaled = angular_z * scaling_factor_circel

        left_motor_speed = linear_x_scaled - angular_z_scaled
        right_motor_speed = linear_x_scaled + angular_z_scaled

        command = {"T": 1, "L": left_motor_speed, "R": right_motor_speed}
        json_command = json.dumps(command)
        encoded_command = quote(json_command)
        url = f"{ESP_IP}/js?json={encoded_command}"
        try:
            response = requests.get(url, timeout=0.5)
            if response.status_code == 200:
                self.get_logger().info(f"Command sent to ESP: {command}")
            else:
                self.get_logger().warn(f"Failed to send command to ESP, status code: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error communicating with ESP: {e}")


def main():
    rclpy.init()
    node = ESPHttpControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
