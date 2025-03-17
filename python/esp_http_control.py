import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
import json
import requests
from math import cos, sin, pi
from urllib.parse import quote
from collections import deque
import sys
import numpy as np
import threading
import time  

ESP_IP = sys.argv[1]

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

        self.wheel_offset_x = 0.15  
        self.wheel_radius = 0.033  
        self.TICKS_PER_REV = 100  

        self.position_history = deque(maxlen=5)
        self.last_command_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        # **Neu: Timer für Encoder-Updates auf 100 ms**
        self.create_timer(0.1, self.update_odom)

        # **Neu: Speichert den letzten gesendeten Befehl, um doppelte Anfragen zu vermeiden**
        self.last_sent_linear_x = None
        self.last_sent_angular_z = None

        self.reset_odometry()  

    def reset_odometry(self):
        """Setzt die Odometrie auf (0,0,0) zurück und speichert die aktuellen Encoder-Werte."""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.encoder_left, self.encoder_right = self.fetch_encoder_data()
        if self.encoder_left is None or self.encoder_right is None:
            self.encoder_left, self.encoder_right = 0, 0

        self.last_encoder_left = self.encoder_left
        self.last_encoder_right = self.encoder_right

        self.get_logger().info("🔄 Odometrie auf (0,0,0) zurückgesetzt.")

        self.publish_odometry()
        self.send_tf_transform()

    def fetch_encoder_data(self):
        """Holt Encoder-Daten vom ESP, aber mit reduzierter Frequenz."""
        try:
            command = json.dumps({"T": 1001})
            url = f"http://{ESP_IP}/js?json={quote(command, safe='{}:,')}"
            response = requests.get(url, timeout=1.0)

            if response.status_code == 200:
                data = response.json()
                if data is None or 'odr' not in data or 'odl' not in data:
                    self.get_logger().error("⚠️ Leere oder unvollständige Encoder-Daten empfangen!")
                    return None, None
                return data.get('odr', 0), data.get('odl', 0)
            else:
                self.get_logger().warn(f"⚠️ ESP-Befehl fehlgeschlagen, Status: {response.status_code}")
                return None, None
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"⚠️ Kommunikationsfehler mit ESP: {e}")
            return None, None

    def cmd_vel_callback(self, msg):
        """Verarbeitet Steuerbefehle und sendet sie nur, wenn sie sich ändern."""
        self.linear_x = msg.linear.x * 7
        self.angular_z = msg.angular.z * 7

        # **Nur senden, wenn sich der Befehl geändert hat**
        if (self.linear_x, self.angular_z) != (self.last_sent_linear_x, self.last_sent_angular_z):
            self.last_sent_linear_x = self.linear_x
            self.last_sent_angular_z = self.angular_z

            threading.Thread(target=self.send_motor_command, args=(self.linear_x, self.angular_z), daemon=True).start()
            self.get_logger().info(f"🟢 ERHALTEN: linear={self.linear_x}, angular={self.angular_z}")

        self.last_command_time = self.get_clock().now()

    def send_motor_command(self, linear_x, angular_z):
        """Sendet Befehle an das ESP-Modul, aber nur bei Änderungen."""
        scaling_factor_circle = 0.0627
        scaling_factor_straight = 0.09
        linear_x_scaled = linear_x * scaling_factor_straight
        angular_z_scaled = angular_z * scaling_factor_circle

        left_motor_speed = linear_x_scaled - angular_z_scaled
        right_motor_speed = linear_x_scaled + angular_z_scaled

        command = {"T": 1, "L": left_motor_speed, "R": right_motor_speed}
        json_command = json.dumps(command)
        encoded_command = quote(json_command)
        url = f"http://{ESP_IP}/js?json={encoded_command}"

        try:
            response = requests.get(url, timeout=0.5)
            if response.status_code == 200:
                self.get_logger().info(f"✅ ESP bestätigt Befehl: {command}")
            else:
                self.get_logger().warn(f"⚠️ ESP-Befehl fehlgeschlagen, Status: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"❌ Kommunikationsfehler mit ESP: {e}")

    def update_odom(self):
        """Berechnet die Position basierend auf Encoder-Daten, aber weniger oft."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_command_time).nanoseconds / 1e9

        if dt > 0.1:  # **Erhöhtes Intervall für weniger Requests**
            dt = 0.1

        new_encoder_left, new_encoder_right = self.fetch_encoder_data()

        if new_encoder_left is None or new_encoder_right is None:
            return  # Falls keine neuen Werte vorliegen, keine Aktualisierung

        left_distance = (new_encoder_left - self.last_encoder_left) / self.TICKS_PER_REV * (2 * pi * self.wheel_radius)
        right_distance = (new_encoder_right - self.last_encoder_right) / self.TICKS_PER_REV * (2 * pi * self.wheel_radius)

        avg_distance = (left_distance + right_distance) / 2.0
        delta_theta = (left_distance - right_distance) / (2 * self.wheel_offset_x)

        self.x += avg_distance * cos(self.theta)
        self.y += avg_distance * sin(self.theta)
        self.theta = (self.theta + delta_theta + pi) % (2 * pi) - pi

        self.last_command_time = current_time

        self.publish_odometry()
        self.send_tf_transform()

        self.last_encoder_left = new_encoder_left
        self.last_encoder_right = new_encoder_right

    def publish_odometry(self):
        """Veröffentlicht die aktuelle Odometrie."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)

        self.odom_publisher.publish(odom)

    def send_tf_transform(self):
        """Sendet die TF-Transformation base_link → odom."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = sin(self.theta / 2)
        t.transform.rotation.w = cos(self.theta / 2)

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ESPHttpControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
