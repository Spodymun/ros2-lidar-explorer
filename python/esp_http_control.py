import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
import json
import requests
from math import cos, sin
from urllib.parse import quote
import sys

# ESP32 URL
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

        # Position & Orientierung
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odl_initial = None  
        self.odr_initial = None  

        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()  # Prevents spam requests
        self.create_timer(0.05, self.update_odom)  # 20 Hz Update-Rate

        # Roboter-Spezifikationen
        self.wheel_radius = 0.033  # Radgröße in Metern
        self.wheel_base = 0.297  # Abstand zwischen den Rädern
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to left/right wheel speeds and send to ESP32."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_cmd_time).nanoseconds / 1e9

        if dt < 0.1:  # Prevent excessive requests (limit to 10Hz)
            return

        self.last_cmd_time = current_time  # Update last sent time
        
        try:
            v = msg.linear.x  # Forward velocity (m/s)
            w = msg.angular.z  # Angular velocity (rad/s)

            left_speed = v - (self.wheel_base / 2.0) * w
            right_speed = v + (self.wheel_base / 2.0) * w

            # Debugging logs
            self.get_logger().info(f"cmd_vel received: linear={v:.3f}, angular={w:.3f}")
            self.get_logger().info(f"Calculated wheel speeds: L={left_speed:.3f}, R={right_speed:.3f}")

            # Properly formatted JSON
            command = json.dumps({"T": 1, "L": left_speed, "R": right_speed})
            url = f"http://{ESP_IP}/js?json={quote(command, safe='{}:,')}"
            response = requests.get(url, timeout=1.0)

            # Debug response
            self.get_logger().info(f"Sent: {command}, ESP Response: {response.text}")
        
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error sending speed: {e}")

    def update_odom(self):
        """Holt Sensordaten vom ESP32 und berechnet die Odometrie."""
        try:
            command = json.dumps({"T": 1001})
            url = f"http://{ESP_IP}/js?json={quote(command, safe='{}:,')}"
            response = requests.get(url, timeout=1.0)
            content = response.text

            self.get_logger().info(f"ESP32 Response: {content}")

            data = json.loads(content)
            if data is None:
                self.get_logger().error("Fehler: ESP32 hat 'null' gesendet.")
                return

            if not all(k in data for k in ["odl", "odr", "gz"]):
                self.get_logger().error(f"Fehlende Daten in der ESP32-Antwort: {data}")
                return

            odl, odr, gz = data["odl"], data["odr"], data["gz"]

            if self.odl_initial is None or self.odr_initial is None:
                self.odl_initial = odl
                self.odr_initial = odr
                self.get_logger().info(f"Initiale Encoder-Werte gesetzt: odl={odl}, odr={odr}")

            odl -= self.odl_initial
            odr -= self.odr_initial

            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            left_distance = (odl / 1000) * self.wheel_radius
            right_distance = (odr / 1000) * self.wheel_radius

            linear_distance = (left_distance + right_distance) / 2
            angular_change = (right_distance - left_distance) / self.wheel_base

            self.x += linear_distance * cos(self.theta)
            self.y += linear_distance * sin(self.theta)
            self.theta += angular_change

            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.z = sin(self.theta / 2)
            odom.pose.pose.orientation.w = cos(self.theta / 2)
            self.odom_publisher.publish(odom)

            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = sin(self.theta / 2)
            t.transform.rotation.w = cos(self.theta / 2)
            self.tf_broadcaster.sendTransform(t)
        
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error getting odometry data: {e}")


def main():
    rclpy.init()
    node = ESPHttpControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()