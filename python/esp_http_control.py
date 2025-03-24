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
        self.session = requests.Session()

        # Subscribe to velocity commands
        self.velocity_topic = "/cmd_vel"
        self.vel_subscriber = self.create_subscription(
            Twist,
            self.velocity_topic,
            self.cmd_vel_callback,
            10
        )
        

        # Create odometry publisher and TF broadcaster
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot-specific parameters
        self.wheel_offset_x = 0.174  # Half of the distance between the wheels
        self.wheel_radius = 0.037  # Wheel radius
        self.TICKS_PER_REV_LEFT = 23  # Encoder ticks per revolution (left wheel)
        self.TICKS_PER_REV_RIGHT = 23  # Encoder ticks per revolution (right wheel)

        self.position_history = deque(maxlen=5)
        self.last_command_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # Timeout for velocity commands

        # Timer to periodically update odometry
        self.create_timer(0.05, self.update_odom)

        # Store the last sent command to avoid redundant messages
        self.last_sent_linear_x = 0.0
        self.last_sent_angular_z = None

        self.reset_odometry()

    def reset_odometry(self):
        """Resets odometry to (0,0,0) and stores the current encoder values."""
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

        self.get_logger().info("Odometry reset to (0,0,0).")

        self.publish_odometry()
        self.send_tf_transform()

    def fetch_encoder_data(self):
        """Fetches encoder data from ESP, but at a reduced frequency."""
        time.sleep(0.1)
        try:
            command = json.dumps({"T": 1001})
            url = f"http://{ESP_IP}/js?json={quote(command, safe='{}:,')}"
            response = requests.get(url, timeout=1.0)

            if response.status_code == 200:
                data = response.json()
                if data is None or 'odr' not in data or 'odl' not in data:
                    self.get_logger().error("Received empty or incomplete encoder data!")
                    return None, None

                time.sleep(0.1)
                return data.get('odr', 0), data.get('odl', 0)
            else:
                self.get_logger().warn(f"ESP command failed, Status: {response.status_code}")
                return None, None
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Communication error with ESP: {e}")
            return None, None

    def cmd_vel_callback(self, msg: Twist):
        """Processes velocity commands and sends them only if they change."""
        max_linear_speed = 0.5
        max_angular_speed = 1.9

        self.linear_x = msg.linear.x 
        self.angular_z = msg.angular.z 

        # self.linear_x = max(min(msg.linear.x / max_linear_speed, 1.0), -1.0)
        # self.angular_x = max(min(msg.angular.x / max_angular_speed, 1.0), -1.0)

        threading.Thread(target=self.send_motor_command, args=(self.linear_x, self.angular_z), daemon=True).start()
        self.get_logger().info(f"RECEIVED: linear={self.linear_x}, angular={self.angular_z}")

        self.last_command_time = self.get_clock().now()

        if not hasattr(self, "send_command_timer"):
            self.send_command_timer = self.create_timer(0.1, self.send_continuous_command)

    def send_continuous_command(self):
        """Continuously sends motor commands if within the timeout period."""
        if (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9 < 0.5:
            self.send_motor_command(self.linear_x, self.angular_z)

    def send_motor_command(self, linear_x, angular_z):
        """Sends commands to the ESP module, but only on changes."""
        scaling_factor_circle = 0.18
        scaling_factor_straight = 0.1
        linear_x_scaled = linear_x  # * scaling_factor_straight
        angular_z_scaled = angular_z * scaling_factor_circle

        left_motor_speed = (linear_x_scaled - angular_z_scaled) 
        right_motor_speed = (linear_x_scaled + angular_z_scaled) 

        command = {"T": 1, "L": left_motor_speed, "R": right_motor_speed}
        json_command = json.dumps(command)
        encoded_command = quote(json_command)
        url = f"http://{ESP_IP}/js?json={encoded_command}"

        try:
            response = requests.get(url, timeout=2)
            time.sleep(0.2)

            if response.status_code == 200:
                self.get_logger().info(f"ESP confirmed command: {command}")
            else:
                self.get_logger().warn(f"ESP command failed, Status: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Communication error with ESP: {e}")

    def update_odom(self):
        """Calculates the position based on encoder data, but less frequently."""
        current_time = self.get_clock().now()
        
        if not hasattr(self, "last_odom_time"):
            self.last_odom_time = current_time
            return

        dt = (current_time - self.last_odom_time).nanoseconds / 1e9

        if dt == 0.0:
            return
        self.last_odom_time = current_time

        new_encoder_left, new_encoder_right = self.fetch_encoder_data()

        if new_encoder_left is None or new_encoder_right is None:
            return  

        self.get_logger().info(f"Links = {new_encoder_left}, Rechts = {new_encoder_right}")

        left_distance = (new_encoder_left - self.last_encoder_left) / self.TICKS_PER_REV_LEFT * (
                    2 * pi * self.wheel_radius)
        right_distance = (new_encoder_right - self.last_encoder_right) / self.TICKS_PER_REV_RIGHT * (
                    2 * pi * self.wheel_radius)

        avg_distance = (left_distance + right_distance) / 2.0

        delta_theta = (left_distance - right_distance) / (2 * self.wheel_offset_x)

        if abs(delta_theta) < 0.01:
            delta_theta = 0.0

        self.theta = (self.theta + delta_theta + pi) % (2 * pi) - pi

        if abs(delta_theta) > 0.01:
            self.x += avg_distance * cos(self.theta)
            self.y += avg_distance * sin(self.theta)
        else:
            self.x += avg_distance

        self.publish_odometry()
        self.send_tf_transform()

        self.last_encoder_left = new_encoder_left
        self.last_encoder_right = new_encoder_right

    def publish_odometry(self):
        """Publishes the current odometry."""
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
        """Sends the TF transformation base_link → odom."""
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
    """Main function to start the ROS2 node."""
    rclpy.init()
    node = ESPHttpControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
