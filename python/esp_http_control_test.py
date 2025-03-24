import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
import json
import requests
import math
from urllib.parse import quote
from collections import deque
import sys
import threading
import time

ESP_IP = sys.argv[1]

# === Roboterparameter ===
WHEEL_RADIUS = 0.035        # Meter
WHEEL_OFFSET_X = 0.197     # halber Radabstand => voller Abstand = 0.394
TICKS_PER_WHEEL_REV = 23    # Encoder-Ticks pro Radumdrehung

# === Berechnung: Ticks für eine Roboterdrehung ===
robot_rotation_path_per_wheel = math.pi * (2 * WHEEL_OFFSET_X)
wheel_circumference = 2 * math.pi * WHEEL_RADIUS
wheel_revolutions_for_1_robot_turn = robot_rotation_path_per_wheel / wheel_circumference
ticks_for_1_robot_turn = int(TICKS_PER_WHEEL_REV * wheel_revolutions_for_1_robot_turn)

class ESPHttpControl(Node):
    def __init__(self):
        super().__init__('esp_http_control')
        self.session = requests.Session()

        self.velocity_topic = "/cmd_vel"
        self.vel_subscriber = self.create_subscription(
            Twist,
            self.velocity_topic,
            self.cmd_vel_callback,
            10
        )

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.wheel_offset_x = WHEEL_OFFSET_X
        self.wheel_radius = WHEEL_RADIUS
        self.TICKS_PER_REV_LEFT = TICKS_PER_WHEEL_REV
        self.TICKS_PER_REV_RIGHT = TICKS_PER_WHEEL_REV

        self.position_history = deque(maxlen=5)
        self.last_command_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        self.create_timer(0.1, self.update_odom)

        self.last_sent_linear_x = 0.0
        self.last_sent_angular_z = None

        self.reset_odometry()
        self.rotating = False

    def reset_odometry(self):
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
        time.sleep(0.1)
        try:
            command = json.dumps({"T": 1001})
            url = f"http://{ESP_IP}/js?json={quote(command, safe='{}:,')}"
            response = requests.get(url, timeout=1.0)
            if response.status_code == 200:
                data = response.json()
                if data is None or 'odr' not in data or 'odl' not in data:
                    self.get_logger().error("Received incomplete encoder data!")
                    return None, None
                time.sleep(0.2)
                return data.get('odr', 0), data.get('odl', 0)
            else:
                self.get_logger().warn(f"ESP command failed, Status: {response.status_code}")
                return None, None
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Communication error with ESP: {e}")
            return None, None

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x * -1
        self.angular_z = msg.angular.z

        if abs(self.angular_z) > 0 and abs(self.linear_x) < 1e-4:
            threading.Thread(target=self.rotate_exactly_once, args=(self.angular_z,), daemon=True).start()
        else:
            threading.Thread(target=self.send_motor_command, args=(self.linear_x, self.angular_z), daemon=True).start()

        self.get_logger().info(f"RECEIVED: linear={self.linear_x}, angular={self.angular_z}")
        self.last_command_time = self.get_clock().now()

        if not hasattr(self, "send_command_timer"):
            self.send_command_timer = self.create_timer(0.1, self.send_continuous_command)

    def send_continuous_command(self):
        time.sleep(0.2)
        if (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9 < 0.5:
            self.send_motor_command(self.linear_x, self.angular_z)

    def send_motor_command(self, linear_x, angular_z):
        scaling_factor_circle = 1.0  # Direktwert
        linear_x_scaled = linear_x
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

    def rotate_exactly_once(self, angular_speed):
        if self.rotating:
            return
        self.rotating = True

        start_l, start_r = self.fetch_encoder_data()
        if start_l is None or start_r is None:
            self.get_logger().error("Encoderwerte konnten nicht gelesen werden!")
            self.rotating = False
            return

        self.get_logger().info(f"Start Encoderwerte: L={start_l}, R={start_r}")
        while True:
            self.send_motor_command(-angular_speed, angular_speed)
            time.sleep(0.5)
            _, current_r = self.fetch_encoder_data()
            if current_r is None:
                continue

            delta_ticks = abs(current_r - start_r)
            self.get_logger().info(f"Ticks: {delta_ticks}/{ticks_for_1_robot_turn}")
            if delta_ticks >= ticks_for_1_robot_turn:
                self.get_logger().info("Ziel erreicht: Genau eine Drehung.")
                self.send_motor_command(0, 0)
                break

        self.rotating = False

    def update_odom(self):
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

        left_distance = (new_encoder_left - self.last_encoder_left) / self.TICKS_PER_REV_LEFT * (2 * math.pi * self.wheel_radius)
        right_distance = (new_encoder_right - self.last_encoder_right) / self.TICKS_PER_REV_RIGHT * (2 * math.pi * self.wheel_radius)

        avg_distance = (left_distance + right_distance) / 2.0
        delta_theta = (left_distance - right_distance) / (2 * self.wheel_offset_x)

        if abs(delta_theta) < 0.01:
            delta_theta = 0.0

        self.theta = (self.theta + delta_theta + math.pi) % (2 * math.pi) - math.pi

        if abs(delta_theta) > 0.01:
            self.x += avg_distance * math.cos(self.theta)
            self.y += avg_distance * math.sin(self.theta)
        else:
            self.x += avg_distance

        self.publish_odometry()
        self.send_tf_transform()

        self.last_encoder_left = new_encoder_left
        self.last_encoder_right = new_encoder_right

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        self.odom_publisher.publish(odom)

    def send_tf_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ESPHttpControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
