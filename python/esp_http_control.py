import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
import json
import requests
from math import cos, sin, pi
from urllib.parse import quote
import sys
import threading

ESP_IP = sys.argv[1]

class ESPHttpControl(Node):
    def __init__(self):
        super().__init__('esp_http_control')
        self.session = requests.Session()

        # Subscribe to the velocity command topic to control the robot
        self.velocity_topic = "/cmd_vel"
        self.vel_subscriber = self.create_subscription(
            Twist,
            self.velocity_topic,
            self.cmd_vel_callback,
            10
        )

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Wheel parameters
        self.wheel_offset_x = 0.174     #Don't measure these; just experiment until it matches your robot in RVIZ
        self.wheel_radius = 0.032       #Don't measure these; just experiment until it matches your robot in RVIZ
        self.TICKS_PER_REV_LEFT = 23
        self.TICKS_PER_REV_RIGHT = 23

        self.last_command_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        # Robot's current position and velocity
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Encoder data (initially fetched from the ESP)
        self.encoder_left, self.encoder_right = self.fetch_encoder_data()
        if self.encoder_left is None or self.encoder_right is None:
            self.encoder_left, self.encoder_right = 0, 0

        self.last_encoder_left = self.encoder_left
        self.last_encoder_right = self.encoder_right

        # Timers for updating odometry and publishing frequent odometry
        self.create_timer(0.1, self.update_odom)
        self.create_timer(0.08, self.publish_frequent_odometry)

    def fetch_encoder_data(self):
        """Fetch encoder data from the ESP via HTTP."""
        try:
            command = json.dumps({"T": 1001})
            url = f"http://{ESP_IP}/js?json={quote(command, safe='{}:,')}"
            response = requests.get(url, timeout=0.5)

            if response.status_code == 200:
                data = response.json()
                if data is None or 'odr' not in data or 'odl' not in data:
                    self.get_logger().error("Incomplete encoder data!")
                    return None, None
                return data.get('odr', 0), data.get('odl', 0)
            else:
                self.get_logger().warn(f"ESP response: {response.status_code}")
                return None, None
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"ESP communication error: {e}")
            return None, None

    def update_odom(self):
        """Update the robot's odometry based on the encoder values."""
        odom_time = self.get_clock().now()

        new_encoder_left, new_encoder_right = self.fetch_encoder_data()
        if new_encoder_left is None or new_encoder_right is None:
            return

        # Calculate distances based on encoder values
        left_distance = (new_encoder_left - self.last_encoder_left) / self.TICKS_PER_REV_LEFT * (2 * pi * self.wheel_radius)
        right_distance = (new_encoder_right - self.last_encoder_right) / self.TICKS_PER_REV_RIGHT * (2 * pi * self.wheel_radius)
        avg_distance = (left_distance + right_distance) / 2.0
        delta_theta = (left_distance - right_distance) / (2 * self.wheel_offset_x)

        if abs(delta_theta) < 0.01:
            delta_theta = 0.0

        # Update robot's orientation and position
        self.theta = (self.theta + delta_theta + pi) % (2 * pi) - pi
        if abs(delta_theta) > 0.01:
            self.x += avg_distance * cos(self.theta)
            self.y += avg_distance * sin(self.theta)
        else:
            self.x += avg_distance

        # Save current encoder values for the next update
        self.last_encoder_left = new_encoder_left
        self.last_encoder_right = new_encoder_right

        self.last_odom_time = odom_time

    def publish_frequent_odometry(self):
        """Publish odometry and send transform at a high frequency."""
        timestamp = self.get_clock().now().to_msg()
        self.publish_odometry(timestamp)
        self.send_tf_transform(timestamp)

    def publish_odometry(self, timestamp):
        """Publish the current odometry information."""
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)
        self.odom_publisher.publish(odom)

    def send_tf_transform(self, timestamp):
        """Send the robot's transform (position and orientation) to TF."""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = sin(self.theta / 2)
        t.transform.rotation.w = cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg: Twist):
        """Callback to receive velocity commands and process them."""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        if abs(self.linear_x) > 0.017:
            self.angular_z = 0.0
        else:
            self.linear_x = 0.0 

       # Adjust small velocity commands to ensure movement
        if 0 < abs(self.linear_x) < 0.07:
            self.linear_x = 0.07 if self.linear_x > 0 else -0.07

        if abs(self.linear_x) > 0.3:
            self.linear_x = 0.3 if self.linear_x > 0 else -0.3

        if 0 < abs(self.angular_z) < 0.2:
            self.angular_z = 0.2 if self.angular_z > 0 else -0.2

        if abs(self.angular_z) > 0.8:
            self.angular_z = 0.8 if self.angular_z > 0 else -0.

        threading.Thread(target=self.send_motor_command, args=(self.linear_x, self.angular_z), daemon=True).start()
        self.last_command_time = self.get_clock().now()

        if not hasattr(self, "send_command_timer"):
            self.send_command_timer = self.create_timer(0.1, self.send_continuous_command)

    def send_continuous_command(self):
        """Send motor commands periodically while the robot is active."""
        if (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9 < 0.5:
            self.send_motor_command(self.linear_x, self.angular_z)

    def send_motor_command(self, linear_x, angular_z):
        """Send motor control commands to the ESP."""
        scaling_factor_circle = 0.3
        linear_x_scaled = linear_x
        angular_z_scaled = angular_z * scaling_factor_circle

        left_motor_speed = (linear_x_scaled - angular_z_scaled)
        right_motor_speed = (linear_x_scaled + angular_z_scaled)

        command = {"T": 1, "L": left_motor_speed, "R": right_motor_speed}
        url = f"http://{ESP_IP}/js?json={quote(json.dumps(command))}"

        try:
            response = requests.get(url, timeout=0.5)
            if response.status_code != 200:
                self.get_logger().warn(f"ESP command failed: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Motor command error: {e}")

def main():
    """Main function to initialize the node and start ROS 2 spinning."""
    rclpy.init()
    node = ESPHttpControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
