import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_for_seconds()

    def move_for_seconds(self):
        """Lässt den Roboter genau `duration` Sekunden fahren."""
        twist = Twist()
        twist.linear.x = 0.1  # straight
        # twist.angular.z = 0.3 # circle

        # duration_notactive = (2 * math.pi) / twist.angular.z
        duration_active = 10

        self.get_logger().info(f"🚀 Roboter fährt für {duration_active} Sekunden...")

        start_time = time.time()
        while time.time() - start_time < duration_active:
            self.publisher.publish(twist)
            time.sleep(0.1)  

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("🛑 Roboter gestoppt!")

def main():
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
