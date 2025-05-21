import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_for_seconds()

    def move_for_seconds(self):
        """Moves the robot for a specified duration in seconds."""
        twist = Twist()
        twist.linear.x = 0.1  # Move forward at a constant speed
        #twist.angular.z = 0.43 # Uncomment this for circular motion

        # Duration of movement
        duration_active = 10

        self.get_logger().info(f"🚀 Moving the robot for {duration_active} seconds...")

        start_time = time.time()
        while time.time() - start_time < duration_active:
            self.publisher.publish(twist)
            time.sleep(0.1)  # Wait briefly before publishing again

        # Stop the robot after the duration
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("🛑 Robot stopped!")

def main():
    """Main function to initialize the ROS node and run the robot movement."""
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()