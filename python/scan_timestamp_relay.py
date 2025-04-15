#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.time import Duration

class ScanTimestampRelay(Node):
    def __init__(self):
        super().__init__('scan_timestamp_relay')

        # Subscription to the '/scan_raw' topic to receive LaserScan messages
        self.sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.callback,
            10
        )

        # Publisher to forward modified LaserScan messages to the '/scan' topic
        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

    def callback(self, msg):
        """Callback function to adjust the timestamp of the received LaserScan message."""
        # Adjust the timestamp by adding 0.1 seconds from the current time
        adjusted_time = self.get_clock().now() - Duration(seconds=0.05)
        msg.header.stamp = adjusted_time.to_msg()
        
        # Publish the modified LaserScan message
        self.pub.publish(msg)

def main(args=None):
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args)
    node = ScanTimestampRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()