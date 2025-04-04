#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.time import Duration

class ScanTimestampRelay(Node):
    def __init__(self):
        super().__init__('scan_timestamp_relay')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

    def callback(self, msg):
        # Ersetze Zeitstempel durch einen leicht in der Vergangenheit liegenden Wert
        adjusted_time = self.get_clock().now() - Duration(seconds=0.1)
        msg.header.stamp = adjusted_time.to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
