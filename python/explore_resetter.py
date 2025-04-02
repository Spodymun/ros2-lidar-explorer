#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray

class ExploreResetter(Node):
    def __init__(self):
        super().__init__('explore_resetter')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.frontier_sub = self.create_subscription(
            MarkerArray, '/explore/frontiers', self.frontier_callback, 10
        )

        self.timer = self.create_timer(10.0, self.check_frontiers)
        self.frontier_count = 0
        self.get_logger().info('ExploreResetter gestartet und wartet auf Frontiers...')

    def frontier_callback(self, msg):
        self.frontier_count = len(msg.markers)

    def check_frontiers(self):
        if self.frontier_count == 0:
            self.get_logger().warn('Keine Frontiers erkannt – schubse Roboter leicht vorwärts...')
            self.nudge_forward()
            self.resume_exploration()
        else:
            self.get_logger().info(f'{self.frontier_count} Frontiers erkannt – alles ok.')

    def nudge_forward(self):
        twist = Twist()
        twist.linear.x = -0.1
        for _ in range(10):  # ca. 1 Sekunde lang
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def resume_exploration(self):
        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)
        self.get_logger().info('Exploration wieder aufgenommen.')

def main(args=None):
    rclpy.init(args=args)
    node = ExploreResetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
