#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, JointState
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np

class ScanToPointCloudNode(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        self.publisher = self.create_publisher(
            PointCloud2,
            '/servo_lidar/pointcloud',
            10)

        self.current_angle_rad = 0.0  # Default: 0° Servo-Winkel

    def joint_callback(self, msg: JointState):
        try:
            index = msg.name.index('servo_joint')
            self.current_angle_rad = msg.position[index]
        except ValueError:
            pass  # Joint nicht gefunden

    def scan_callback(self, scan: LaserScan):
        points = []

        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = -math.tan(self.current_angle_rad) * x  # Höhe aus Servo-Winkel
                points.append([x, y, z])
            angle += scan.angle_increment

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'  # Oder 'map' je nach TF-Basis

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud_msg)

def main():
    rclpy.init()
    node = ScanToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
