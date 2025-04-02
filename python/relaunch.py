#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
import subprocess
import time


class ExploreRelauncher(Node):
    def __init__(self):
        super().__init__('explore_relauncher')

        self.last_frontier_time = time.time()
        self.frontiers_exist = False
        self.has_open_white_edge = False
        self.map_unknown_ratio = 1.0
        self.restart_delay = 15  # seconds
        self.check_interval = 5  # seconds
        self.unknown_threshold = 0.02  # if less than 2% unknown, map is "done"

        self.create_subscription(MarkerArray, '/explore/frontiers', self.frontier_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        self.create_timer(self.check_interval, self.check_conditions)

        self.explore_process = None
        self.start_explore()

    def frontier_cb(self, msg):
        self.frontiers_exist = bool(msg.markers)
        if self.frontiers_exist:
            self.last_frontier_time = time.time()
            self.get_logger().info(f"🟢 Frontiers sichtbar: {len(msg.markers)}")
        else:
            self.get_logger().info("🟡 Keine sichtbaren Frontiers.")

    def map_cb(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        unknown = 0
        edge_found = False

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x
                if data[idx] == 0:  # free space
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            n_idx = (y + dy) * width + (x + dx)
                            if 0 <= n_idx < len(data) and data[n_idx] == -1:
                                edge_found = True
                                break
                        if edge_found:
                            break
                if data[idx] == -1:
                    unknown += 1

        self.has_open_white_edge = edge_found
        self.map_unknown_ratio = unknown / len(data) if len(data) > 0 else 1.0

        self.get_logger().info(
            f"🗺️ Karte: {self.map_unknown_ratio:.2%} unbekannt | Offen am Rand: {self.has_open_white_edge}"
        )

    def check_conditions(self):
        now = time.time()
        no_frontier_recently = (now - self.last_frontier_time) > self.restart_delay
        map_done = self.map_unknown_ratio < self.unknown_threshold

        if map_done:
            self.get_logger().info("✅ Karte vollständig – keine offenen Stellen mehr.")
            return

        if no_frontier_recently and self.has_open_white_edge:
            self.get_logger().warn("🔁 Neustart: Keine sichtbaren Frontiers, aber Karte offen – versuche neu.")
            self.restart_explore()
            self.last_frontier_time = now
        else:
            self.get_logger().info("⏳ Warte: Noch sichtbar oder kein Restart nötig.")

    def start_explore(self):
        self.get_logger().info("🚀 Starte explore_lite...")
        self.explore_process = subprocess.Popen([
            'ros2', 'run', 'explore_lite', 'explore',
            '--ros-args',
            '--params-file', '/home/robi/ws_lidar/src/m-explore-ros2/explore/config/params.yaml',
            '-p', 'use_sim_time:=false',
            '-r', '/map:=/map',
            '-r', '/cmd_vel:=/cmd_vel',
            '-r', '/tf:=/tf',
            '-r', '/tf_static:=/tf_static'
        ])
        self.get_logger().info(f"🧠 explore_lite gestartet (PID: {self.explore_process.pid})")

    def restart_explore(self):
        if self.explore_process:
            self.get_logger().info("🛑 Stoppe vorherigen explore_lite-Prozess...")
            self.explore_process.terminate()
            self.explore_process.wait()
            time.sleep(1)
        self.start_explore()


def main():
    rclpy.init()
    node = ExploreRelauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
