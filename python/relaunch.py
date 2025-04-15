#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
import subprocess
import time
from threading import Thread
import sys
import os


class ExploreRelauncher(Node):
    def __init__(self):
        super().__init__('explore_relauncher')

        # Get the map name from the command line argument
        self.map_name = sys.argv[1]
        self.get_logger().info(f"Received map name: {self.map_name}")

        self.last_frontier_time = time.time()
        self.last_map_change_time = time.time()
        self.last_restart_time = 0
        self.explore_ready_time = None
        self.first_frontier_seen = False

        # Set time limits for different checks
        self.restart_delay = 10  # seconds
        self.frontier_timeout = 12  # seconds
        self.map_change_timeout = 30  # seconds
        self.check_interval = 5  # seconds
        self.max_restarts_without_progress = 2
        self.monitor_after_restart_delay = 20  # seconds after restart to evaluate

        # State variables
        self.frontiers_exist = False
        self.last_unknown_count = None
        self.restarts_without_progress = 0

        # Subscriptions
        self.create_subscription(MarkerArray, '/explore/frontiers', self.frontier_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_timer(self.check_interval, self.check_conditions)

        # Exploration process and thread
        self.explore_process = None
        self.explore_monitor_thread = None
        self.start_explore()

    def frontier_cb(self, msg):
        self.frontiers_exist = bool(msg.markers)
        if self.frontiers_exist:
            self.first_frontier_seen = True
            self.last_frontier_time = time.time()

    def map_cb(self, msg):
        unknown = sum(1 for cell in msg.data if cell == -1)
        if self.last_unknown_count is None or unknown != self.last_unknown_count:
            self.last_unknown_count = unknown
            self.last_map_change_time = time.time()

    def check_conditions(self):
        now = time.time()
        frontier_timed_out = (now - self.last_frontier_time) > self.frontier_timeout
        map_stagnant = (now - self.last_map_change_time) > self.map_change_timeout
        recently_restarted = (now - self.last_restart_time) < self.restart_delay

        self.get_logger().info(f"frontier_timed_out={frontier_timed_out}, map_stagnant={map_stagnant}, recently_restarted={recently_restarted}")
        self.get_logger().info(f"Restart counter: {self.restarts_without_progress}")

        # Reset counter if progress is made
        if not frontier_timed_out or not map_stagnant:
            if self.restarts_without_progress > 0:
                self.get_logger().info("Progress detected – resetting restart counter.")
            
            if map_stagnant:
                self.get_logger().info("Map still stagnant despite progress – setting restart counter to 1.")
                self.restarts_without_progress = 1
            else:
                self.restarts_without_progress = 0

        if not recently_restarted and frontier_timed_out and map_stagnant:
            self.restarts_without_progress += 1
            self.get_logger().info(f"Restart triggered. Count: {self.restarts_without_progress}")

            if self.restarts_without_progress >= self.max_restarts_without_progress:
                self.get_logger().info("No progress after multiple restarts. Saving map, then returning to start, then shutting down...")

                self.destroy_explore()

                # Save map
                map_folder = f"/home/robi/ws_lidar/src/ros2-lidar-explorer/maps/{self.map_name}"
                os.makedirs(map_folder, exist_ok=True)

                map_save_path = os.path.join(map_folder, "map")
                self.get_logger().info(f"Saving map to: {map_save_path}")

                try:
                    subprocess.run([
                        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                        '-f', map_save_path
                    ], check=True)
                    self.get_logger().info("Map saved successfully.")
                except subprocess.CalledProcessError as e:
                    self.get_logger().error(f"Failed to save map: {e}")

                # Send robot back to start position (0, 0)
                self.send_robot_home()

                if rclpy.ok():
                    rclpy.shutdown()
                return

            self.restart_explore()

    def monitor_after_restart(self):
        time.sleep(self.monitor_after_restart_delay)
        self.get_logger().info("Checking post-restart movement...")
        no_movement = (time.time() - self.last_map_change_time) > self.monitor_after_restart_delay

        if no_movement and not self.frontiers_exist:
            self.restarts_without_progress += 1
            self.get_logger().info("No movement + no frontiers. Incrementing fail counter.")
        else:
            self.get_logger().info("Movement detected or frontiers found. Resetting counter.")
            self.restarts_without_progress = 0

    def start_explore(self):
        self.explore_process = subprocess.Popen([
            'ros2', 'run', 'explore_lite', 'explore',
            '--ros-args',
            '--params-file', '/home/robi/ws_lidar/src/m-explore-ros2/explore/config/params.yaml',
            '-p', 'use_sim_time:=false',
            '-r', '/navigate_to_pose:=/explore/navigate_to_pose',
            '-r', '/map:=/map',
            '-r', '/cmd_vel:=/cmd_vel',
            '-r', '/tf:=/tf',
            '-r', '/tf_static:=/tf_static'
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        self.explore_start_time = time.time()
        self.first_frontier_seen = False
        self.explore_ready_time = None

        self.explore_monitor_thread = Thread(target=self.monitor_explore_output, daemon=True)
        self.explore_monitor_thread.start()

    def monitor_explore_output(self):
        for line in self.explore_process.stdout:
            if "Connected to move_base Nav2 server" in line:
                self.explore_ready_time = time.time()
                self.get_logger().info("Connected to Nav2 detected.")
                monitor = Thread(target=self.monitor_after_restart, daemon=True)
                monitor.start()
                break

    def restart_explore(self):
        self.get_logger().info("Restarting explore_lite process...")
        self.destroy_explore()
        time.sleep(1)
        self.last_restart_time = time.time()
        self.start_explore()

    def destroy_explore(self):
        if self.explore_process:
            self.explore_process.terminate()
            self.explore_process.wait()
            self.explore_process = None

    def send_robot_home(self):
        self.get_logger().info("Attempting to send robot back to (0, 0)...")

        attempt = 1
        while rclpy.ok():
            try:
                result = subprocess.run([
                    'ros2', 'action', 'send_goal', '/explore/navigate_to_pose',
                    'nav2_msgs/action/NavigateToPose',
                    '{pose: {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}',
                ], capture_output=True, text=True)

                if result.returncode == 0:
                    output = result.stdout
                    if "Goal reached" in output or "SUCCEEDED" in output:
                        self.get_logger().info("Robot has successfully reached (0, 0).")
                        rclpy.shutdown()
                        return
                    else:
                        self.get_logger().warn(f"[Attempt {attempt}] Goal sent but not reached. Output:\n{output}")
                else:
                    self.get_logger().warn(f"[Attempt {attempt}] Navigation command failed:\n{result.stderr.strip()}")

            except Exception as e:
                self.get_logger().error(f"Exception during navigation: {str(e)}")

            attempt += 1
            time.sleep(5)

def main():
    rclpy.init()
    node = ExploreRelauncher()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
