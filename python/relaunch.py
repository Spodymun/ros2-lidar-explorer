#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import subprocess
import time
from threading import Thread
import sys
import os
import math
from std_msgs.msg import Bool, Int8

class ExploreRelauncher(Node):
    def __init__(self):
        super().__init__('explore_relauncher')

        # Get the map name from the command line argument
        self.map_name = sys.argv[1]
        self.get_logger().info(f"Received map name: {self.map_name}")

        # Time & movement tracking
        self.last_frontier_time = time.time()
        self.last_restart_time = 0
        self.no_movement_since = time.time()
        self.last_pose = None

        # Parameters
        self.min_progress_distance = 0.03  # 3 cm
        self.restart_delay = 10  # seconds
        self.frontier_timeout = 12  # seconds
        self.check_interval = 5  # seconds
        self.max_restarts_without_progress = 2
        self.monitor_after_restart_delay = 20  # seconds

        # State
        self.frontiers_exist = False
        self.restarts_without_progress = 0
        self.explore_process = None
        self.explore_monitor_thread = None
        self.goal_active = False
        self.current_goal_position = None

        # Subscriptions
        self.create_subscription(Int8, '/explore/goal_result_status', self.goal_result_cb, 10)
        self.create_subscription(Bool, '/explore/goal_status', self.goal_status_cb, 10)
        self.create_subscription(MarkerArray, '/explore/frontiers', self.frontier_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(self.check_interval, self.check_conditions)

        # Start exploration
        self.start_explore()

    def goal_status_cb(self, msg):
        self.goal_active = msg.data

    def goal_result_cb(self, msg):
        status_code = msg.data
        if self.current_goal_position == (0.0, 0.0) and status_code == 3:
            self.get_logger().info("Robot has reached (0, 0) successfully. Shutting down.")
            rclpy.shutdown()

    def frontier_cb(self, msg):
        self.frontiers_exist = bool(msg.markers)
        if self.frontiers_exist:
            self.last_frontier_time = time.time()

    def odom_cb(self, msg):
        position = msg.pose.pose.position
        current_pose = (position.x, position.y)

        if self.last_pose is not None:
            dx = current_pose[0] - self.last_pose[0]
            dy = current_pose[1] - self.last_pose[1]
            dist = math.hypot(dx, dy)

            if dist > self.min_progress_distance:
                self.no_movement_since = time.time()
                self.restarts_without_progress = 0
                self.get_logger().info(f"Movement detected: {dist:.3f} m")

        self.last_pose = current_pose

    def check_conditions(self):
        now = time.time()
        frontier_timed_out = not self.goal_active and (now - self.last_frontier_time) > self.frontier_timeout
        no_movement = (now - self.no_movement_since) > self.monitor_after_restart_delay
        recently_restarted = (now - self.last_restart_time) < self.restart_delay

        self.get_logger().info(f"frontier_timed_out={frontier_timed_out}, no_movement={no_movement}, recently_restarted={recently_restarted}")
        self.get_logger().info(f"Restart counter: {self.restarts_without_progress}")

        if not recently_restarted and frontier_timed_out and no_movement:
            self.restarts_without_progress += 1
            self.get_logger().info(f"Restart triggered. Count: {self.restarts_without_progress}")

            if self.restarts_without_progress >= self.max_restarts_without_progress:
                self.get_logger().info("No progress after multiple restarts. Saving map & sending robot home...")

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

                self.send_robot_home()
                return

            self.restart_explore()

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

        self.explore_monitor_thread = Thread(target=self.monitor_explore_output, daemon=True)
        self.explore_monitor_thread.start()

    def monitor_explore_output(self):
        for line in self.explore_process.stdout:
            if "Connected to move_base Nav2 server" in line:
                self.get_logger().info("Connected to Nav2 detected.")
                monitor = Thread(target=self.monitor_after_restart, daemon=True)
                monitor.start()
                break

    def monitor_after_restart(self):
        time.sleep(self.monitor_after_restart_delay)
        self.get_logger().info("Checking post-restart movement...")
        no_movement = (time.time() - self.no_movement_since) > self.monitor_after_restart_delay

        if no_movement and not self.frontiers_exist:
            self.restarts_without_progress += 1
            self.get_logger().info("No movement + no frontiers. Incrementing fail counter.")
        else:
            self.get_logger().info("Movement or frontiers detected. Resetting counter.")
            self.restarts_without_progress = 0

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
        self.current_goal_position = (0.0, 0.0)

        try:
            subprocess.run([
                'ros2', 'action', 'send_goal', '/explore/navigate_to_pose',
                'nav2_msgs/action/NavigateToPose',
                f'{{pose: {{header: {{frame_id: "map"}}, pose: {{position: {{x: 0.0, y: 0.0, z: 0.0}}, orientation: {{w: 1.0}}}}}}}}'
            ], capture_output=True, text=True)
        except Exception as e:
            self.get_logger().error(f"Exception during navigation: {str(e)}")


def main():
    rclpy.init()
    node = ExploreRelauncher()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
