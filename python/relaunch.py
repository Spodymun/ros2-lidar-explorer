#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, Int8
import subprocess
import time
import sys
import os
import math
from threading import Thread


def wait_for_map_topic(node, topic='/map', timeout=30.0):
    """Wait for map data to become available on the specified topic."""
    has_data = False

    def callback(msg):
        nonlocal has_data
        has_data = True

    qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

    sub = node.create_subscription(OccupancyGrid, topic, callback, qos)

    start_time = node.get_clock().now().seconds_nanoseconds()[0]
    node.get_logger().info(f"Waiting for map data on '{topic}'...")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
        current_time = node.get_clock().now().seconds_nanoseconds()[0]

        if has_data:
            node.get_logger().info(f"Map data received on '{topic}'.")
            break

        if current_time - start_time > timeout:
            node.get_logger().error(f"Timeout waiting for map data on '{topic}'.")
            break

    node.destroy_subscription(sub)
    return has_data


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

        # Goal tracking
        self.current_goal_position = None
        self.reached_goal_clusters = []  # list of {'pos': (x, y), 'count': int}
        self.homing_phase = False  # indicates map saved and homing in progress

        # Parameters
        self.min_progress_distance = 0.03  # 3 cm
        self.restart_delay = 10  # seconds
        self.frontier_timeout = 12  # seconds
        self.check_interval = 5  # seconds
        self.max_restarts_without_progress = 2
        self.monitor_after_restart_delay = 20  # seconds
        self.same_goal_radius = 0.05  # 5 cm
        self.same_goal_threshold = 5  # count to trigger

        # State
        self.frontiers_exist = False
        self.restarts_without_progress = 0
        self.goal_active = False

        # Subscriptions
        self.create_subscription(Int8, '/explore/goal_result_status', self.goal_result_cb, 10)
        self.create_subscription(Bool, '/explore/goal_status', self.goal_status_cb, 10)
        self.create_subscription(MarkerArray, '/explore/frontiers', self.frontier_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Timer
        self.timer = self.create_timer(self.check_interval, self.check_conditions)

        # Start exploration
        self.start_explore()

    def goal_status_cb(self, msg: Bool):
        self.goal_active = msg.data

    def goal_result_cb(self, msg: Int8):
        status_code = msg.data
        # Check home goal with tolerance ±25 cm and then shutdown
        if self.current_goal_position is not None:
            dx = self.current_goal_position[0]
            dy = self.current_goal_position[1]
            if math.hypot(dx, dy) <= 0.25 and status_code == 3:
                self.get_logger().info("Robot has reached home (0,0) within tolerance. Shutting down.")
                rclpy.shutdown()
                return
        # On any other reached goal
        if status_code == 3 and self.current_goal_position is not None:
            self.handle_reached_goal(self.current_goal_position)

    def handle_reached_goal(self, pos):
        # Cluster detection
        for cluster in self.reached_goal_clusters:
            dx = pos[0] - cluster['pos'][0]
            dy = pos[1] - cluster['pos'][1]
            if math.hypot(dx, dy) <= self.same_goal_radius:
                cluster['count'] += 1
                self.get_logger().info(f"Reached same goal cluster {cluster['pos']} count={cluster['count']}")
                if cluster['count'] >= self.same_goal_threshold and not self.homing_phase:
                    self.get_logger().info("Same goal reached threshold. Saving map & sending robot home.")
                    self.save_map_and_home()
                return
        # New cluster
        self.reached_goal_clusters.append({'pos': pos, 'count': 1})
        self.get_logger().info(f"New goal cluster created at {pos}")

    def frontier_cb(self, msg: MarkerArray):
        self.frontiers_exist = bool(msg.markers)
        if self.frontiers_exist:
            self.last_frontier_time = time.time()

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        current = (pos.x, pos.y)
        if self.last_pose is not None:
            dx = current[0] - self.last_pose[0]
            dy = current[1] - self.last_pose[1]
            dist = math.hypot(dx, dy)
            if dist > self.min_progress_distance:
                self.no_movement_since = time.time()
                self.restarts_without_progress = 0
                self.get_logger().info(f"Movement detected: {dist:.3f} m")
        self.last_pose = current

    def check_conditions(self):
        # Skip any logic once homing has begun
        if self.homing_phase:
            return
        now = time.time()
        frontier_timeout = not self.goal_active and (now - self.last_frontier_time) > self.frontier_timeout
        no_move = (now - self.no_movement_since) > self.monitor_after_restart_delay
        recent = (now - self.last_restart_time) < self.restart_delay

        self.get_logger().info(f"frontier_timeout={frontier_timeout}, no_move={no_move}, recent_restart={recent}")
        self.get_logger().info(f"Restart counter: {self.restarts_without_progress}")

        if not recent and frontier_timeout and no_move:
            self.restarts_without_progress += 1
            self.get_logger().info(f"Restart triggered. Count: {self.restarts_without_progress}")
            if self.restarts_without_progress >= self.max_restarts_without_progress:
                self.get_logger().info("No progress after multiple restarts. Saving map & sending robot home.")
                self.save_map_and_home()
            else:
                self.restart_explore()

    def restart_explore(self):
        self.get_logger().info("Restarting explore process.")
        self.destroy_explore()
        time.sleep(1)
        self.last_restart_time = time.time()
        self.start_explore()

    def destroy_explore(self):
        if hasattr(self, 'explore_process') and self.explore_process:
            self.explore_process.terminate()
            self.explore_process.wait()
            self.explore_process = None

    def start_explore(self):
        # Only start if not in homing phase
        if self.homing_phase:
            return
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
        Thread(target=self.monitor_explore_output, daemon=True).start()

    def monitor_explore_output(self):
        for line in self.explore_process.stdout:
            if "Connected to move_base Nav2 server" in line:
                self.get_logger().info("Nav2 server connected.")
                Thread(target=self.monitor_after_restart, daemon=True).start()
                break

    def monitor_after_restart(self):
        time.sleep(self.monitor_after_restart_delay)
        self.get_logger().info("Post-restart movement check...")
        if (time.time() - self.no_movement_since) > self.monitor_after_restart_delay and not self.frontiers_exist:
            self.restarts_without_progress += 1
            self.get_logger().info("No move & no frontiers. Fail counter incremented.")
        else:
            self.get_logger().info("Movement or frontiers detected. Resetting counter.")
            self.restarts_without_progress = 0

    def send_robot_home(self):
        self.get_logger().info("Sending robot to home (0,0).")
        self.current_goal_position = (0.0, 0.0)
        def home_loop():
            while rclpy.ok():
                try:
                    res = subprocess.run([
                        'ros2', 'action', 'send_goal', '/explore/naviate_to_pose',
                        'nav2_msgs/action/NavigateToPose',
                        '{pose: {header: {frame_id: "map"}, pose: '
                        '{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
                    ], capture_output=True, text=True)
                    output = (res.stdout or "") + (res.stderr or "")
                    if 'Goal reached' in output:
                        self.get_logger().info('Home reached! Shutting down.')
                        rclpy.shutdown()
                        return
                except Exception as e:
                    self.get_logger().error(f"Navigation exception: {e}")
                time.sleep(5)
        Thread(target=home_loop, daemon=True).start()

    def save_map_and_home(self):
        self.homing_phase = True
        if hasattr(self, 'timer'):
            self.timer.cancel()
        self.destroy_explore()
        if not wait_for_map_topic(self, '/map', timeout=30):
            self.get_logger().error("Aborting: No map data available.")
            return
        folder = f"/home/robi/ws_lidar/src/ros2-lidar-explorer/maps/{self.map_name}"
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, "map")
        self.get_logger().info(f"Saving map to: {path}")
        try:
            res = subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', path,
                '--ros-args', '-p', 'map_subscribe_transient_local:=true'
            ], check=True, capture_output=True, text=True)
            self.get_logger().info(f"Map successfully saved:\n{res.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error saving map: {e.stderr}")
            return
        self.send_robot_home()


def main():
    rclpy.init()
    node = ExploreRelauncher()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
