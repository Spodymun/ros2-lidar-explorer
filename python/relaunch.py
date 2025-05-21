#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8
import subprocess
import time
import sys
import os
import math
from threading import Thread


class ExploreRelauncher(Node):
    # ------------------------------------------------------------
    # constructor
    # ------------------------------------------------------------
    def __init__(self):
        super().__init__('explore_relauncher')

        # -------- runtime parameters --------
        self.map_name                     = sys.argv[1]
        self.min_progress_distance        = 0.03   # m
        self.restart_delay                = 10     # s – do not restart twice in this window
        self.frontier_timeout             = 12     # s – no frontiers & no goal active
        self.check_interval               = 5      # s – main watchdog period
        self.monitor_after_restart_delay  = 20     # s – grace period after restart
        self.max_restarts_without_progress= 2
        self.same_goal_radius             = 0.05   # m clustering radius
        self.same_goal_threshold          = 5      # hits until we abort exploration

        # -------- state variables --------
        self.last_frontier_time           = time.time()
        self.last_restart_time            = 0
        self.no_movement_since            = time.time()
        self.last_pose                    = None
        self.restarts_without_progress    = 0
        self.frontiers_exist              = False
        self.goal_active                  = False
        self.homing_phase                 = False

        # cluster tracking
        self.reached_goal_clusters: list[dict] = []   # [{'pos': (x,y), 'count': n}, …]
        self.current_goal_position: tuple|None = None  # set only when we send a goal

        # -------- subscriptions --------
        self.create_subscription(Bool, '/explore/goal_status',
                                 self.goal_status_cb,      10)
        self.create_subscription(Int8, '/explore/goal_result_status',
                                 self.goal_result_cb,      10)
        self.create_subscription(MarkerArray, '/explore/frontiers',
                                 self.frontier_cb,         10)
        self.create_subscription(Odometry, '/odom',
                                 self.odom_cb,             10)

        # -------- watchdog timer --------
        self.timer = self.create_timer(self.check_interval, self.check_conditions)

        # -------- start explorer --------
        self.start_explore()
        self.get_logger().info(f"Explore relauncher started – map name: {self.map_name}")

    # ------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------
    def goal_status_cb(self, msg: Bool):
        """True while Nav2 is processing an active goal."""
        self.goal_active = msg.data

    def goal_result_cb(self, msg: Int8):
        """Handle Nav2 result codes (3 = SUCCEEDED)."""
        status = msg.data
        if status != 3:
            return

        # did we just reach home? – shutdown
        if self.homing_phase and self.current_goal_position == (0.0, 0.0):
            self.get_logger().info("Home reached – shutting down.")
            rclpy.shutdown()
            return

        # during exploration: update cluster statistics
        if self.current_goal_position is not None and not self.homing_phase:
            self.handle_reached_goal(self.current_goal_position)

    def frontier_cb(self, msg: MarkerArray):
        """Remember timestamp of last frontier set published."""
        self.frontiers_exist = bool(msg.markers)
        if self.frontiers_exist:
            self.last_frontier_time = time.time()

    def odom_cb(self, msg: Odometry):
        """Detect real movement to reset 'no progress' timer."""
        p   = msg.pose.pose.position
        pos = (p.x, p.y)
        if self.last_pose:
            if math.hypot(pos[0] - self.last_pose[0], pos[1] - self.last_pose[1]) > self.min_progress_distance:
                self.no_movement_since = time.time()
                self.restarts_without_progress = 0
        self.last_pose = pos

    # ------------------------------------------------------------
    # cluster handling
    # ------------------------------------------------------------
    def handle_reached_goal(self, pos: tuple):
        """Count how often the same goal cluster is reached."""
        for cluster in self.reached_goal_clusters:
            if math.hypot(pos[0]-cluster['pos'][0], pos[1]-cluster['pos'][1]) <= self.same_goal_radius:
                cluster['count'] += 1
                self.get_logger().info(f"Cluster {cluster['pos']} hit {cluster['count']}/{self.same_goal_threshold}")
                if cluster['count'] >= self.same_goal_threshold:
                    self.get_logger().info("Cluster threshold reached – saving map & going home.")
                    self.save_map_and_home()
                return
        # new cluster
        self.reached_goal_clusters.append({'pos': pos, 'count': 1})
        self.get_logger().info(f"New goal cluster created @ {pos}")

    # ------------------------------------------------------------
    # main watchdog
    # ------------------------------------------------------------
    def check_conditions(self):
        if self.homing_phase:
            return

        now = time.time()
        frontier_timeout = not self.goal_active and (now - self.last_frontier_time) > self.frontier_timeout
        no_move          = (now - self.no_movement_since)  > self.monitor_after_restart_delay
        recent_restart   = (now - self.last_restart_time)  < self.restart_delay

        if not recent_restart and frontier_timeout and no_move:
            self.restarts_without_progress += 1
            self.get_logger().info(f"Restart trigger {self.restarts_without_progress}/"
                                   f"{self.max_restarts_without_progress}")
            if self.restarts_without_progress >= self.max_restarts_without_progress:
                self.save_map_and_home()
            else:
                self.restart_explore()

    # ------------------------------------------------------------
    # explore process management
    # ------------------------------------------------------------
    def start_explore(self):
        """Launch explore_lite as a separate process."""
        if self.homing_phase:
            return
        self.explore_proc = subprocess.Popen([
            'ros2', 'run', 'explore_lite', 'explore',
            '--ros-args',
            '--params-file', '/home/robi/ws_lidar/src/m-explore-ros2/explore/config/params.yaml',
            '-p', 'use_sim_time:=false',
            '-r', '/navigate_to_pose:=/explore/navigate_to_pose',
            '-r', '/map:=/map'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, text=True)

    def restart_explore(self):
        """Terminate and relaunch explorer."""
        if getattr(self, 'explore_proc', None):
            self.explore_proc.terminate()
            self.explore_proc.wait()
        self.last_restart_time = time.time()
        self.start_explore()

    # ------------------------------------------------------------
    # map saving and homing
    # ------------------------------------------------------------
    def save_map_and_home(self):
        """Save map immediately and start homing sequence."""
        self.homing_phase = True
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if getattr(self, 'explore_proc', None):
            self.explore_proc.terminate()
            self.explore_proc.wait()

        folder = f"/home/robi/ws_lidar/src/ros2-lidar-explorer/maps/{self.map_name}"
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, "map")
        self.get_logger().info(f"Saving map to: {path}")

        try:
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                            '-f', path],
                           check=True, capture_output=True, text=True)
            self.get_logger().info("Map saved successfully.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Map save failed: {e.stderr}")

        self.send_robot_home()

    def send_robot_home(self):
        """Continuously send a NavigateToPose goal at (0,0) until SUCCEEDED."""
        self.get_logger().info("Sending robot home.")
        self.current_goal_position = (0.0, 0.0)   # used by result callback

        def loop():
            while rclpy.ok():
                subprocess.run([
                    'ros2', 'action', 'send_goal', '/explore/navigate_to_pose',
                    'nav2_msgs/action/NavigateToPose',
                    '{pose: {header: {frame_id: "map"}, pose: '
                    '{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
                ], capture_output=True)
                time.sleep(5)
        Thread(target=loop, daemon=True).start()


# ------------------------------------------------------------
# main entry
# ------------------------------------------------------------
def main():
    rclpy.init()
    node = ExploreRelauncher()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
