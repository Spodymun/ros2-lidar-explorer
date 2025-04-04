#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
import subprocess
import time
from threading import Thread


class ExploreRelauncher(Node):
    def __init__(self):
        super().__init__('explore_relauncher')

        self.last_frontier_time = time.time()
        self.last_map_change_time = time.time()
        self.last_restart_time = 0
        self.explore_ready_time = None
        self.first_frontier_seen = False

        self.restart_delay = 10  # seconds
        self.frontier_timeout = 12  # seconds
        self.map_change_timeout = 30  # seconds
        self.check_interval = 5  # seconds
        self.max_restarts_without_progress = 3
        self.monitor_after_restart_delay = 20  # seconds after restart to evaluate

        self.frontiers_exist = False
        self.last_unknown_count = None
        self.restarts_without_progress = 0

        self.create_subscription(MarkerArray, '/explore/frontiers', self.frontier_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_timer(self.check_interval, self.check_conditions)

        self.declare_parameter('explore_cancelled', False)
        self.add_on_set_parameters_callback(self.parameter_callback)

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

        print(f"[CHECK] frontier_timed_out={frontier_timed_out}, map_stagnant={map_stagnant}, recently_restarted={recently_restarted}")
        print(f"[CHECK] Restart counter: {self.restarts_without_progress}")

        if not recently_restarted and frontier_timed_out and map_stagnant:
            self.restarts_without_progress += 1
            print(f"[RESTART] Triggered. Count: {self.restarts_without_progress}")

            if self.restarts_without_progress >= self.max_restarts_without_progress:
                print("\n🎉 DONE! No progress after multiple restarts. Shutting down. 🎉🎉🎉")
                self.destroy_explore()
                rclpy.shutdown()
                return

            self.restart_explore()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'explore_cancelled' and param.value is True:
                print("[RESTART] Cancel detected via parameter callback. Relaunching explore immediately.")
                self.set_parameters([Parameter('explore_cancelled', Parameter.Type.BOOL, False)])
                self.restart_explore()
        return rclpy.parameter.ParameterEventDescriptors()

    def monitor_after_restart(self):
        time.sleep(self.monitor_after_restart_delay)
        print("[MONITOR] Checking post-restart movement...")
        no_movement = (time.time() - self.last_map_change_time) > self.monitor_after_restart_delay

        if no_movement and not self.frontiers_exist:
            self.restarts_without_progress += 1
            print("[MONITOR] No movement + no frontiers. Incrementing fail counter.")
        else:
            print("[MONITOR] Movement detected or frontiers found. Resetting counter.")
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
                print("[MONITOR] Connected to Nav2 detected.")
                monitor = Thread(target=self.monitor_after_restart, daemon=True)
                monitor.start()
                break  # Only trigger monitor once per restart

    def restart_explore(self):
        print("[RESTART] Restarting explore_lite process...")
        self.destroy_explore()
        time.sleep(1)
        self.last_restart_time = time.time()
        self.start_explore()

    def destroy_explore(self):
        if self.explore_process:
            self.explore_process.terminate()
            self.explore_process.wait()
            self.explore_process = None


def main():
    rclpy.init()
    node = ExploreRelauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
