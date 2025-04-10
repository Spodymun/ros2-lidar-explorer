#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse

class NavigateRelay(Node):
    def __init__(self):
        super().__init__('navigate_relay')

        # Declare a parameter to track whether exploration was cancelled
        self.declare_parameter('explore_cancelled', False)

        # Action server to receive goals from explore_lite
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            '/explore/navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Action client to forward goals to the main Nav2 NavigateToPose server
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # State tracking
        self.goal_active = False
        self.last_goal_time = self.get_clock().now()
        self.min_execution_duration = Duration(seconds=15)  # Minimum time between goals

    def goal_callback(self, goal_request):
        """Decide whether to accept a new goal request."""
        if self.goal_active:
            self.get_logger().warn('New goal received, but previous goal is still active.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation requests."""
        self.get_logger().info('Goal cancellation requested – setting explore_cancelled = True')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'explore_cancelled',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal by forwarding it to Nav2 NavigateToPose."""
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available.')
            goal_handle.abort()
            return NavigateToPose.Result()

        now = self.get_clock().now()

        # Prevent rapid re-submission of goals if one is active and time hasn't passed
        if self.goal_active and now - self.last_goal_time < self.min_execution_duration:
            self.get_logger().warn('Too soon for a new goal – skipping.')
            goal_handle.abort()
            return NavigateToPose.Result()

        self.goal_active = True
        self.last_goal_time = now

        # Forward goal to Nav2
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = goal_handle.request.pose

        self.get_logger().info(
            f'Forwarding goal to Nav2: x={nav2_goal.pose.pose.position.x:.2f}, y={nav2_goal.pose.pose.position.y:.2f}'
        )

        nav2_future = self.nav2_client.send_goal_async(nav2_goal)
        nav2_goal_handle = await nav2_future

        if not nav2_goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            goal_handle.abort()
            self.goal_active = False
            return NavigateToPose.Result()

        # Wait for the goal to complete
        result_future = nav2_goal_handle.get_result_async()
        await result_future

        result = result_future.result().result
        status = result_future.result().status

        self.goal_active = False

        # Handle final status
        if status == 4:  # ABORTED
            goal_handle.abort()
            self.get_logger().warn('Goal was aborted by Nav2.')
        elif status == 3:  # SUCCEEDED
            goal_handle.succeed()
            self.get_logger().info('Goal successfully reached.')
        elif status == 2:  # CANCELED
            goal_handle.canceled()
            self.get_logger().warn('Goal was canceled.')
        else:
            goal_handle.abort()
            self.get_logger().warn(f'Unexpected goal status: {status} – treating as aborted.')

        return result

def main(args=None):
    """Main function to initialize the node and spin it."""
    rclpy.init(args=args)
    node = NavigateRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
