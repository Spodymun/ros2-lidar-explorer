#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse

import time

class NavigateRelay(Node):
    def __init__(self):
        super().__init__('navigate_relay')

        self.declare_parameter('explore_cancelled', False)

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            '/explore/navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_active = False
        self.last_goal_time = self.get_clock().now()
        self.min_execution_duration = Duration(seconds=15)  # Zeit zwischen Zielen

    def goal_callback(self, goal_request):
        if self.goal_active:
            self.get_logger().warn('Ziel empfangen, aber vorheriges Ziel ist noch aktiv.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Zielabbruch angefragt – wird durchgereicht.')
        self.set_parameters([rclpy.parameter.Parameter('explore_cancelled', rclpy.Parameter.Type.BOOL, True)])
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2-Action-Server nicht verfügbar.')
            goal_handle.abort()
            return NavigateToPose.Result()

        now = self.get_clock().now()
        if now - self.last_goal_time < self.min_execution_duration:
            self.get_logger().warn('Zu früh für neues Ziel – überspringe.')
            goal_handle.abort()
            return NavigateToPose.Result()

        self.goal_active = True
        self.last_goal_time = now

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = goal_handle.request.pose

        self.get_logger().info(f'Ziel weitergeleitet an Nav2: {nav2_goal.pose.pose.position.x:.2f}, {nav2_goal.pose.pose.position.y:.2f}')
        nav2_future = self.nav2_client.send_goal_async(nav2_goal)
        nav2_goal_handle = await nav2_future

        if not nav2_goal_handle.accepted:
            self.get_logger().warn('Nav2 hat Ziel abgelehnt.')
            goal_handle.abort()
            self.goal_active = False
            return NavigateToPose.Result()

        result_future = nav2_goal_handle.get_result_async()
        await result_future

        result = result_future.result().result
        status = result_future.result().status

        self.goal_active = False

        if status == 4:  # ABORTED
            goal_handle.abort()
            self.get_logger().warn('Ziel wurde von Nav2 abgebrochen (ABORTED).')
        elif status == 3:  # SUCCEEDED
            goal_handle.succeed()
            self.get_logger().info('Ziel erfolgreich erreicht (SUCCEEDED).')
        elif status == 2:  # CANCELED
            goal_handle.canceled()
            self.get_logger().warn('Ziel wurde von Nav2 abgebrochen (CANCELED).')
            self.set_parameters([rclpy.parameter.Parameter('explore_cancelled', rclpy.Parameter.Type.BOOL, True)])
        else:
            goal_handle.abort()
            self.get_logger().warn(f'Zielstatus unerwartet: {status} – wird als ABORTED behandelt.')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigateRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
