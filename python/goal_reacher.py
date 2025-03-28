import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray
from unique_identifier_msgs.msg import UUID
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from std_msgs.msg import Header
import math
import tf_transformations
import uuid

class ProGoalRepeater(Node):
    def __init__(self):
        super().__init__('pro_goal_repeater')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',
            self.status_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.retry_if_needed)

        self.current_goal_id = None
        self.last_abort_time = None
        self.retry_in_progress = False
        self.offset_applied = False

    def status_callback(self, msg: GoalStatusArray):
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)

            if self.current_goal_id is None or goal_id != self.current_goal_id:
                continue  # Nicht unser Ziel – ignorieren

            if status.status == 6 and not self.retry_in_progress:
                self.get_logger().warn("❌ Goal aborted! Scheduling retry...")
                self.last_abort_time = self.get_clock().now()
                self.retry_in_progress = True

            elif status.status == 3:
                self.get_logger().info("✅ Goal succeeded.")
                self.current_goal_id = None
                self.retry_in_progress = False
                self.offset_applied = False

    def retry_if_needed(self):
        if not self.retry_in_progress or self.last_abort_time is None:
            return

        # Warte 2 Sekunden nach Abbruch
        time_passed = (self.get_clock().now() - self.last_abort_time).nanoseconds / 1e9
        if time_passed < 2.0:
            return

        pose = self.get_robot_pose()
        if pose:
            if not self.offset_applied:
                yaw = self.get_yaw_from_quaternion(pose.pose.orientation)
                pose.pose.position.x -= 0.00001 * math.cos(yaw)
                pose.pose.position.y -= 0.00001 * math.sin(yaw)
                self.offset_applied = True

            self.goal_pub.publish(pose)

            # Neue Ziel-ID erzeugen und merken
            new_uuid = uuid.uuid4().bytes
            self.current_goal_id = new_uuid
            self.retry_in_progress = False
            self.get_logger().info(f"🔁 Retry sent with new goal_id: {uuid.UUID(bytes=new_uuid)}")

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position = trans.transform.translation
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f"⚠️ TF lookup failed: {e}")
            return None

    def get_yaw_from_quaternion(self, orientation):
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = ProGoalRepeater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
