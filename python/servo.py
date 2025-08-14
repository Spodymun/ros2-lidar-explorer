#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import os
import sys
import math
from threading import Thread
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Servo SDK Setup
CANDIDATE_PATHS = [
    "/home/robi/ws_lidar/src/STServo_Python/stservo-env",
    "/home/robi/ws_lidar/src/STServo_Python",
]
for p in CANDIDATE_PATHS:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
try:
    from STservo_sdk import PortHandler, sts  # type: ignore
except Exception:
    PortHandler = None
    sts = None


class ServoSweepNode(Node):
    def __init__(self):
        super().__init__('servo_sweep_node')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameter
        self.declare_parameter("device", "/dev/ttyACM0")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("servo_id", 1)
        self.declare_parameter("speed", 175)
        self.declare_parameter("acc", 175)
        self.declare_parameter("min_deg", -30.0)
        self.declare_parameter("max_deg", 30.0)
        self.declare_parameter("delta_deg", 0.5)
        self.declare_parameter("mid_pos", 2048)
        self.declare_parameter("update_rate", 50.0)

        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.servo_id = self.get_parameter("servo_id").get_parameter_value().integer_value
        self.speed = self.get_parameter("speed").get_parameter_value().integer_value
        self.acc = self.get_parameter("acc").get_parameter_value().integer_value
        self.min_deg = self.get_parameter("min_deg").get_parameter_value().double_value
        self.max_deg = self.get_parameter("max_deg").get_parameter_value().double_value
        self.delta_deg = self.get_parameter("delta_deg").get_parameter_value().double_value
        self.mid_pos = self.get_parameter("mid_pos").get_parameter_value().integer_value
        self.update_rate = self.get_parameter("update_rate").get_parameter_value().double_value

        self.angle = self.max_deg
        self.direction = -1.0

        self.port = None
        self.servo = None
        self.running = True

        if PortHandler is None or sts is None:
            self.get_logger().error("STservo_SDK konnte nicht importiert werden.")
            return

        try:
            self.port = PortHandler(self.device)
            self.servo = sts(self.port)
            if not self.port.openPort():
                raise RuntimeError(f"Port öffnen fehlgeschlagen: {self.device}")
            if not self.port.setBaudRate(self.baud):
                raise RuntimeError(f"Baudrate setzen fehlgeschlagen: {self.baud}")
            try:
                self.servo.WriteEnable(self.servo_id, 1)
            except AttributeError:
                pass
            self.get_logger().info("✅ Servo initialisiert.")
        except Exception as e:
            self.get_logger().error(f"❌ Servo-Initialisierung fehlgeschlagen: {e}")
            return

        self.thread = Thread(target=self.sweep_loop, daemon=True)
        self.thread.start()

    def deg2pos(self, angle):
        return int(round(self.mid_pos + angle * 4096.0 / 360.0))

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return (qx, qy, qz, qw)

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'chassis'
        t.child_frame_id = 'servo_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.09

        q = self.euler_to_quaternion(0.0, math.radians(self.angle), 0.0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def sweep_loop(self):
        self.get_logger().info("🔁 Sweep gestartet")
        try:
            while rclpy.ok() and self.running:
                pos = self.deg2pos(self.angle)
                self.servo.WritePosEx(self.servo_id, pos, self.speed, self.acc)
                time.sleep(1.0 / self.update_rate)

                # JointState (optional)
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = ['servo_joint']
                joint_state.position = [math.radians(self.angle)]
                self.joint_pub.publish(joint_state)

                # TF transform senden
                self.broadcast_tf()

                # Winkel updaten
                if self.angle <= self.min_deg:
                    self.angle = self.min_deg
                    self.direction = 1.0
                elif self.angle >= self.max_deg:
                    self.angle = self.max_deg
                    self.direction = -1.0

                self.angle += self.direction * self.delta_deg
        except Exception as e:
            self.get_logger().error(f"Sweep-Fehler: {e}")

    def destroy_node(self):
        self.running = False
        try:
            if self.servo:
                self.servo.WritePosEx(self.servo_id, self.deg2pos(0.0), self.speed, self.acc)
                time.sleep(0.3)
                try:
                    self.servo.WriteEnable(self.servo_id, 0)
                except AttributeError:
                    pass
        finally:
            if self.port:
                self.port.closePort()
        super().destroy_node()


def main():
    rclpy.init()
    node = ServoSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
