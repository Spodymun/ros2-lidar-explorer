#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
import os
import sys
import math
from threading import Thread
from sensor_msgs.msg import JointState
from typing import Any, Dict, Optional, Iterable
import yaml

CANDIDATE_PATHS = [
    "/home/robi/ws_lidar/src/STServo_Python/stservo-env",
    "/home/robi/ws_lidar/src/STServo_Python",
]
for p in CANDIDATE_PATHS:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

PortHandler = None
sts_cls = None
try:
    from STservo_sdk import PortHandler as _PH, sts as _STS  # type: ignore
    PortHandler, sts_cls = _PH, _STS
except Exception:
    pass

if PortHandler is None:
    try:
        from scservo_sdk import PortHandler as _PH  # type: ignore
        PortHandler = _PH
    except Exception:
        pass
if sts_cls is None:
    try:
        from sms_sts import SMS_STS as _STS  # type: ignore
        sts_cls = _STS
    except Exception:
        pass


class ServoSweepNode(Node):
    def __init__(self):
        super().__init__('servo_sweep_node')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.declare_parameter("device", "/dev/ttyACM0")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("servo_id", 1)
        self.declare_parameter("speed", 175)
        self.declare_parameter("acc", 175)
        self.declare_parameter("delta_deg", 0.5)
        self.declare_parameter("mid_pos", 2048)
        self.declare_parameter("update_rate", 30.0)
        self.declare_parameter("calib_yaml", "")
        self.declare_parameter("pendulum_deg", 15.0)
        self.declare_parameter("publish_commanded", False)
        self.declare_parameter("feedback_stamp_delay_ms", 0.0)
        self.declare_parameter("debug_sdk", False)

        gp = lambda k: self.get_parameter(k).get_parameter_value()
        self.device = gp("device").string_value
        self.baud = gp("baud").integer_value
        self.servo_id = gp("servo_id").integer_value
        self.speed = gp("speed").integer_value
        self.acc = gp("acc").integer_value
        self.mid_pos = gp("mid_pos").integer_value
        self.update_rate = gp("update_rate").double_value
        self.calib_yaml = gp("calib_yaml").string_value
        self.pendulum_deg = abs(gp("pendulum_deg").double_value)
        self.publish_commanded = gp("publish_commanded").bool_value
        self.feedback_stamp_delay_ms = float(gp("feedback_stamp_delay_ms").double_value)
        self.debug_sdk = gp("debug_sdk").bool_value

        if self.calib_yaml:
            loaded_mid = self.try_load_mid_pos_from_yaml(self.calib_yaml)
            if loaded_mid is not None:
                self.mid_pos = int(loaded_mid)

        self.min_deg = -self.pendulum_deg
        self.max_deg = +self.pendulum_deg
        self.angle = self.max_deg
        self.direction = 1.0
        self._next_feedback_log_time = 0.0

        self.port = None
        self.servo = None
        self.running = True

        if PortHandler is None or sts_cls is None:
            return

        try:
            self.port = PortHandler(self.device)
            self.servo = sts_cls(self.port)
            opened = False
            if hasattr(self.port, "openPort"):
                opened = self.port.openPort()
            elif hasattr(self.port, "begin"):
                opened = self.port.begin(self.baud)
            if not opened:
                return
            if hasattr(self.port, "setBaudRate") and not self.port.setBaudRate(self.baud):
                return
            for attr in ("WriteEnable", "EnableTorque", "Load", "TorqueEnable"):
                f = getattr(self.servo, attr, None)
                if callable(f):
                    try:
                        f(self.servo_id, 1)
                        break
                    except Exception:
                        pass
        except Exception:
            return

        self.thread = Thread(target=self.sweep_loop, daemon=True)
        self.thread.start()

    def try_load_mid_pos_from_yaml(self, path: str) -> Optional[int]:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception:
            return None
        if not isinstance(data, dict):
            return None
        direct = data.get("servo_sweep_node")
        if isinstance(direct, dict):
            params = direct.get("ros__parameters")
            if isinstance(params, dict) and "mid_pos" in params:
                return int(params["mid_pos"])

        def deep_find_mid(d: Dict[str, Any]) -> Optional[int]:
            if not isinstance(d, dict):
                return None
            if "ros__parameters" in d and isinstance(d["ros__parameters"], dict):
                params = d["ros__parameters"]
                if "mid_pos" in params:
                    try:
                        return int(params["mid_pos"])
                    except Exception:
                        pass
            for vv in d.values():
                if isinstance(vv, dict):
                    res = deep_find_mid(vv)
                    if res is not None:
                        return res
            return None

        return deep_find_mid(data)

    def deg2pos(self, angle_deg: float) -> int:
        return int(round(self.mid_pos + (-angle_deg) * 4096.0 / 360.0))

    def pos2rad(self, ticks: int) -> float:
        return -((float(ticks) - float(self.mid_pos)) * (2.0 * math.pi / 4096.0))

    @staticmethod
    def _is_seq(x: Any) -> bool:
        return isinstance(x, (list, tuple, bytes, bytearray))

    @staticmethod
    def _u8(x: int) -> int:
        return x & 0xFF

    def read_servo_ticks(self) -> Optional[int]:
        candidates = [
            "ReadPos", "ReadNowPos", "ReadPosition", "ReadPosEx", "ReadPositionEx",
            "PresentPosition", "ReadPresentPosition"
        ]
        for name in candidates:
            f = getattr(self.servo, name, None)
            if callable(f):
                try:
                    res = f(self.servo_id)
                except TypeError:
                    res = None
                except Exception:
                    res = None
                if isinstance(res, (int, float)):
                    val = int(res)
                    if val >= 0:
                        return val
                if isinstance(res, tuple) and len(res) > 0 and isinstance(res[0], (int, float)):
                    return int(res[0])
                if isinstance(res, dict):
                    for k in ("Pos", "pos", "position", "Position", "present", "Present"):
                        v = res.get(k)
                        if isinstance(v, (int, float)):
                            return int(v)
        for name in ("Read", "ReadData", "read", "ReadDataEx"):
            f = getattr(self.servo, name, None)
            if callable(f):
                for args in ((self.servo_id, 0x38, 2), (self.servo_id, 0x38, 0x02)):
                    try:
                        data = f(*args)
                    except Exception:
                        data = None
                    if data is None:
                        continue
                    seq: Optional[Iterable[int]] = None
                    if self._is_seq(data):
                        seq = data
                    elif isinstance(data, dict):
                        for k in ("data", "params", "Param"):
                            v = data.get(k)
                            if self._is_seq(v):
                                seq = v
                                break
                    if seq is not None:
                        arr = list(seq)
                        if len(arr) >= 2:
                            L, H = int(arr[0]) & 0xFF, int(arr[1]) & 0xFF
                            return (H << 8) | L
        return None

    def sweep_loop(self):
        period = max(0.001, 1.0 / self.update_rate)
        try:
            while rclpy.ok() and self.running:
                target_pos = self.deg2pos(self.angle)
                wrote = False
                for name in ("WritePosEx", "WritePos", "RegWritePosEx", "RegWritePos"):
                    f = getattr(self.servo, name, None)
                    if callable(f):
                        try:
                            f(self.servo_id, int(target_pos), int(self.speed), int(self.acc))
                            wrote = True
                            break
                        except TypeError:
                            try:
                                f(self.servo_id, int(target_pos), int(self.acc), int(self.speed))
                                wrote = True
                                break
                            except Exception:
                                pass
                        except Exception:
                            pass
                time.sleep(period)
                js = JointState()
                now = self.get_clock().now()
                if self.feedback_stamp_delay_ms > 0.0:
                    js.header.stamp = (now - Duration(seconds=self.feedback_stamp_delay_ms / 1000.0)).to_msg()
                else:
                    js.header.stamp = now.to_msg()
                js.name = ['servo_joint']
                if self.publish_commanded:
                    js.position = [math.radians(self.angle)]
                else:
                    ticks = self.read_servo_ticks()
                    if ticks is not None:
                        js.position = [self.pos2rad(ticks)]
                    else:
                        js.position = [math.radians(self.angle)]
                self.joint_pub.publish(js)
                if self.angle <= self.min_deg:
                    self.angle = self.min_deg
                    self.direction = 1.0
                elif self.angle >= self.max_deg:
                    self.angle = self.max_deg
                    self.direction = -1.0
                self.angle += self.direction * self.get_parameter("delta_deg").get_parameter_value().double_value
        except Exception:
            pass

    def destroy_node(self):
        self.running = False
        try:
            if self.servo:
                try:
                    for name in ("WritePosEx", "WritePos"):
                        f = getattr(self.servo, name, None)
                        if callable(f):
                            f(self.servo_id, self.deg2pos(0.0), self.speed, self.acc)
                            break
                except Exception:
                    pass
                time.sleep(0.3)
                for attr in ("WriteEnable", "EnableTorque", "Unload", "TorqueEnable"):
                    f = getattr(self.servo, attr, None)
                    if callable(f):
                        try:
                            f(self.servo_id, 0)
                            break
                        except Exception:
                            pass
        finally:
            try:
                if self.port and hasattr(self.port, "closePort"):
                    self.port.closePort()
            except Exception:
                pass
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
