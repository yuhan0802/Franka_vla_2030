#!/usr/bin/env python3
"""Short, model-free acceptance pulse for the direct velocity interface."""

import argparse
import json
import math
import time

from control_msgs.msg import JointJog
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
SPEED = 0.02
PULSE_SECONDS = 0.5


class Commission(Node):
    def __init__(self):
        super().__init__("pi05_direct_velocity_commission")
        self.publisher = self.create_publisher(JointJog, "/pi05/direct_joint_velocity_cmd", 10)
        self.create_subscription(JointState, "/franka/joint_states", self.on_joint, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/pi05/direct_executor/status", self.on_status, 10)
        self.feedback = None
        self.feedback_at = 0.0
        self.status = None
        self.status_at = 0.0
        self.initial_position = None
        self.peak_speed = [0.0] * 7
        self.samples = 0

    def on_joint(self, message):
        index = {name: i for i, name in enumerate(message.name)}
        if not all(name in index for name in JOINTS):
            return
        if len(message.velocity) < len(message.name):
            return
        position = [message.position[index[name]] for name in JOINTS]
        velocity = [message.velocity[index[name]] for name in JOINTS]
        if not all(math.isfinite(value) for value in position + velocity):
            return
        self.feedback = (position, velocity)
        self.feedback_at = time.monotonic()
        self.samples += 1
        self.peak_speed = [max(old, abs(new)) for old, new in zip(self.peak_speed, velocity)]

    def on_status(self, message):
        for entry in message.status:
            if entry.name == "pi05_direct_velocity_executor":
                self.status = (entry.level, entry.message)
                self.status_at = time.monotonic()

    def check(self):
        now = time.monotonic()
        if self.status is None or now - self.status_at > 0.3:
            raise RuntimeError("executor status unavailable or stale")
        if self.status != (DiagnosticStatus.OK, "armed"):
            raise RuntimeError(f"executor not healthy: {self.status}")
        if self.feedback is None or now - self.feedback_at > 0.1:
            raise RuntimeError("joint feedback unavailable or stale")
        if self.count_subscribers("/pi05/direct_joint_velocity_cmd") != 1:
            raise RuntimeError("expected exactly one executor command subscriber")

    def send(self, speed):
        command = JointJog()
        command.header.stamp = self.get_clock().now().to_msg()
        command.joint_names = JOINTS
        command.velocities = [speed] + [0.0] * 6
        command.duration = 1.0 / 25.0
        self.publisher.publish(command)

    def hold(self, speed, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.check()
            self.send(speed)
            time.sleep(0.03)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json-log", required=True)
    parser.add_argument("--fake", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = Commission()
    started = time.monotonic()
    result = {"test": "single_joint_velocity_commission", "joint": JOINTS[0],
              "requested_velocity_rad_s": SPEED, "pulse_seconds": PULSE_SECONDS,
              "robot_command_sent": False, "passed": False}
    try:
        while time.monotonic() - started < 10:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                node.check()
                break
            except RuntimeError:
                pass
        node.check()
        node.initial_position = list(node.feedback[0])
        if max(abs(value) for value in node.feedback[1]) > 0.01:
            raise RuntimeError("robot was not stationary before pulse")
        node.hold(0.0, 0.3)
        result["robot_command_sent"] = True
        node.hold(SPEED, PULSE_SECONDS)
        node.hold(0.0, 0.7)
        displacement = [end - start for end, start in zip(node.feedback[0], node.initial_position)]
        result.update(peak_feedback_velocity_rad_s=node.peak_speed,
                      displacement_rad=displacement, feedback_samples=node.samples)
        if node.peak_speed[0] < 0.001:
            raise RuntimeError("joint 1 did not respond measurably")
        if max(node.peak_speed) > 0.05:
            raise RuntimeError("unexpected actual joint speed above 0.05 rad/s")
        if not args.fake and not (0.0005 < displacement[0] < 0.02):
            raise RuntimeError("real joint 1 displacement inconsistent with short pulse")
        if not args.fake and max(abs(value) for value in displacement[1:]) > 0.005:
            raise RuntimeError("unexpected movement of another joint")
        if max(abs(value) for value in node.feedback[1]) > 0.01:
            raise RuntimeError("robot did not stop after pulse")
        result["passed"] = True
    except Exception as error:
        result["error"] = str(error)
    finally:
        # An unstamped zero is always accepted by the executor, even after a fault.
        for _ in range(8):
            node.send(0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        with open(args.json_log, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2)
            stream.write("\n")
        print(json.dumps(result, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
