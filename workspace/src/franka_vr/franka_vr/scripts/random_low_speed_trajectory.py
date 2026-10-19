#!/usr/bin/env python3
"""Execute a bounded, smooth Cartesian trajectory through the Servo target API."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from franka_vr.srv import SetTargetPose


class LowSpeedTrajectory(Node):
    def __init__(self, duration: float, start_delay: float):
        super().__init__("random_low_speed_trajectory")
        self.duration, self.start_delay = duration, start_delay
        self.current = None
        self.client = self.create_client(SetTargetPose, "/set_target_pose")
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            PoseStamped, "/franka_robot_state_broadcaster/current_pose", self.pose_cb, 10
        )

    def pose_cb(self, message):
        self.current = message

    def publish_target(self, pose):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "fr3_manual_target"
        tf.transform.translation.x = pose.pose.position.x
        tf.transform.translation.y = pose.pose.position.y
        tf.transform.translation.z = pose.pose.position.z
        tf.transform.rotation = pose.pose.orientation
        self.broadcaster.sendTransform(tf)

    def send(self, pose, stop=False):
        request = SetTargetPose.Request()
        request.target_pose = pose
        request.target_pose.header.stamp = self.get_clock().now().to_msg()
        request.target_pose.header.frame_id = "world"
        request.stop = stop
        return self.client.call_async(request)

    def run(self):
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and (self.current is None or not self.client.service_is_ready()):
            if time.monotonic() >= deadline:
                raise RuntimeError("No current pose or /set_target_pose service; no target was sent")
            rclpy.spin_once(self, timeout_sec=0.1)
        origin = self.current
        self.get_logger().info(
            f"Trajectory armed at [{origin.pose.position.x:.3f}, {origin.pose.position.y:.3f}, {origin.pose.position.z:.3f}]; "
            f"starting in {self.start_delay:.1f}s"
        )
        until = time.monotonic() + self.start_delay
        while rclpy.ok() and time.monotonic() < until:
            self.publish_target(origin)
            rclpy.spin_once(self, timeout_sec=0.05)

        start = time.monotonic()
        period = 0.2  # 5 Hz target updates; Servo interpolates between them.
        while rclpy.ok() and time.monotonic() - start < self.duration:
            t = time.monotonic() - start
            pose = PoseStamped()
            pose.header.frame_id = "world"
            # Closed, smooth Lissajous path. Absolute displacements are bounded
            # at 8 mm, 8 mm and 5 mm respectively; orientation stays fixed.
            pose.pose.position.x = origin.pose.position.x + 0.008 * math.sin(0.55 * t)
            pose.pose.position.y = origin.pose.position.y + 0.008 * math.sin(0.73 * t + 0.6)
            pose.pose.position.z = origin.pose.position.z + 0.005 * math.sin(0.41 * t + 1.2)
            pose.pose.orientation = origin.pose.orientation
            self.publish_target(pose)
            self.send(pose)
            rclpy.spin_once(self, timeout_sec=period)

        self.send(origin, stop=True)
        self.get_logger().info("Trajectory complete; target tracking stopped")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--start-delay", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = LowSpeedTrajectory(args.duration, args.start_delay)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
