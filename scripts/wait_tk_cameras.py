#!/usr/bin/env python3
"""Wait for real camera frames only; no policy inference or robot commands."""
import argparse
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("tk_camera_readiness")
    topics = ("/camera_left/image_raw", "/camera/camera_front/color/image_raw")
    samples = {topic: [] for topic in topics}
    subscriptions = []
    for topic in topics:
        def received(message, topic=topic):
            if message.width > 0 and message.height > 0 and message.data:
                samples[topic].append(time.monotonic())
                samples[topic] = samples[topic][-3:]
        subscriptions.append(node.create_subscription(Image, topic, received, qos_profile_sensor_data))
    started = time.monotonic()
    warned_at = started - 5
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            missing = [topic for topic, stamps in samples.items() if len(stamps) < 3 or now - stamps[-1] > 0.5]
            if not missing:
                print("Both camera streams ready; robot control may now start.", flush=True)
                return
            if now - warned_at >= 5:
                print("Waiting for fresh camera frames: " + ", ".join(missing), flush=True)
                warned_at = now
        raise RuntimeError("camera startup timed out: " + ", ".join(missing))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
