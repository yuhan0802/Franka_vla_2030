#!/usr/bin/env python3
"""Record the EEF policy's exterior and wrist camera topics until SIGINT, as MP4 or as JPEG frames.

Both streams are written at one fixed rate (default 15 Hz): a frame is kept only when at
least one period has passed since the last kept frame, so a 30 Hz camera is halved and
video time matches wall time.  Frames are dropped before decoding, so a low rate (e.g. 1 Hz
JPEG logging) costs almost no CPU.  A per-frame ROS timestamp CSV sits next to each stream.
On exit the mp4v files are re-encoded to H.264 with ffmpeg when it is available.
"""
import argparse
import csv
import shutil
import signal
import subprocess
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, output: Path, streams, jpeg: bool):
        super().__init__("eef_camera_recorder")
        self.bridge, self.output, self.jpeg, self.streams = CvBridge(), output, jpeg, {}
        for name, topic, fps in streams:
            self.streams[name] = dict(topic=topic, fps=fps, writer=None, count=0, last=None,
                                      times=open(output / f"{name}_timestamps.csv", "w", newline=""))
            csv.writer(self.streams[name]["times"]).writerow(["frame", "stamp_s"])
            self.create_subscription(Image, topic, self.callback(name), qos_profile_sensor_data)
            self.get_logger().info(f"recording {topic} -> {name}.mp4 at {fps} fps")

    def callback(self, name):
        def record(message):
            stream = self.streams[name]
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            # Keep a frame once a full period has elapsed (10% slack absorbs jitter).
            if stream["last"] is not None and stamp - stream["last"] < 0.9 / stream["fps"]:
                return
            stream["last"] = stamp
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            if self.jpeg:
                (self.output / name).mkdir(exist_ok=True)
                cv2.imwrite(str(self.output / name / f"{stream['count']:06d}_{stamp:.3f}.jpg"), frame)
            elif stream["writer"] is None:
                height, width = frame.shape[:2]
                stream["writer"] = cv2.VideoWriter(str(self.output / f"{name}_raw.mp4"),
                                                   cv2.VideoWriter_fourcc(*"mp4v"), stream["fps"], (width, height))
            if not self.jpeg:
                stream["writer"].write(frame)
            csv.writer(stream["times"]).writerow([stream["count"], f"{stamp:.6f}"])
            stream["count"] += 1
        return record

    def finalize(self):
        for name, stream in self.streams.items():
            stream["times"].close()
            if self.jpeg:
                print(f"{name}: {stream['count']} frames -> {self.output / name}", flush=True)
                continue
            if stream["writer"] is None:
                print(f"{name}: no frames received from {stream['topic']}", flush=True)
                continue
            stream["writer"].release()
            raw, final = self.output / f"{name}_raw.mp4", self.output / f"{name}.mp4"
            if shutil.which("ffmpeg") and subprocess.run(
                    ["ffmpeg", "-y", "-loglevel", "error", "-i", str(raw), "-c:v", "libx264",
                     "-pix_fmt", "yuv420p", "-crf", "18", str(final)]).returncode == 0:
                raw.unlink()
            else:
                raw.rename(final)
            print(f"{name}: {stream['count']} frames -> {final}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--exterior-topic", default="/camera_left/image_rect")
    parser.add_argument("--wrist-topic", default="/camera/camera_front/color/image_raw")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--jpeg", action="store_true", help="save JPEG frames instead of MP4")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Recorder(args.output, [("exterior", args.exterior_topic, args.fps),
                                  ("wrist", args.wrist_topic, args.fps)], args.jpeg)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.finalize()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
