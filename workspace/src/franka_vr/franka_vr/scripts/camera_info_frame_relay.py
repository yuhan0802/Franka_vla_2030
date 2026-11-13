#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class CameraInfoFrameRelay(Node):
    def __init__(self):
        super().__init__("camera_info_frame_relay")
        input_topic = self.declare_parameter("input_topic", "camera_info_driver").value
        output_topic = self.declare_parameter("output_topic", "camera_info").value
        self.frame_id = self.declare_parameter("frame_id", "camera_optical_frame").value
        # image_proc/rectify subscribes to CameraInfo with reliable QoS.  The
        # camera driver's input is sensor-data/best-effort, but forwarding the
        # corrected message with the same profile makes the output
        # incompatible with rectify and prevents image_rect from being
        # produced.  Keep the input permissive and publish corrected metadata
        # reliably.
        self.publisher = self.create_publisher(
            CameraInfo, output_topic, 10
        )
        self.subscription = self.create_subscription(
            CameraInfo, input_topic, self.callback, qos_profile_sensor_data
        )

    def callback(self, message: CameraInfo):
        corrected = deepcopy(message)
        corrected.header.frame_id = self.frame_id
        self.publisher.publish(corrected)


def main():
    rclpy.init()
    node = CameraInfoFrameRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
