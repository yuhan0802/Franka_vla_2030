#!/usr/bin/env python3
"""One-shot, read-only FrankaVR sensor and policy probe.

This node only subscribes to ROS state.  It deliberately does not import the
SetTargetPose service type and cannot send a robot or gripper command.
"""

import argparse
import functools
import math
import sys
import time

import msgpack
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener
import websockets.exceptions
import websockets.sync.client


EXPECTED_MODEL = "qwen3vl2b_tube_pick_simulation_state_first_step6000"
EXPECTED_FORMAT = "franka_vr_front_left_right_and_joint_state_v1"
JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))
GRIPPER_JOINTS = ("fr3_finger_joint1", "fr3_finger_joint", "fr3_finger_joint2")


def _pack(value):
    if isinstance(value, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": value.tobytes(),
            b"dtype": value.dtype.str,
            b"shape": value.shape,
        }
    if isinstance(value, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": value.item(),
            b"dtype": value.dtype.str,
        }
    return value


def _unpack(value):
    if b"__ndarray__" in value:
        return np.ndarray(
            buffer=value[b"data"],
            dtype=np.dtype(value[b"dtype"]),
            shape=value[b"shape"],
        )
    if b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value


PACK = functools.partial(msgpack.Packer, default=_pack)
UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack)


class SensorCollector(Node):
    def __init__(self, args):
        super().__init__("franka_vr_read_only_detector")
        self.args = args
        self.bridge = CvBridge()
        self.images = {}
        self.image_info = {}
        self.joints = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for key, topic in (
            ("front", args.front_topic),
            ("left", args.left_topic),
            ("right", args.right_topic),
        ):
            self.create_subscription(
                Image,
                topic,
                lambda message, key=key, topic=topic: self._image(key, topic, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(JointState, args.joint_topic, self._joint, 10)
        if args.joint_topic != "/franka/joint_states":
            self.create_subscription(JointState, "/franka/joint_states", self._joint, 10)
        self.create_subscription(JointState, "/fr3_gripper/franka_gripper/joint_states", self._joint, 10)

    def _image(self, key, topic, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            image = np.ascontiguousarray(image, dtype=np.uint8)
            if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
                raise ValueError(f"decoded shape is {image.shape}")
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.images[key] = image
            self.image_info[key] = (
                topic,
                stamp,
                message.encoding,
                int(message.width),
                int(message.height),
            )
        except Exception as error:  # cv_bridge reports the useful encoding detail.
            self.get_logger().error(f"Cannot decode {key} image from {topic}: {error}")

    def _joint(self, message):
        self.joints.update(zip(message.name, message.position))

    def ready(self):
        return set(self.images) == {"front", "left", "right"} and all(
            name in self.joints and math.isfinite(self.joints[name]) for name in JOINTS
        ) and any(name in self.joints and math.isfinite(self.joints[name]) for name in GRIPPER_JOINTS)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read three images + 8-D state and make one non-executing policy request."
    )
    parser.add_argument("--host", default="10.24.116.29")
    parser.add_argument("--port", type=int, default=8093)
    parser.add_argument("--front-topic", default="/camera/camera_front/color/image_raw")
    parser.add_argument("--left-topic", default="/camera_left/image_raw")
    parser.add_argument("--right-topic", default="/camera_right/image_raw")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--tf-parent", default="world")
    parser.add_argument("--tf-child", default="fr3_link8")
    parser.add_argument("--instruction", default="put the red test tube into the basket")
    parser.add_argument("--sensor-timeout", type=float, default=12.0)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--response-timeout", type=float, default=60.0)
    parser.add_argument("--max-image-age", type=float, default=3.0)
    parser.add_argument(
        "--sensors-only",
        action="store_true",
        help="validate images, joints, and updating TF without contacting the model",
    )
    return parser.parse_args()


def collect(args):
    rclpy.init()
    node = SensorCollector(args)
    try:
        deadline = time.monotonic() + args.sensor_timeout
        while time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        now = node.get_clock().now().nanoseconds * 1e-9
        for key in sorted(node.image_info):
            topic, stamp, encoding, width, height = node.image_info[key]
            age = now - stamp if stamp > 0 else float("nan")
            if stamp <= 0 or age < -1.0 or age > args.max_image_age:
                raise RuntimeError(f"{key} image timestamp is stale/invalid: stamp={stamp}, age={age}")
            print(
                f"SENSOR_IMAGE key={key} topic={topic} stamp={stamp:.9f} "
                f"age={age:.3f}s encoding={encoding} size={width}x{height} rgb8=ok"
            )

        if not node.ready():
            missing_images = sorted({"front", "left", "right"} - set(node.images))
            missing_joints = [name for name in JOINTS if name not in node.joints]
            if not any(name in node.joints for name in GRIPPER_JOINTS):
                missing_joints.append("gripper (expected one of " + ", ".join(GRIPPER_JOINTS) + ")")
            raise RuntimeError(
                f"sensor timeout; missing_images={missing_images}, missing_joints={missing_joints}"
            )

        finger_name = next(name for name in GRIPPER_JOINTS if name in node.joints)
        finger_position = float(node.joints[finger_name])
        gripper_state = float(np.clip(finger_position / 0.08, 0.0, 1.0))
        joint_state = np.asarray([*(node.joints[name] for name in JOINTS), gripper_state], dtype=np.float32)
        if joint_state.shape != (8,) or not np.isfinite(joint_state).all():
            raise RuntimeError(f"invalid 8-D state: {joint_state!r}")
        print(f"SENSOR_JOINTS names={','.join(JOINTS)},{finger_name} gripper_position={finger_position:.6f} gripper_state={gripper_state:.6f} state_dtype={joint_state.dtype} state_shape={joint_state.shape}")

        tf_samples = []
        tf_deadline = time.monotonic() + args.sensor_timeout
        while time.monotonic() < tf_deadline and len(tf_samples) < 2:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = node.tf_buffer.lookup_transform(
                    args.tf_parent,
                    args.tf_child,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
            except Exception:
                continue
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if stamp > 0 and (not tf_samples or stamp > tf_samples[-1]):
                tf_samples.append(stamp)
                if len(tf_samples) < 2:
                    time.sleep(0.1)
        if len(tf_samples) != 2:
            raise RuntimeError(
                f"TF {args.tf_parent}->{args.tf_child} did not provide two updating timestamps"
            )
        print(
            f"SENSOR_TF parent={args.tf_parent} child={args.tf_child} "
            f"stamp1={tf_samples[0]:.9f} stamp2={tf_samples[1]:.9f} updating=ok"
        )
        observation = {
            "front_image": node.images["front"].copy(),
            "left_image": node.images["left"].copy(),
            "right_image": node.images["right"].copy(),
            "joint_state": joint_state,
            "instruction": args.instruction,
        }
        return observation
    finally:
        node.destroy_node()
        rclpy.shutdown()


def probe_model(args, observation):
    uri = f"ws://{args.host}:{args.port}"
    websocket = websockets.sync.client.connect(
        uri,
        compression=None,
        max_size=None,
        open_timeout=args.connect_timeout,
        close_timeout=2,
        proxy=None,
    )
    try:
        metadata = UNPACK(websocket.recv(timeout=args.connect_timeout))
        if not isinstance(metadata, dict):
            raise RuntimeError(f"invalid server metadata type: {type(metadata).__name__}")
        if metadata.get("model_family") != EXPECTED_MODEL:
            raise RuntimeError(f"wrong model_family: {metadata!r}")
        if metadata.get("observation_format") != EXPECTED_FORMAT:
            raise RuntimeError(f"wrong observation_format: {metadata!r}")
        if metadata.get("action_shape") != [15, 7]:
            raise RuntimeError(f"wrong metadata action_shape: {metadata!r}")
        print(
            f"MODEL_METADATA host={args.host} port={args.port} "
            f"model_family={metadata['model_family']} observation_format={metadata['observation_format']}"
        )

        websocket.send(PACK().pack(observation))
        response = websocket.recv(timeout=args.response_timeout)
        if isinstance(response, str):
            raise RuntimeError(f"server returned error text: {response}")
        unpacked = UNPACK(response)
        if not isinstance(unpacked, dict) or "action" not in unpacked:
            raise RuntimeError(f"response has no action field: {type(unpacked).__name__}")
        action = unpacked["action"]
        if not isinstance(action, np.ndarray):
            raise RuntimeError(f"action is not an ndarray: {type(action).__name__}")
        if action.dtype != np.dtype(np.float32):
            raise RuntimeError(f"action dtype must be float32, got {action.dtype}")
        if action.shape != (15, 7):
            raise RuntimeError(f"action shape must be (15, 7), got {action.shape}")
        if not np.isfinite(action).all():
            raise RuntimeError("action contains NaN or Inf")
        print(
            "MODEL_ACTION dtype=float32 shape=[15,7] finite=true "
            f"min={action.min():.7g} max={action.max():.7g} "
            f"first={np.array2string(action[0], precision=5, separator=',')}"
        )
    finally:
        websocket.close()


def main():
    args = parse_args()
    try:
        observation = collect(args)
        if not args.sensors_only:
            probe_model(args, observation)
    except (RuntimeError, ValueError, TimeoutError, OSError, websockets.exceptions.WebSocketException) as error:
        print(f"DETECT_ERROR {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(
        "READ_ONLY_SENSOR_PROBE_OK no robot action was sent"
        if args.sensors_only
        else "READ_ONLY_PROBE_OK no robot action was sent"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
