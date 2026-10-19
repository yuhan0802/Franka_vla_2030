#!/usr/bin/env python3
"""Safe ROS 2 bridge for the remote Franka Qwen/ActionCodec policy service.

The model server is deliberately kept off the FCI control network.  This node
only sends observations over WebSocket and, unless ``--execute`` is supplied,
prints validated policy actions.  In execute mode the existing local MoveIt
Servo target service remains the sole motion controller.
"""

import argparse
import concurrent.futures
import copy
import functools
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import msgpack
import numpy as np
import yaml
import websockets.exceptions
import websockets.sync.client

import rclpy
from cv_bridge import CvBridge
from franka_msgs.action import Move
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener

from franka_vr.srv import SetTargetPose


WORLD_FRAME = "world"
TIP_FRAME = "fr3_link8"
EXPECTED_MODEL_FAMILY = "qwen_vl_franka_actioncodec_future_state"


def _pack_array(value: Any) -> Any:
    """Use exactly the NumPy MessagePack representation used by mac_29."""
    if isinstance(value, (np.ndarray, np.generic)) and value.dtype.kind in ("V", "O", "c"):
        raise ValueError(f"Unsupported dtype: {value.dtype}")
    if isinstance(value, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": value.tobytes(),
            b"dtype": value.dtype.str,
            b"shape": value.shape,
        }
    if isinstance(value, np.generic):
        return {b"__npgeneric__": True, b"data": value.item(), b"dtype": value.dtype.str}
    return value


def _unpack_array(value: Dict[Any, Any]) -> Any:
    if b"__ndarray__" in value:
        return np.ndarray(buffer=value[b"data"], dtype=np.dtype(value[b"dtype"]), shape=value[b"shape"])
    if b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value


_PACKER = functools.partial(msgpack.Packer, default=_pack_array)
_UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack_array)


class PolicyClient:
    """Small dependency-only client compatible with WebsocketPolicyServer."""

    def __init__(self, host: str, port: int) -> None:
        self._uri = f"ws://{host}:{port}"
        self._ws = None
        self._lock = threading.Lock()
        self.metadata = self._connect()

    def _connect(self) -> Dict[str, Any]:
        connection = websockets.sync.client.connect(
            # 10.24.116.29 is a private robot-lab network.  Do not let a
            # shell HTTP(S)_PROXY setting route its WebSocket handshake to an
            # external proxy (which otherwise times out before metadata).
            self._uri, compression=None, max_size=None, open_timeout=5, close_timeout=2, proxy=None
        )
        metadata = _UNPACK(connection.recv(timeout=5))
        self._ws = connection
        return metadata

    def act(self, observation: Dict[str, Any]) -> np.ndarray:
        # A policy connection processes exactly one request at a time.  The
        # executor below also ensures this, and the lock makes reconnect safe.
        with self._lock:
            payload = _PACKER().pack(observation)
            try:
                self._ws.send(payload)
                response = self._ws.recv(timeout=30)
            except websockets.exceptions.ConnectionClosed:
                self.metadata = self._connect()
                self._ws.send(payload)
                response = self._ws.recv(timeout=30)
            if isinstance(response, str):
                raise RuntimeError(f"Policy server error:\n{response}")
            result = _UNPACK(response)
            action = np.asarray(result["action"], dtype=np.float64)
            if action.shape != (7,) or not np.isfinite(action).all():
                raise ValueError(f"Expected one finite 7-D policy action, got {action!r}")
            return action

    def close(self) -> None:
        if self._ws is not None:
            self._ws.close()
            self._ws = None


def validate_server_metadata(metadata: Dict[str, Any]) -> None:
    """Prevent a live bridge from silently talking to another user's model."""
    if metadata.get("model_family") != EXPECTED_MODEL_FAMILY:
        raise RuntimeError(
            f"Wrong policy server: expected {EXPECTED_MODEL_FAMILY!r}, got {metadata!r}. "
            "Choose the dedicated VLA port or start run_vla_server.sh."
        )


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Compose XYZW quaternions, returning a normalized XYZW quaternion."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    value = np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )
    return value / np.linalg.norm(value)


def _quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rotation_vector_to_quaternion(rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    axis = rotation_vector / angle
    return np.r_[axis * math.sin(angle / 2.0), math.cos(angle / 2.0)]


class Calibration:
    """Rigid transform from local ROS ``world`` into LIBERO training frame."""

    def __init__(self, path: Path) -> None:
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except FileNotFoundError as error:
            raise RuntimeError(f"Calibration file does not exist: {path}") from error
        self.calibrated = bool(data.get("calibrated", False))
        matrix = np.asarray(data.get("libero_from_world"), dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise RuntimeError("libero_from_world must be a finite 4 x 4 homogeneous matrix")
        if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
            raise RuntimeError("libero_from_world last row must be [0, 0, 0, 1]")
        self.matrix = matrix
        self.rotation = matrix[:3, :3]
        if not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-5) or np.linalg.det(self.rotation) < 0.999:
            raise RuntimeError("libero_from_world rotation must be a proper orthonormal rotation")
        self.translation = matrix[:3, 3]
        bounds = data.get("workspace_bounds_world", {})
        self.minimum = np.asarray(bounds.get("min", [np.nan, np.nan, np.nan]), dtype=np.float64)
        self.maximum = np.asarray(bounds.get("max", [np.nan, np.nan, np.nan]), dtype=np.float64)

    def require_execution_ready(self) -> None:
        if not self.calibrated:
            raise RuntimeError("Execution requires calibrated: true in the calibration file")
        if self.minimum.shape != (3,) or self.maximum.shape != (3,) or not np.isfinite(self.minimum).all() or not np.isfinite(self.maximum).all() or np.any(self.maximum <= self.minimum):
            raise RuntimeError("Execution requires finite workspace_bounds_world min/max in the calibration file")


class VLAControlBridge(Node):
    def __init__(self, args: argparse.Namespace, calibration: Calibration) -> None:
        super().__init__("vla_policy_bridge")
        self._args = args
        self._calibration = calibration
        self._bridge = CvBridge()
        self._external_image: Optional[np.ndarray] = None
        self._wrist_image: Optional[np.ndarray] = None
        self._joints: Dict[str, float] = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._target_client = self.create_client(SetTargetPose, "/set_target_pose")
        self._gripper_client = ActionClient(self, Move, "/fr3_gripper/franka_gripper/move")
        self._last_gripper_open: Optional[bool] = None
        self._client = PolicyClient(args.server_host, args.server_port)
        validate_server_metadata(self._client.metadata)
        self._pool = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="vla_inference")
        self._future: Optional[concurrent.futures.Future] = None
        self._last_dispatch = 0.0
        self._sent_actions = 0
        self._stop_timer = None

        self.create_subscription(Image, args.external_image_topic, self._external_cb, 2)
        self.create_subscription(Image, args.wrist_image_topic, self._wrist_cb, 2)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(JointState, "/franka/joint_states", self._joint_cb, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f"Connected to {args.server_host}:{args.server_port}; metadata={self._client.metadata}")
        self.get_logger().info(
            f"mode={'EXECUTE' if args.execute else 'DRY RUN'}, policy_hz={args.policy_hz:.2f}, "
            f"external={args.external_image_topic}, wrist={args.wrist_image_topic}"
        )
        if not calibration.calibrated:
            self.get_logger().warning("Using uncalibrated identity/example transform; execution is locked")

    def destroy_node(self):
        self._pool.shutdown(wait=False, cancel_futures=True)
        self._client.close()
        return super().destroy_node()

    def _image(self, message: Image) -> np.ndarray:
        image = self._bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
        return np.ascontiguousarray(image, dtype=np.uint8)

    def _external_cb(self, message: Image) -> None:
        try:
            self._external_image = self._image(message)
        except Exception as error:
            self.get_logger().warning(f"Cannot decode external RGB image: {error}")

    def _wrist_cb(self, message: Image) -> None:
        try:
            self._wrist_image = self._image(message)
        except Exception as error:
            self.get_logger().warning(f"Cannot decode wrist RGB image: {error}")

    def _joint_cb(self, message: JointState) -> None:
        self._joints.update({name: position for name, position in zip(message.name, message.position)})

    def _current_pose(self) -> Optional[tuple[np.ndarray, np.ndarray]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                WORLD_FRAME, TIP_FRAME, Time(), timeout=Duration(seconds=0.05)
            ).transform
        except Exception as error:
            self.get_logger().info(f"Waiting for {WORLD_FRAME} -> {TIP_FRAME}: {error}", throttle_duration_sec=2.0)
            return None
        position = np.array([transform.translation.x, transform.translation.y, transform.translation.z], dtype=np.float64)
        quaternion = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w], dtype=np.float64)
        if not np.isfinite(position).all() or np.linalg.norm(quaternion) < 1e-8:
            return None
        return position, quaternion / np.linalg.norm(quaternion)

    def _observation(self) -> Optional[Dict[str, Any]]:
        if self._external_image is None or self._wrist_image is None:
            self.get_logger().info("Waiting for external and wrist RGB images", throttle_duration_sec=2.0)
            return None
        pose = self._current_pose()
        if pose is None:
            return None
        gripper = self._joints.get("fr3_finger_joint1")
        if gripper is None:
            self.get_logger().info("Waiting for fr3_finger_joint1 feedback", throttle_duration_sec=2.0)
            return None
        position_world, quaternion_world = pose
        position_libero = self._calibration.rotation @ position_world + self._calibration.translation
        # q(libero<-eef) = q(libero<-world) * q(world<-eef).  Deriving q from
        # the calibrated matrix avoids a dependency on tf_transformations.
        rotation = self._calibration.rotation
        trace = np.trace(rotation)
        if trace > 0.0:
            scale = 2.0 * math.sqrt(trace + 1.0)
            q_lw = np.array([(rotation[2, 1] - rotation[1, 2]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale, (rotation[1, 0] - rotation[0, 1]) / scale, 0.25 * scale])
        else:
            # This branch is intentionally explicit: calibration can include
            # 180 degree frame changes, where the trace-only formula fails.
            index = int(np.argmax(np.diag(rotation)))
            next_index, last_index = (index + 1) % 3, (index + 2) % 3
            scale = 2.0 * math.sqrt(1.0 + rotation[index, index] - rotation[next_index, next_index] - rotation[last_index, last_index])
            q_lw = np.zeros(4)
            q_lw[index] = 0.25 * scale
            q_lw[3] = (rotation[last_index, next_index] - rotation[next_index, last_index]) / scale
            q_lw[next_index] = (rotation[next_index, index] + rotation[index, next_index]) / scale
            q_lw[last_index] = (rotation[last_index, index] + rotation[index, last_index]) / scale
        quaternion_libero = _quaternion_multiply(q_lw, quaternion_world)
        finger = float(gripper)
        return {
            "external_camera_image": copy.copy(self._external_image),
            "wrist_camera_image": copy.copy(self._wrist_image),
            "eef_pos_libero_frame": position_libero.astype(np.float32),
            "eef_quat_xyzw_libero_frame": quaternion_libero.astype(np.float32),
            "gripper_qpos": np.array([finger, finger], dtype=np.float32),
            # The server falls back to its configured default for legacy
            # clients, but this bridge always supplies the task selected on
            # the local control computer.
            "instruction": self._args.instruction,
        }

    def _tick(self) -> None:
        if self._future is not None:
            if not self._future.done():
                return
            completed, self._future = self._future, None
            try:
                self._handle_action(completed.result())
            except Exception as error:
                self.get_logger().error(f"Policy request failed: {error}")
            return
        now = time.monotonic()
        if now - self._last_dispatch < 1.0 / self._args.policy_hz:
            return
        observation = self._observation()
        if observation is None:
            return
        self._last_dispatch = now
        self._future = self._pool.submit(self._client.act, observation)

    def _handle_action(self, action: np.ndarray) -> None:
        raw = action.copy()
        action[:3] = np.clip(action[:3], -self._args.max_translation_step, self._args.max_translation_step)
        action[3:6] = np.clip(action[3:6], -self._args.max_rotation_step, self._args.max_rotation_step)
        self.get_logger().info(
            "VLA action raw=%s guarded=%s" % (
                np.array2string(raw, precision=4), np.array2string(action, precision=4)
            )
        )
        if not self._args.execute:
            return
        if self._args.max_actions and self._sent_actions >= self._args.max_actions:
            return
        pose = self._current_pose()
        if pose is None:
            return
        position_world, quaternion_world = pose
        # The server's deltas are in the calibrated LIBERO world frame.
        # Rotate them back to ROS world before creating a Cartesian target.
        target_position = position_world + self._calibration.rotation.T @ action[:3]
        if np.any(target_position < self._calibration.minimum) or np.any(target_position > self._calibration.maximum):
            self.get_logger().error(f"Rejected action outside calibrated workspace: {target_position}")
            return
        rotation_world = self._calibration.rotation.T @ action[3:6]
        target_quaternion = _quaternion_multiply(_rotation_vector_to_quaternion(rotation_world), quaternion_world)
        if not self._send_target(target_position, target_quaternion):
            return
        self._sent_actions += 1
        if self._args.enable_gripper:
            self._send_gripper(open_gripper=bool(action[6] < 0.0))
        if self._args.max_actions and self._sent_actions >= self._args.max_actions:
            self.get_logger().warning(
                f"Reached max-actions={self._args.max_actions}; stopping after this guarded target"
            )
            # Keep spinning briefly so the asynchronous service request is
            # transmitted and its result can be observed before shutdown.
            self._stop_timer = self.create_timer(1.0, self._stop_after_action_limit)

    def _stop_after_action_limit(self) -> None:
        self._stop_timer.cancel()
        self.get_logger().info("Single-step VLA test complete; no further targets will be sent")
        rclpy.shutdown()

    def _send_target(self, position: np.ndarray, quaternion: np.ndarray) -> bool:
        if not self._target_client.service_is_ready():
            self.get_logger().error("/set_target_pose is unavailable; action was not executed")
            return False
        request = SetTargetPose.Request()
        request.target_pose = PoseStamped()
        request.target_pose.header.stamp = self.get_clock().now().to_msg()
        request.target_pose.header.frame_id = WORLD_FRAME
        request.target_pose.pose.position.x, request.target_pose.pose.position.y, request.target_pose.pose.position.z = position
        request.target_pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        request.stop = False
        future = self._target_client.call_async(request)
        future.add_done_callback(self._target_response)
        return True

    def _target_response(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"Servo rejected VLA target: {response.message}")
        except Exception as error:
            self.get_logger().error(f"Servo target request failed: {error}")

    def _send_gripper(self, open_gripper: bool) -> None:
        if self._last_gripper_open is open_gripper or not self._gripper_client.server_is_ready():
            return
        goal = Move.Goal()
        goal.width, goal.speed = (0.08, 0.05) if open_gripper else (0.0, 0.05)
        self._last_gripper_open = open_gripper
        self._gripper_client.send_goal_async(goal)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", default="10.24.116.29")
    parser.add_argument("--server-port", type=int, default=8092)
    parser.add_argument("--external-image-topic", default="/camera_left/image_rect")
    parser.add_argument("--wrist-image-topic", default="/camera/camera_front/color/image_raw")
    parser.add_argument("--instruction", default="pick up the red block", help="Task instruction sent to the remote policy")
    parser.add_argument("--calibration-config", type=Path, required=True)
    parser.add_argument("--policy-hz", type=float, default=2.0)
    parser.add_argument("--max-translation-step", type=float, default=0.005)
    parser.add_argument("--max-rotation-step", type=float, default=0.04)
    parser.add_argument("--max-actions", type=int, default=0, help="Stop after this many dispatched targets (0 means unlimited)")
    parser.add_argument("--execute", action="store_true", help="Allow guarded Cartesian targets to reach local MoveIt Servo")
    parser.add_argument("--enable-gripper", action="store_true", help="Also execute model gripper actions (off by default)")
    parser.add_argument("--probe", action="store_true", help="Verify WebSocket metadata then exit; does not subscribe or move")
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if args.policy_hz <= 0 or args.max_translation_step <= 0 or args.max_rotation_step <= 0 or args.max_actions < 0:
        parser.error("policy rate and action limits must be positive")
    return args


def main() -> None:
    rclpy.init()
    node = None
    try:
        args = parse_args()
        calibration = Calibration(args.calibration_config)
        if args.execute:
            calibration.require_execution_ready()
        if args.probe:
            client = PolicyClient(args.server_host, args.server_port)
            validate_server_metadata(client.metadata)
            print(f"VLA policy server reachable: {client.metadata}", flush=True)
            client.close()
            return
        node = VLAControlBridge(args, calibration)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
