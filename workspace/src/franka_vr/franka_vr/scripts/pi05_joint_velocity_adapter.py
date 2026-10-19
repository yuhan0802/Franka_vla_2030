#!/usr/bin/env python3
"""Bridge pi0.5-DROID observations/actions to guarded MoveIt Servo JointJog.

Dry-run is the default and never creates a robot command publisher. Real motion
requires both ``--execute`` and the exact confirmation token documented by
``--help``. The server performs DROID normalization/denormalization. Execution
is serial: finish a chunk, stop, observe, infer, then execute the next chunk.
"""

from __future__ import annotations

import argparse
import collections
import concurrent.futures
import functools
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import msgpack
import numpy as np
import websockets.exceptions
import websockets.sync.client

import rclpy
from control_msgs.msg import JointJog
from cv_bridge import CvBridge
from franka_msgs.action import Grasp, Move
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pi05_joint_velocity_safety import (  # noqa: E402
    EXPECTED_ACTION_SHAPE,
    JOINT_NAMES,
    JointVelocityGuard,
    validate_action_chunk,
    gripper_closedness,
    gripper_target_width,
)


EXECUTION_CONFIRMATION = "JOINT_VELOCITY_PI05"
FATAL_SERVO_CODES = {
    ServoStatus.HALT_FOR_SINGULARITY,
    ServoStatus.HALT_FOR_COLLISION,
    ServoStatus.JOINT_BOUND,
}


def _pack_array(value: Any) -> Any:
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
    def __init__(self, host: str, port: int) -> None:
        self.uri = f"ws://{host}:{port}"
        self.websocket = None
        self.lock = threading.Lock()
        self.metadata = self._connect()

    def _connect(self) -> Dict[str, Any]:
        websocket = websockets.sync.client.connect(
            self.uri,
            compression=None,
            max_size=None,
            open_timeout=10,
            close_timeout=2,
            proxy=None,
        )
        metadata = _UNPACK(websocket.recv(timeout=10))
        self.websocket = websocket
        return metadata

    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        with self.lock:
            payload = _PACKER().pack(observation)
            try:
                self.websocket.send(payload)
                response = self.websocket.recv(timeout=60)
            except websockets.exceptions.ConnectionClosed:
                self.metadata = self._connect()
                self.websocket.send(payload)
                response = self.websocket.recv(timeout=60)
            if isinstance(response, str):
                raise RuntimeError(f"policy server error: {response}")
            result = _UNPACK(response)
            actions = result.get("actions", result.get(b"actions"))
            if actions is None:
                raise ValueError(f"policy response has no actions field: {result.keys()}")
            return validate_action_chunk(actions)

    def close(self) -> None:
        if self.websocket is not None:
            self.websocket.close()
            self.websocket = None


class Pi05JointVelocityAdapter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pi05_joint_velocity_adapter")
        self.args = args
        self.bridge = CvBridge()
        self.images: Dict[str, np.ndarray] = {}
        self.image_times: Dict[str, float] = {}
        self.joint_positions: Dict[str, float] = {}
        self.joint_time = 0.0
        self.gripper_positions: Dict[str, float] = {}
        self.gripper_time = 0.0
        self.actions = collections.deque()
        self.inference_pool = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="pi05")
        self.inference_future: Optional[concurrent.futures.Future] = None
        self.policy = PolicyClient(args.server_host, args.server_port)
        self.guard = JointVelocityGuard(
            max_velocity=args.max_joint_velocity,
            max_acceleration=args.max_joint_acceleration,
            joint_limit_margin=args.joint_limit_margin,
            action_units=args.action_units,
        )
        self.executed_actions = 0
        self.last_action_time = 0.0
        self.gap_halted = False
        self.last_gripper_closed: Optional[bool] = None
        self.last_gripper_target_width: Optional[float] = None
        self.pending_gripper_target_width: Optional[float] = None
        self.pending_gripper_closed: Optional[bool] = None
        self.gripper_busy = False
        self.fault_latched = False
        self.finished = False
        self.command_publisher = None
        self.switch_client = None
        self.gripper_move_client = None
        self.gripper_grasp_client = None

        self.create_subscription(Image, args.left_image_topic, functools.partial(self._image_cb, "left"), qos_profile_sensor_data)
        self.create_subscription(Image, args.wrist_image_topic, functools.partial(self._image_cb, "wrist"), qos_profile_sensor_data)
        self.create_subscription(JointState, "/franka/joint_states", self._joint_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, qos_profile_sensor_data)
        self.create_subscription(
            JointState,
            "/fr3_gripper/franka_gripper/joint_states",
            self._joint_cb,
            qos_profile_sensor_data,
        )

        if args.execute:
            self.command_publisher = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
            self.create_subscription(ServoStatus, "/servo_node/status", self._servo_status_cb, 10)
            self.switch_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
            self.gripper_move_client = ActionClient(self, Move, "/fr3_gripper/franka_gripper/move")
            self.gripper_grasp_client = ActionClient(self, Grasp, "/fr3_gripper/franka_gripper/grasp")

        self.create_timer(0.05, self._policy_tick)
        self.create_timer(1.0 / args.action_hz, self._action_tick)
        self.get_logger().info(
            f"connected to {args.server_host}:{args.server_port}; mode={'EXECUTE' if args.execute else 'DRY-RUN'}; "
            f"expected actions={EXPECTED_ACTION_SHAPE}; action_units={args.action_units}; "
            f"serial=True; image_resize=server; metadata={self.policy.metadata}"
        )
        if not args.execute:
            self.get_logger().warning("dry-run: no JointJog publisher exists and no robot motion command can be sent")

    def arm_servo(self) -> None:
        if not self.args.execute:
            return
        if not self.switch_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/servo_node/switch_command_type is unavailable")
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.JOINT_JOG
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError("MoveIt Servo refused JOINT_JOG command mode")
        if self.count_subscribers("/servo_node/delta_joint_cmds") != 1:
            raise RuntimeError("expected exactly one MoveIt Servo JointJog subscriber")
        if self.args.enable_gripper:
            if not self.gripper_move_client.wait_for_server(timeout_sec=15.0):
                raise RuntimeError("gripper move action server is unavailable")
            if not self.gripper_grasp_client.wait_for_server(timeout_sec=15.0):
                raise RuntimeError("gripper grasp action server is unavailable")
        self.get_logger().warning(
            f"REAL ROBOT ARMED: max_velocity={self.args.max_joint_velocity:.3f} rad/s, "
            f"max_acceleration={self.args.max_joint_acceleration:.3f} rad/s^2, max_actions={self.args.max_actions}"
        )

    def destroy_node(self):
        if self.args.execute and self.command_publisher is not None and rclpy.ok():
            self._publish_zero(repetitions=8)
        self.inference_pool.shutdown(wait=False, cancel_futures=True)
        self.policy.close()
        return super().destroy_node()

    def _image_cb(self, name: str, message: Image) -> None:
        try:
            # Match collection's color conversion and 640x480 frame preparation.
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            if image.shape[:2] != (480, 640):
                image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
            # Model-size resize/padding remains on the server, as in training.
            self.images[name] = np.ascontiguousarray(image, dtype=np.uint8)
            self.image_times[name] = time.monotonic()
        except Exception as error:
            self.get_logger().warning(f"cannot decode {name} image: {error}", throttle_duration_sec=2.0)

    def _joint_cb(self, message: JointState) -> None:
        now = time.monotonic()
        values = dict(zip(message.name, message.position))
        if any(name in values for name in JOINT_NAMES):
            self.joint_positions.update({name: float(values[name]) for name in JOINT_NAMES if name in values})
            self.joint_time = now
        for name in ("fr3_finger_joint1", "fr3_finger_joint2"):
            if name in values:
                self.gripper_positions[name] = float(values[name])
                self.gripper_time = now

    def _observation(self) -> Optional[Dict[str, Any]]:
        now = time.monotonic()
        missing_images = [name for name in ("left", "wrist") if name not in self.images]
        if missing_images or len(self.joint_positions) != 7 or not self.gripper_positions:
            self.get_logger().info(
                f"waiting for observations: images={missing_images}, joints={len(self.joint_positions)}/7, "
                f"gripper={'yes' if self.gripper_positions else 'no'}",
                throttle_duration_sec=2.0,
            )
            return None
        ages = [now - self.image_times[name] for name in ("left", "wrist")]
        ages.extend([now - self.joint_time, now - self.gripper_time])
        if max(ages) > self.args.max_observation_age:
            self.get_logger().warning(f"stale observation rejected; oldest age={max(ages):.3f}s", throttle_duration_sec=2.0)
            return None
        joints = np.array([self.joint_positions[name] for name in JOINT_NAMES], dtype=np.float32)
        finger1 = self.gripper_positions.get("fr3_finger_joint1")
        finger2 = self.gripper_positions.get("fr3_finger_joint2")
        width = (finger1 + finger2) if finger1 is not None and finger2 is not None else 2.0 * next(iter(self.gripper_positions.values()))
        width = float(np.clip(width, 0.0, 0.08))
        return {
            "observation/exterior_image_1_left": self.images["left"].copy(),
            "observation/wrist_image_left": self.images["wrist"].copy(),
            "observation/joint_position": joints,
            # prepare_franka_pi05_droid.py stores closedness, not metres.
            "observation/gripper_position": np.array([gripper_closedness(width)], dtype=np.float32),
            "prompt": self.args.instruction,
        }

    def _policy_tick(self) -> None:
        if self.finished or self.fault_latched:
            return
        if self.gripper_busy:
            return
        if self.inference_future is not None:
            if not self.inference_future.done():
                return
            future, self.inference_future = self.inference_future, None
            try:
                chunk = future.result()
            except Exception as error:
                self._latch_fault(f"policy inference failed: {error}")
                return
            self.get_logger().info(
                f"validated policy chunk {chunk.shape}; range=[{chunk.min():.4f}, {chunk.max():.4f}]"
            )
            if self.args.execute:
                remaining = self.args.max_actions - self.executed_actions - len(self.actions)
                self.actions.extend(chunk[:max(0, remaining)])
            else:
                guarded = JointVelocityGuard(
                    max_velocity=self.args.max_joint_velocity,
                    max_acceleration=self.args.max_joint_acceleration,
                    joint_limit_margin=self.args.joint_limit_margin,
                    action_units=self.args.action_units,
                )
                joints = [self.joint_positions[name] for name in JOINT_NAMES]
                peak = 0.0
                for action in chunk:
                    peak = max(peak, float(np.max(np.abs(guarded.apply(action, joints, 1.0 / self.args.action_hz).velocity))))
                self.get_logger().info(f"dry-run guarded peak={peak:.4f} rad/s; robot_command_sent=false")
                self.finished = True
            return
        if self.args.execute:
            if self.executed_actions >= self.args.max_actions:
                return
            if self.actions:
                return
            # A deque becomes empty immediately after publishing its last
            # action. Wait until that action's full 15 Hz interval has elapsed,
            # and until the action timer has commanded a stop.
            if self.last_action_time and (
                time.monotonic() < self.last_action_time + 1.0 / self.args.action_hz
                or not self.gap_halted
            ):
                return
            boundary = self.last_action_time + 1.0 / self.args.action_hz
            if self.last_action_time and (
                self.joint_time < boundary
                or any(self.image_times.get(name, 0.0) < boundary for name in ("left", "wrist"))
            ):
                return
        observation = self._observation()
        if observation is not None:
            self.inference_future = self.inference_pool.submit(self.policy.infer, observation)

    def _servo_status_cb(self, message: ServoStatus) -> None:
        if message.code in FATAL_SERVO_CODES:
            self._latch_fault(f"MoveIt Servo safety halt: code={message.code}, message={message.message}")

    def _latch_fault(self, reason: str) -> None:
        if self.fault_latched:
            return
        self.fault_latched = True
        self.finished = True
        self.gripper_busy = False
        self.actions.clear()
        self.guard.reset()
        self.get_logger().error(f"FAULT LATCHED: {reason}; restart is required")
        if self.args.execute:
            self._publish_zero(repetitions=8)

    def _action_tick(self) -> None:
        if not self.args.execute or self.finished or self.fault_latched:
            return
        now = time.monotonic()
        if self.gripper_busy:
            if not self.gap_halted:
                self._publish_zero(repetitions=2)
                self.guard.reset()
                self.gap_halted = True
            return
        if not self.actions:
            if self.executed_actions >= self.args.max_actions:
                if now - self.last_action_time >= 1.0 / self.args.action_hz:
                    self._finish_execution()
                return
            if self.last_action_time and not self.gap_halted and now - self.last_action_time >= 1.0 / self.args.action_hz:
                self._publish_zero(repetitions=2)
                self.guard.reset()
                self.gap_halted = True
                self.get_logger().info("chunk complete; stopped before acquiring observations for the next inference")
            return
        if now - self.joint_time > self.args.max_observation_age:
            self._latch_fault("joint feedback became stale")
            return
        action = self.actions.popleft()
        positions = np.array([self.joint_positions[name] for name in JOINT_NAMES], dtype=np.float64)
        try:
            result = self.guard.apply(action, positions, 1.0 / self.args.action_hz)
        except ValueError as error:
            self._latch_fault(str(error))
            return
        command = JointJog()
        command.header.stamp = self.get_clock().now().to_msg()
        command.joint_names = list(JOINT_NAMES)
        command.velocities = result.velocity.tolist()
        command.duration = 1.0 / self.args.action_hz
        self.command_publisher.publish(command)
        self.executed_actions += 1
        self.last_action_time = now
        self.gap_halted = False
        self.get_logger().info(
            f"action {self.executed_actions}/{self.args.max_actions}: "
            f"policy={np.array2string(action, precision=5)} "
            f"decoded_rad_s={np.array2string(result.raw_velocity, precision=5)} "
            f"guarded={np.array2string(result.velocity, precision=5)} "
            f"vel_clip={result.velocity_clipped} accel_clip={result.acceleration_clipped} "
            f"joint_limit_block={result.joint_limit_blocked}"
        )
        if self.args.enable_gripper:
            self._command_gripper(result.gripper_closedness)

    def _finish_execution(self) -> None:
        if self.finished:
            return
        self.finished = True
        self.actions.clear()
        self._publish_zero(repetitions=8)
        self.get_logger().warning(f"commissioning limit reached ({self.executed_actions} actions); commanded a smooth halt")

    def _publish_zero(self, repetitions: int) -> None:
        if self.command_publisher is None:
            return
        for _ in range(repetitions):
            command = JointJog()
            command.header.stamp = self.get_clock().now().to_msg()
            command.joint_names = list(JOINT_NAMES)
            command.velocities = [0.0] * 7
            command.duration = 1.0 / self.args.action_hz
            self.command_publisher.publish(command)
            time.sleep(0.01)

    def _command_gripper(self, closedness: float) -> None:
        target_width = gripper_target_width(closedness)
        if self.last_gripper_closed:
            close = closedness > self.args.gripper_open_threshold
        else:
            close = closedness >= self.args.gripper_close_threshold
        if (close == self.last_gripper_closed and self.last_gripper_target_width is not None
                and abs(target_width - self.last_gripper_target_width) < 0.002):
            return
        if close:
            if not self.gripper_grasp_client.server_is_ready():
                self._latch_fault("gripper grasp action server became unavailable")
                return
            goal = Grasp.Goal()
            goal.width = target_width
            goal.speed = self.args.gripper_speed
            goal.force = self.args.gripper_force
            goal.epsilon.inner = self.args.gripper_epsilon_inner
            goal.epsilon.outer = self.args.gripper_epsilon_outer
            future = self.gripper_grasp_client.send_goal_async(goal)
            self.get_logger().warning(
                f"gripper CLOSE requested: width={goal.width:.3f}m force={goal.force:.1f}N "
                f"closedness={closedness:.3f} "
                f"epsilon_inner={goal.epsilon.inner:.3f}m epsilon_outer={goal.epsilon.outer:.3f}m"
            )
        else:
            if not self.gripper_move_client.server_is_ready():
                self._latch_fault("gripper move action server became unavailable")
                return
            goal = Move.Goal()
            goal.width = target_width
            goal.speed = self.args.gripper_speed
            future = self.gripper_move_client.send_goal_async(goal)
            self.get_logger().info(f"gripper OPEN requested: closedness={closedness:.3f}")
        self.gripper_busy = True
        self.pending_gripper_closed = close
        self.pending_gripper_target_width = target_width
        future.add_done_callback(self._gripper_goal_response)

    def _gripper_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._latch_fault(f"gripper goal request failed: {error}")
            return
        if not goal_handle.accepted:
            self._latch_fault("gripper goal was rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result)

    def _gripper_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as error:
            self._latch_fault(f"gripper action failed: {error}")
            return
        if not result.success:
            if self.args.allow_unconfirmed_grasp and self.pending_gripper_closed is True and not result.error:
                self.get_logger().warning(
                    "gripper reported an unconfirmed grasp without a hardware error; "
                    "continuing because --allow-unconfirmed-grasp is enabled"
                )
                self._complete_gripper_command(confirmed=False)
                return
            self._latch_fault(f"gripper action reported failure: {result.error}")
            return
        self._complete_gripper_command(confirmed=True)

    def _complete_gripper_command(self, confirmed: bool) -> None:
        self.last_gripper_closed = self.pending_gripper_closed
        self.last_gripper_target_width = self.pending_gripper_target_width
        self.pending_gripper_target_width = None
        self.pending_gripper_closed = None
        self.gripper_busy = False
        self.guard.reset()
        self.gap_halted = True
        self.get_logger().warning(
            f"gripper {'CLOSED' if self.last_gripper_closed else 'OPENED'} "
            f"{'successfully' if confirmed else 'without confirmation'}; continuing current chunk"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", default="10.24.116.66")
    parser.add_argument("--server-port", type=int, default=8094)
    parser.add_argument("--instruction", default="put the red test tube into the red basket")
    parser.add_argument("--left-image-topic", default="/camera_left/image_rect")
    parser.add_argument("--right-image-topic", default="/camera_right/image_rect")
    parser.add_argument("--wrist-image-topic", default="/camera/camera_front/color/image_raw")
    parser.add_argument("--action-hz", type=float, default=15.0)
    parser.add_argument("--action-units", choices=["droid", "direct-rad-s"], default="droid",
                        help="droid inverts the verified training dataset velocity conversion; direct-rad-s is experimental")
    parser.add_argument("--max-observation-age", type=float, default=0.5)
    parser.add_argument("--max-joint-velocity", type=float, default=0.15)
    parser.add_argument("--max-joint-acceleration", type=float, default=0.5)
    parser.add_argument("--joint-limit-margin", type=float, default=0.15)
    parser.add_argument("--max-actions", type=int, default=16)
    parser.add_argument("--prefetch-actions", type=int, choices=[0], default=0,
                        help="compatibility option; only 0 is supported (strict serial execution)")
    parser.add_argument("--enable-gripper", action="store_true")
    parser.add_argument("--gripper-close-threshold", type=float, default=0.6)
    parser.add_argument("--gripper-open-threshold", type=float, default=0.4)
    parser.add_argument("--gripper-speed", type=float, default=0.02)
    parser.add_argument("--gripper-force", type=float, default=10.0)
    parser.add_argument("--gripper-grasp-width", type=float, default=0.0)
    parser.add_argument("--gripper-epsilon-inner", type=float, default=0.01)
    parser.add_argument("--gripper-epsilon-outer", type=float, default=0.03)
    parser.add_argument(
        "--allow-unconfirmed-grasp",
        action="store_true",
        help="continue only when grasp returns false without a hardware error",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--confirm-real-robot",
        default="",
        help=f"required with --execute; exact value: {EXECUTION_CONFIRMATION}",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if args.action_hz != 15.0:
        parser.error("pi0.5-DROID execution requires --action-hz 15")
    if not 0.1 <= args.max_observation_age <= 1.0:
        parser.error("--max-observation-age must be in [0.1, 1.0]")
    if not 1 <= args.max_actions <= 2000:
        parser.error("--max-actions must be in [1, 2000]")
    if not 0.0 < args.gripper_speed <= 0.05:
        parser.error("--gripper-speed must be in (0, 0.05]")
    if not 1.0 <= args.gripper_force <= 30.0:
        parser.error("--gripper-force must be in [1, 30] N")
    if not 0.0 <= args.gripper_grasp_width <= 0.08:
        parser.error("--gripper-grasp-width must be in [0, 0.08] m")
    if not 0.0 <= args.gripper_open_threshold < args.gripper_close_threshold <= 1.0:
        parser.error("gripper thresholds must satisfy 0 <= open < close <= 1")
    if args.execute and args.confirm_real_robot != EXECUTION_CONFIRMATION:
        parser.error(f"--execute requires --confirm-real-robot {EXECUTION_CONFIRMATION}")
    return args


def main() -> None:
    rclpy.init()
    node = None
    try:
        args = parse_args()
        node = Pi05JointVelocityAdapter(args)
        node.arm_servo()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
