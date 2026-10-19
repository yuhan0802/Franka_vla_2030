#!/usr/bin/env python3
"""pi0.5-DROID receding-horizon adapter for the dedicated direct executor.

Dry-run never creates a JointJog publisher. Execute requires the direct-flow
confirmation token; velocity safety and trajectory publication remain in the
100 Hz C++ executor.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import functools
import json
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
from control_msgs.msg import JointJog
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from franka_msgs.action import Grasp, Move
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pi05_direct_velocity_safety import (  # noqa: E402
    ACTION_SHAPE, JOINT_NAMES, RecedingHorizonBuffer, decode_droid_action,
    validate_action_chunk,
)


def _pack(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return {b"__ndarray__": True, b"data": value.tobytes(), b"dtype": str(value.dtype), b"shape": value.shape}
    raise TypeError(type(value))


def _unpack(value: Dict[Any, Any]) -> Any:
    if value.get(b"__ndarray__"):
        return np.ndarray(buffer=value[b"data"], dtype=np.dtype(value[b"dtype"]), shape=value[b"shape"])
    return value


PACK = functools.partial(msgpack.packb, default=_pack, use_bin_type=True)
UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack, raw=False)


class PolicyClient:
    def __init__(self, host: str, port: int) -> None:
        self.url = f"ws://{host}:{port}"
        self.lock = threading.Lock()
        self.websocket = None
        self.metadata = self._connect()
        metadata_text = json.dumps(self.metadata, default=str).lower()
        for key in ("action_dim", "action_dimension"):
            if key in self.metadata and int(self.metadata[key]) != 8:
                raise RuntimeError(f"server metadata {key}={self.metadata[key]}, expected 8")
        if "action_shape" in self.metadata and tuple(self.metadata["action_shape"]) != ACTION_SHAPE:
            raise RuntimeError(f"server action_shape={self.metadata['action_shape']}, expected {ACTION_SHAPE}")
        if "port" in self.metadata and int(self.metadata["port"]) != port:
            raise RuntimeError(f"server metadata port={self.metadata['port']}, connected port={port}")
        if "pi0" not in metadata_text and "policy" in self.metadata:
            raise RuntimeError(f"unexpected policy metadata: {self.metadata}")

    def _connect(self) -> Dict[str, Any]:
        ws = websockets.sync.client.connect(
            self.url, compression=None, open_timeout=10, close_timeout=2, max_size=None, proxy=None
        )
        metadata = UNPACK(ws.recv(timeout=10))
        if not isinstance(metadata, dict):
            ws.close(); raise RuntimeError("policy metadata is not a mapping")
        self.websocket = ws
        return metadata

    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        payload = PACK(observation)
        with self.lock:
            try:
                self.websocket.send(payload); response = self.websocket.recv(timeout=60)
            except websockets.exceptions.ConnectionClosed:
                self.metadata = self._connect()
                self.websocket.send(payload); response = self.websocket.recv(timeout=60)
        result = UNPACK(response)
        actions = result.get("actions", result.get(b"actions")) if isinstance(result, dict) else None
        if actions is None: raise ValueError("policy response has no actions field")
        return validate_action_chunk(actions)

    def close(self) -> None:
        if self.websocket is not None: self.websocket.close(); self.websocket = None


def closedness_from_width(width: float) -> float:
    return float(np.clip(1.0 - width / 0.08, 0, 1))


class DirectAdapter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pi05_direct_policy_adapter")
        self.args = args
        self.bridge = CvBridge()
        self.images: Dict[str, np.ndarray] = {}; self.image_times: Dict[str, float] = {}
        self.joints: Dict[str, float] = {}; self.joint_time = 0.0
        self.fingers: Dict[str, float] = {}; self.gripper_time = 0.0
        self.queue = RecedingHorizonBuffer(args.execute_horizon)
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="pi05-direct")
        self.future: Optional[concurrent.futures.Future] = None; self.inference_started = 0.0
        self.policy = PolicyClient(args.server_host, args.server_port)
        self.publisher = None
        self.move_client = None; self.grasp_client = None
        self.executed = 0; self.last_publish = 0.0; self.need_fresh_after = 0.0
        self.fault = False; self.finished = False; self.gripper_busy = False
        self.last_gripper_closed: Optional[bool] = None; self.pending_gripper_closed: Optional[bool] = None
        self.log_file = open(args.jsonl_log, "a", buffering=1) if args.jsonl_log else None
        for key, topic in (("left", args.left_image_topic), ("wrist", args.wrist_image_topic)):
            self.create_subscription(Image, topic, functools.partial(self.image_cb, key), qos_profile_sensor_data)
        self.create_subscription(JointState, "/franka/joint_states", self.joint_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.joint_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, "/fr3_gripper/franka_gripper/joint_states", self.joint_cb, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/pi05/direct_executor/status", self.executor_status_cb, 10)
        if args.execute:
            self.publisher = self.create_publisher(JointJog, "/pi05/direct_joint_velocity_cmd", 10)
            if args.enable_gripper:
                self.move_client = ActionClient(self, Move, "/fr3_gripper/franka_gripper/move")
                self.grasp_client = ActionClient(self, Grasp, "/fr3_gripper/franka_gripper/grasp")
        self.create_timer(0.02, self.policy_tick)
        self.create_timer(1.0 / args.action_hz, self.action_tick)
        self.get_logger().warning(
            f"mode={'EXECUTE' if args.execute else 'DRY-RUN'} server={args.server_host}:{args.server_port} "
            f"shape={ACTION_SHAPE} horizon={args.execute_horizon} gripper={args.enable_gripper} metadata={self.policy.metadata}")
        if not args.execute: self.get_logger().warning("dry-run: JointJog publisher NOT created")

    def arm(self) -> None:
        if not self.args.execute: return
        deadline = time.monotonic() + 15
        while self.count_subscribers("/pi05/direct_joint_velocity_cmd") != 1 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
        if self.count_subscribers("/pi05/direct_joint_velocity_cmd") != 1:
            raise RuntimeError("expected exactly one direct executor command subscriber")
        if self.args.enable_gripper:
            if not self.move_client.wait_for_server(timeout_sec=10) or not self.grasp_client.wait_for_server(timeout_sec=10):
                raise RuntimeError("gripper action server unavailable")
            deadline = time.monotonic() + 5
            while len(self.fingers) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.1)
            if len(self.fingers) < 2:
                raise RuntimeError("complete gripper joint feedback unavailable")
            width = sum(self.fingers[name] for name in ("fr3_finger_joint1", "fr3_finger_joint2"))
            self.last_gripper_closed = closedness_from_width(width) >= self.args.gripper_close_threshold
            self.get_logger().info(
                f"initialized gripper state from feedback: width={width:.4f}m "
                f"closed={self.last_gripper_closed}")

    def image_cb(self, key: str, message: Image) -> None:
        try:
            image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(message, "bgr8"), cv2.COLOR_BGR2RGB)
            if image.shape[:2] != (480, 640): image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
            self.images[key] = np.ascontiguousarray(image, dtype=np.uint8); self.image_times[key] = time.monotonic()
        except Exception as error:
            self.get_logger().warning(f"cannot decode {key} image: {error}", throttle_duration_sec=2)

    def joint_cb(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position)); now = time.monotonic()
        if all(name in values for name in JOINT_NAMES):
            self.joints = {name: float(values[name]) for name in JOINT_NAMES}; self.joint_time = now
        for name in ("fr3_finger_joint1", "fr3_finger_joint2"):
            if name in values: self.fingers[name] = float(values[name]); self.gripper_time = now

    def executor_status_cb(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "pi05_direct_velocity_executor" and status.level == DiagnosticStatus.ERROR:
                self.latch(f"direct executor fault: {status.message}")
                return

    def observation(self) -> Optional[Dict[str, Any]]:
        now = time.monotonic()
        if set(self.images) != {"left", "wrist"} or len(self.joints) != 7:
            return None
        stamps = [self.image_times["left"], self.image_times["wrist"], self.joint_time]
        if self.args.enable_gripper:
            if not self.fingers: return None
            stamps.append(self.gripper_time)
        if min(stamps) <= self.need_fresh_after or now - min(stamps) > self.args.max_observation_age:
            return None
        if abs(self.image_times["left"] - self.image_times["wrist"]) > self.args.max_camera_skew:
            return None
        if max(abs(self.joint_time - self.image_times[key]) for key in ("left", "wrist")) > self.args.max_state_image_skew:
            return None
        if self.fingers:
            width = sum(self.fingers.get(name, 0.0) for name in ("fr3_finger_joint1", "fr3_finger_joint2"))
        else: width = .08
        return {"observation/exterior_image_1_left": self.images["left"].copy(),
                "observation/wrist_image_left": self.images["wrist"].copy(),
                "observation/joint_position": np.array([self.joints[name] for name in JOINT_NAMES], np.float32),
                "observation/gripper_position": np.array([closedness_from_width(width)], np.float32),
                "prompt": self.args.instruction}

    def policy_tick(self) -> None:
        if self.finished or self.fault or self.gripper_busy: return
        now = time.monotonic()
        if self.future is not None:
            if not self.future.done():
                if now - self.inference_started > self.args.max_policy_latency: self.latch("policy latency timeout")
                return
            future, self.future = self.future, None
            try: chunk = future.result()
            except Exception as error: self.latch(f"policy inference failed: {error}"); return
            latency = now - self.inference_started
            if self.args.execute:
                # Replacement, never append. Limit the last chunk to the remaining action budget.
                self.queue.replace(chunk)
            else:
                decoded = np.vstack([decode_droid_action(row)[0] for row in chunk[:self.args.execute_horizon]])
                self.log("dry_run", chunk=chunk.tolist(), decoded_qdot_rad_s=decoded.tolist(),
                         policy_latency=latency, robot_command_sent=False)
                self.get_logger().info(f"dry-run validated {chunk.shape}; decoded peak={np.max(np.abs(decoded)):.4f} rad/s")
                self.finished = True
            return
        if self.args.execute and (len(self.queue) or self.executed >= self.args.max_actions): return
        observation = self.observation()
        if observation is not None:
            self.inference_started = now; self.future = self.pool.submit(self.policy.infer, observation)

    def action_tick(self) -> None:
        if not self.args.execute or self.finished or self.fault or self.gripper_busy: return
        if self.executed >= self.args.max_actions:
            self.stop_command(); self.log("complete", actions_executed=self.executed, stop_requested=True)
            self.finished = True
            self.get_logger().warning("maximum action count reached; stop requested")
            return
        if not len(self.queue):
            # This callback is itself the next 15 Hz boundary after the last
            # action. Send the explicit stop unconditionally: a timer callback
            # can arrive a fraction early, and waiting for another tick would
            # exceed the executor's strict 120 ms nonzero-command watchdog.
            if self.last_publish:
                self.stop_command()
                self.log("chunk_stop", actions_executed=self.executed, stop_requested=True)
                self.last_publish = 0.0
            return
        action = self.queue.pop(); velocity, closedness = decode_droid_action(action)
        command = JointJog(); command.header.stamp = self.get_clock().now().to_msg()
        command.joint_names = list(JOINT_NAMES); command.velocities = velocity.tolist(); command.duration = 1 / self.args.action_hz
        self.publisher.publish(command); self.executed += 1; self.last_publish = time.monotonic()
        if not len(self.queue):
            # The next chunk must be based on observations captured after this
            # final action has had its full policy interval to execute.
            self.need_fresh_after = max(
                self.need_fresh_after, self.last_publish + 1 / self.args.action_hz)
        self.log("action", action_index=self.executed, policy_action_droid=action.tolist(),
                 decoded_qdot_rad_s=velocity.tolist(), gripper_closedness=closedness)
        if self.args.enable_gripper: self.maybe_gripper(closedness)

    def maybe_gripper(self, closedness: float) -> None:
        close = closedness >= self.args.gripper_close_threshold if self.last_gripper_closed is not True else closedness > self.args.gripper_open_threshold
        if close == self.last_gripper_closed: return
        self.queue.clear_for_gripper(); self.stop_command()
        self.need_fresh_after = max(self.need_fresh_after, time.monotonic()); self.gripper_busy = True
        self.pending_gripper_closed = close
        if close:
            goal = Grasp.Goal(); goal.width = (1 - closedness) * .08; goal.speed = self.args.gripper_speed
            goal.force = self.args.gripper_force; goal.epsilon.inner = .01; goal.epsilon.outer = .01
            future = self.grasp_client.send_goal_async(goal)
        else:
            goal = Move.Goal(); goal.width = (1 - closedness) * .08; goal.speed = self.args.gripper_speed
            future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.gripper_goal)

    def gripper_goal(self, future) -> None:
        try: handle = future.result()
        except Exception as error: self.latch(f"gripper request failed: {error}"); return
        if not handle.accepted: self.latch("gripper goal rejected"); return
        handle.get_result_async().add_done_callback(self.gripper_result)

    def gripper_result(self, future) -> None:
        try: result = future.result().result
        except Exception as error: self.latch(f"gripper action failed: {error}"); return
        if not result.success: self.latch(f"gripper action reported failure: {result.error}"); return
        self.last_gripper_closed = self.pending_gripper_closed; self.gripper_busy = False
        self.need_fresh_after = time.monotonic()  # Forces new observation and inference; old chunk was cleared.

    def stop_command(self) -> None:
        if self.publisher is None: return
        command = JointJog(); command.header.stamp = self.get_clock().now().to_msg()
        command.joint_names = list(JOINT_NAMES); command.velocities = [0.0] * 7; command.duration = 1 / self.args.action_hz
        self.publisher.publish(command)

    def latch(self, reason: str) -> None:
        if self.fault: return
        self.fault = True; self.queue.clear_for_gripper(); self.stop_command()
        self.get_logger().error(f"FAULT LATCHED: {reason}; restart required")

    def log(self, event: str, **fields: Any) -> None:
        if self.log_file:
            self.log_file.write(json.dumps({"monotonic_s": time.monotonic(), "event": event, **fields}) + "\n")

    def destroy_node(self):
        if self.args.execute:
            for _ in range(4): self.stop_command(); time.sleep(.01)
        self.pool.shutdown(wait=False, cancel_futures=True); self.policy.close()
        if self.log_file: self.log_file.close()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", default="10.24.116.66"); parser.add_argument("--server-port", type=int, default=8094)
    parser.add_argument("--instruction", default="put the red test tube into the red basket")
    parser.add_argument("--left-image-topic", default="/camera_left/image_rect")
    parser.add_argument("--wrist-image-topic", default="/camera/camera_front/color/image_raw")
    parser.add_argument("--action-hz", type=float, default=15.0); parser.add_argument("--execute-horizon", type=int, default=4)
    parser.add_argument("--max-actions", type=int, default=60); parser.add_argument("--max-policy-latency", type=float, default=2.0)
    parser.add_argument("--max-observation-age", type=float, default=.2); parser.add_argument("--max-camera-skew", type=float, default=.05)
    parser.add_argument("--max-state-image-skew", type=float, default=.05); parser.add_argument("--enable-gripper", action="store_true")
    parser.add_argument("--gripper-open-threshold", type=float, default=.4); parser.add_argument("--gripper-close-threshold", type=float, default=.6)
    parser.add_argument("--gripper-speed", type=float, default=.04); parser.add_argument("--gripper-force", type=float, default=10.)
    parser.add_argument("--jsonl-log", default=""); parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-real-robot", default="")
    args = parser.parse_args()
    if not 1 <= args.execute_horizon <= ACTION_SHAPE[0]:
        parser.error(f"execute-horizon must be in [1, {ACTION_SHAPE[0]}]")
    if not 1 <= args.max_actions <= 2000: parser.error("max-actions must be in [1, 2000]")
    if not 0 < args.gripper_speed <= .05: parser.error("gripper-speed must be in (0, 0.05]")
    if not 1 <= args.gripper_force <= 30: parser.error("gripper-force must be in [1, 30] N")
    if args.execute and args.confirm_real_robot != "DIRECT_JOINT_VELOCITY_PI05": parser.error("execute requires exact direct-flow confirmation token")
    return args


def main() -> None:
    args = parse_args(); rclpy.init(); node = DirectAdapter(args)
    try:
        node.arm()
        while rclpy.ok() and not node.finished and not node.fault: rclpy.spin_once(node, timeout_sec=.1)
    finally: node.destroy_node(); rclpy.shutdown()
    if node.fault:
        raise SystemExit(1)


if __name__ == "__main__": main()
