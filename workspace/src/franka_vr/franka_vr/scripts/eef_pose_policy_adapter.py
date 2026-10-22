#!/usr/bin/env python3
"""Serial-chunk adapter for end-effector pose policies (15x7 EEF actions).

This is the EEF pose option next to pi05_direct_policy_adapter.py (joint
velocity). It never talks to the direct velocity executor: targets go to the
MoveIt Servo pose tracker (/set_target_pose, demo_franka_vr_vel), the same path
used for VR data collection. Dry-run creates no target or gripper client.
"""

from __future__ import annotations

import argparse
import functools
import json
import sys
import threading
import time
import concurrent.futures
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import msgpack
import numpy as np
import websockets.exceptions
import websockets.sync.client
import yaml
import rclpy
from cv_bridge import CvBridge
from franka_msgs.action import Grasp, Move
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener
from franka_vr.srv import SetTargetPose

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eef_pose_safety import (  # noqa: E402
    ACTION_REPRESENTATION, ACTION_SHAPE, ROTATION_EXECUTION, ROTVEC_FLIP_WARNING_RAD, STATE_PROTOCOL,
    ChunkTargetIntegrator, GripperHysteresis, axis_test_chunk, build_state, gripper_open_from_width,
    inside_workspace, physical_step, rotation_angle_between, validate_action_chunk,
)

CONFIRM_TOKEN = "EEF_POSE_TARGET"
FINGERS = ("fr3_finger_joint1", "fr3_finger_joint2")


def _pack(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return {b"__ndarray__": True, b"data": value.tobytes(), b"dtype": value.dtype.str, b"shape": value.shape}
    if isinstance(value, np.generic):
        return {b"__npgeneric__": True, b"data": value.item(), b"dtype": value.dtype.str}
    return value


def _unpack(value: Dict[Any, Any]) -> Any:
    if b"__ndarray__" in value:
        return np.ndarray(buffer=value[b"data"], dtype=np.dtype(value[b"dtype"]), shape=value[b"shape"])
    if b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value


PACK = functools.partial(msgpack.packb, default=_pack, use_bin_type=True)
UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack, raw=False)


class EefPolicyClient:
    """Websocket client that refuses any server not declaring the EEF protocol."""

    def __init__(self, host: str, port: int) -> None:
        self.url = f"ws://{host}:{port}"
        self.lock = threading.Lock()
        self.websocket = None
        self.metadata = self._connect()
        problems = []
        if self.metadata.get("state_protocol") != STATE_PROTOCOL:
            problems.append(f"state_protocol={self.metadata.get('state_protocol')!r}, expected {STATE_PROTOCOL!r}")
        if self.metadata.get("action_representation") != ACTION_REPRESENTATION:
            problems.append(f"action_representation={self.metadata.get('action_representation')!r}, "
                            f"expected {ACTION_REPRESENTATION!r}")
        if tuple(self.metadata.get("action_shape", ())) != ACTION_SHAPE:
            problems.append(f"action_shape={self.metadata.get('action_shape')!r}, expected {list(ACTION_SHAPE)}")
        if float(self.metadata.get("action_hz", 0)) != 15.0:
            problems.append(f"action_hz={self.metadata.get('action_hz')!r}, expected 15")
        if self.metadata.get("gripper_convention") != "0_closed_1_open":
            problems.append(f"gripper_convention={self.metadata.get('gripper_convention')!r}, expected '0_closed_1_open'")
        if self.metadata.get("rotation_execution") not in ROTATION_EXECUTION:
            problems.append(f"rotation_execution={self.metadata.get('rotation_execution')!r}, "
                            f"expected one of {sorted(ROTATION_EXECUTION)}")
        if problems:
            self.close()
            raise RuntimeError("policy server is not an EEF pose server: " + "; ".join(problems))
        self.rotation_frame = ROTATION_EXECUTION[self.metadata["rotation_execution"]]

    def _connect(self) -> Dict[str, Any]:
        ws = websockets.sync.client.connect(
            self.url, compression=None, open_timeout=10, close_timeout=2, max_size=None, proxy=None)
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
        if isinstance(response, str): raise RuntimeError(response)
        result = UNPACK(response)
        if not isinstance(result, dict): raise ValueError("policy response is not a mapping")
        actions = result.get("action", result.get("actions"))
        if actions is None: raise ValueError("policy response has no action field")
        return validate_action_chunk(actions)

    def close(self) -> None:
        if self.websocket is not None: self.websocket.close(); self.websocket = None


class EefAdapter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("eef_pose_policy_adapter")
        self.args = args
        self.bridge = CvBridge()
        self.images: Dict[str, np.ndarray] = {}; self.image_times: Dict[str, float] = {}
        self.fingers: Dict[str, float] = {}; self.gripper_time = 0.0
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        bounds = yaml.safe_load(open(args.bounds_config))["workspace_bounds_world"]
        self.minimum, self.maximum = np.asarray(bounds["min"], float), np.asarray(bounds["max"], float)
        if self.minimum.shape != (3,) or self.maximum.shape != (3,) or np.any(self.maximum <= self.minimum):
            raise RuntimeError("finite world workspace bounds are required")
        if args.test_axis:
            self.policy = None; rotation_frame = args.rotation_frame
            self.metadata: Dict[str, Any] = {"source": "axis-test", "axis": args.test_axis}
        else:
            self.policy = EefPolicyClient(args.server_host, args.server_port)
            rotation_frame = self.policy.rotation_frame
            if args.rotation_frame and args.rotation_frame != rotation_frame:
                raise RuntimeError(f"--rotation-frame {args.rotation_frame} contradicts server "
                                   f"rotation_execution={self.policy.metadata['rotation_execution']}")
            self.metadata = self.policy.metadata
        self.integrator = ChunkTargetIntegrator(rotation_frame)
        self.hysteresis = GripperHysteresis(args.gripper_open_threshold, args.gripper_close_threshold)
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="eef-policy")
        self.future: Optional[concurrent.futures.Future] = None; self.inference_started = 0.0
        self.chunk: list = []; self.chunk_index = 0; self.chunks_done = 0
        self.executed = 0; self.next_action_at = 0.0; self.need_fresh_after = 0.0
        self.fault = False; self.finished = False
        self.gripper_open: Optional[bool] = None; self.pending_gripper_open: Optional[bool] = None
        self.commanded_gripper_open = True
        self.target_client = None; self.move_client = None; self.grasp_client = None
        self.log_file = open(args.jsonl_log, "a", buffering=1) if args.jsonl_log else None
        for key, topic in (("exterior", args.exterior_image_topic), ("wrist", args.wrist_image_topic)):
            self.create_subscription(Image, topic, functools.partial(self.image_cb, key), qos_profile_sensor_data)
        self.create_subscription(JointState, "/fr3_gripper/franka_gripper/joint_states", self.joint_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.joint_cb, qos_profile_sensor_data)
        if args.execute:
            self.target_client = self.create_client(SetTargetPose, "/set_target_pose")
            if args.enable_gripper:
                self.move_client = ActionClient(self, Move, "/fr3_gripper/franka_gripper/move")
                self.grasp_client = ActionClient(self, Grasp, "/fr3_gripper/franka_gripper/grasp")
        self.create_timer(0.01, self.tick)
        self.get_logger().warning(
            f"mode={'EXECUTE' if args.execute else 'DRY-RUN'} source={'axis-test ' + args.test_axis if args.test_axis else f'{args.server_host}:{args.server_port}'} "
            f"rotation_frame={rotation_frame} chunk_start={args.chunk_start} gripper={args.enable_gripper} "
            f"max_step={args.max_translation:.4f}m/{args.max_rotation:.4f}rad metadata={self.metadata}")
        if not args.execute: self.get_logger().warning("dry-run: /set_target_pose and gripper clients NOT created")
        self.log("start", args=vars(args), metadata=self.metadata, rotation_frame=rotation_frame)

    # ----------------------------------------------------------------- inputs
    def image_cb(self, key: str, message: Image) -> None:
        try:
            image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(message, "bgr8"), cv2.COLOR_BGR2RGB)
            if image.shape[:2] != (480, 640): image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
            self.images[key] = np.ascontiguousarray(image, dtype=np.uint8); self.image_times[key] = time.monotonic()
        except Exception as error:
            self.get_logger().warning(f"cannot decode {key} image: {error}", throttle_duration_sec=2)

    def joint_cb(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        for name in FINGERS:
            if name in values: self.fingers[name] = float(values[name]); self.gripper_time = time.monotonic()

    def gripper_width(self) -> Optional[float]:
        if any(name not in self.fingers for name in FINGERS): return None
        # Same total-opening definition as lerobot_episode_recorder.gripper_width().
        return sum(abs(self.fingers[name]) for name in FINGERS)

    def pose(self) -> Optional[tuple]:
        try:
            # Zero timeout: never block the single-threaded executor; staleness is checked below.
            transform = self.tf.lookup_transform("world", "fr3_link8", Time())
        except Exception as error:
            self.get_logger().info(f"waiting for world -> fr3_link8: {error}", throttle_duration_sec=2.0); return None
        stamp = Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > self.args.max_tf_age:
            self.get_logger().warning(f"world -> fr3_link8 is {age:.3f}s old", throttle_duration_sec=2.0); return None
        t, q = transform.transform.translation, transform.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])

    def gripper_state_value(self) -> Optional[float]:
        if self.args.gripper_state_source == "command":
            return 1.0 if self.commanded_gripper_open else 0.0
        width = self.gripper_width()
        if width is None: return None
        return gripper_open_from_width(width, self.args.gripper_state_closed_width, self.args.gripper_state_open_width)

    def observation(self) -> Optional[Dict[str, Any]]:
        now = time.monotonic()
        if set(self.images) != {"exterior", "wrist"}: return None
        stamps = [self.image_times["exterior"], self.image_times["wrist"]]
        if self.args.gripper_state_source == "measured":
            if self.gripper_width() is None: return None
            stamps.append(self.gripper_time)
        if min(stamps) <= self.need_fresh_after or now - min(stamps) > self.args.max_observation_age: return None
        if abs(self.image_times["exterior"] - self.image_times["wrist"]) > self.args.max_camera_skew: return None
        pose = self.pose()
        gripper = self.gripper_state_value()
        if pose is None or gripper is None: return None
        state = build_state(pose[0], pose[1], gripper)
        rotvec_norm = float(np.linalg.norm(state[3:6]))
        if rotvec_norm > ROTVEC_FLIP_WARNING_RAD:
            # |rotvec| near pi: tiny tilts flip its sign. Training used the same
            # conversion, but out-of-distribution flips must be visible in logs.
            self.get_logger().warning(f"state rotvec |r|={rotvec_norm:.3f} rad is near the pi flip boundary",
                                      throttle_duration_sec=5.0)
        observation = {"eef_state": state, "exterior_image": self.images["exterior"].copy(),
                       "wrist_image": self.images["wrist"].copy(), "instruction": self.args.instruction}
        self.log("observation", eef_state=state.tolist(), rotvec_norm=rotvec_norm,
                 gripper_width=self.gripper_width(), quaternion_xyzw=pose[1].tolist())
        return observation

    # ----------------------------------------------------------------- loop
    def tick(self) -> None:
        if self.finished or self.fault: return
        now = time.monotonic()
        if self.future is not None:
            if not self.future.done():
                if now - self.inference_started > self.args.max_policy_latency: self.latch("policy latency timeout")
                return
            future, self.future = self.future, None
            try: chunk = future.result()
            except Exception as error: self.latch(f"policy inference failed: {error}"); return
            self.start_chunk(chunk, policy_latency=now - self.inference_started)
            return
        if self.chunk:
            if now >= self.next_action_at: self.execute_step(now)
            return
        if self.executed >= self.args.max_actions or self.chunks_done >= self.args.max_chunks:
            self.finish("budget reached"); return
        if self.args.test_axis:
            if now >= self.need_fresh_after:
                self.start_chunk(axis_test_chunk(self.args.test_axis, self.args.test_magnitude, self.args.test_steps,
                                                 1.0 if self.commanded_gripper_open else 0.0), policy_latency=0.0)
            return
        observation = self.observation()
        if observation is not None:
            self.inference_started = now; self.future = self.pool.submit(self.policy.infer, observation)

    def start_chunk(self, chunk: np.ndarray, policy_latency: float) -> None:
        pose = self.pose()
        if pose is None: self.latch("end-effector pose unavailable at chunk start"); return
        start_position, start_quaternion = pose
        if (self.args.chunk_start == "last_target" and self.integrator.position is not None
                and np.linalg.norm(self.integrator.position - start_position) <= self.args.max_target_lead
                and rotation_angle_between(self.integrator.quaternion, start_quaternion) <= self.args.max_target_lead_rotation):
            start_position, start_quaternion = self.integrator.position, self.integrator.quaternion
        self.integrator.start(start_position, start_quaternion)
        budget = self.args.max_actions - self.executed
        self.chunk = [row.copy() for row in chunk[:min(self.args.execute_horizon, budget)]]
        self.chunk_index = 0; self.next_action_at = time.monotonic()
        self.log("chunk", chunk=np.asarray(chunk).tolist(), policy_latency=policy_latency,
                 start_position=start_position.tolist(), start_quaternion_xyzw=start_quaternion.tolist(),
                 robot_command_sent=bool(self.args.execute))
        self.get_logger().info(f"chunk {self.chunks_done + 1}: {len(self.chunk)} steps, latency={policy_latency:.3f}s, "
                               f"first={np.array2string(np.asarray(chunk[0]), precision=4)}")

    def execute_step(self, now: float) -> None:
        action = self.chunk.pop(0); self.chunk_index += 1
        try: step = physical_step(action, self.args.max_translation, self.args.max_rotation)
        except ValueError as error: self.latch(f"invalid action: {error}"); return
        target_position, target_quaternion = self.integrator.step(step.delta_position, step.delta_rotvec)
        record = dict(chunk_step=self.chunk_index, action=np.asarray(action).tolist(),
                      delta_position=step.delta_position.tolist(), delta_rotvec=step.delta_rotvec.tolist(),
                      translation_clipped=step.translation_clipped, rotation_clipped=step.rotation_clipped,
                      target_position=target_position.tolist(), target_quaternion_xyzw=target_quaternion.tolist(),
                      gripper_value=step.gripper_open)
        if not inside_workspace(target_position, self.minimum, self.maximum):
            self.log("workspace_violation", **record)
            self.latch(f"target {np.round(target_position, 4).tolist()} outside workspace "
                       f"{self.minimum.tolist()}..{self.maximum.tolist()}"); return
        pose = self.pose()
        if pose is None: self.latch("end-effector pose unavailable during chunk"); return
        lead = float(np.linalg.norm(target_position - pose[0]))
        lead_rotation = rotation_angle_between(target_quaternion, pose[1])
        record.update(measured_position=pose[0].tolist(), measured_quaternion_xyzw=pose[1].tolist(),
                      target_lead_m=lead, target_lead_rad=lead_rotation)
        # Dry-run never moves the arm, so the lead there is only informational.
        if self.args.execute and (lead > self.args.max_target_lead or lead_rotation > self.args.max_target_lead_rotation):
            self.log("tracking_violation", **record)
            self.latch(f"target leads measured pose by {lead:.4f} m / {lead_rotation:.4f} rad; arm is not following"); return
        desired_open = self.hysteresis.desired_open(step.gripper_open, self.commanded_gripper_open)
        record["gripper_request"] = None if desired_open is None else ("open" if desired_open else "close")
        if self.args.execute:
            if not self.send_target(target_position, target_quaternion): return
            self.executed += 1
            if self.args.enable_gripper and desired_open is not None: self.request_gripper(desired_open)
        else:
            self.executed += 1
            # Dry-run: track the virtual gripper so later decisions are meaningful.
            if desired_open is not None: self.commanded_gripper_open = desired_open
        self.log("action", robot_command_sent=bool(self.args.execute), **record)
        self.next_action_at += 1.0 / self.args.action_hz
        # Never burst several targets to catch up after scheduling jitter.
        self.next_action_at = max(self.next_action_at, now)
        if not self.chunk:
            self.chunks_done += 1
            # The next observation must follow the final action's full interval.
            self.need_fresh_after = now + 1.0 / self.args.action_hz

    # ----------------------------------------------------------------- outputs
    def send_target(self, position: np.ndarray, quaternion: np.ndarray) -> bool:
        if not self.target_client.service_is_ready(): self.latch("/set_target_pose became unavailable"); return False
        request = SetTargetPose.Request(); request.stop = False
        request.target_pose = PoseStamped()
        request.target_pose.header.stamp = self.get_clock().now().to_msg(); request.target_pose.header.frame_id = "world"
        p = request.target_pose.pose.position; p.x, p.y, p.z = (float(v) for v in position)
        request.target_pose.pose.orientation = Quaternion(
            x=float(quaternion[0]), y=float(quaternion[1]), z=float(quaternion[2]), w=float(quaternion[3]))
        self.target_client.call_async(request).add_done_callback(self.target_response)
        return True

    def target_response(self, future) -> None:
        try: response = future.result()
        except Exception as error: self.latch(f"target request failed: {error}"); return
        if not response.success: self.latch(f"Servo rejected target: {response.message}")

    def request_stop(self) -> None:
        """Ask the Servo tracker for its controlled stop and Cartesian hold."""
        if self.target_client is None or not self.target_client.service_is_ready(): return
        request = SetTargetPose.Request(); request.stop = True
        request.target_pose = PoseStamped(); request.target_pose.header.frame_id = "world"
        self.target_client.call_async(request)

    def request_gripper(self, open_: bool) -> None:
        # Non-blocking: the arm keeps its 15 Hz targets while the gripper moves,
        # as during VR collection. Repeated requests for the same state are dropped.
        if self.pending_gripper_open is not None: return
        self.pending_gripper_open = open_; self.commanded_gripper_open = open_
        if open_:
            goal = Move.Goal(); goal.width = self.args.gripper_open_width; goal.speed = self.args.gripper_speed
            future = self.move_client.send_goal_async(goal)
        else:
            goal = Grasp.Goal(); goal.width = self.args.gripper_closed_width; goal.speed = self.args.gripper_speed
            goal.force = self.args.gripper_force
            goal.epsilon.inner = self.args.gripper_epsilon_inner; goal.epsilon.outer = self.args.gripper_epsilon_outer
            future = self.grasp_client.send_goal_async(goal)
        self.get_logger().warning(f"gripper {'OPEN' if open_ else 'CLOSE'} requested: width={goal.width:.4f}m")
        self.log("gripper_request", open=open_, width=goal.width)
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
        self.gripper_open = self.pending_gripper_open; self.pending_gripper_open = None
        self.log("gripper_done", open=self.gripper_open, measured_width=self.gripper_width())
        self.get_logger().info(f"gripper {'OPENED' if self.gripper_open else 'CLOSED'}; width={self.gripper_width()}")

    # ----------------------------------------------------------------- lifecycle
    def arm(self) -> None:
        deadline = time.monotonic() + self.args.startup_timeout
        while time.monotonic() < deadline and self.pose() is None:
            rclpy.spin_once(self, timeout_sec=.1)
        if self.pose() is None: raise RuntimeError("world -> fr3_link8 unavailable; is run_robot.sh running?")
        if self.args.gripper_state_source == "measured" or self.args.enable_gripper:
            while time.monotonic() < deadline and self.gripper_width() is None:
                rclpy.spin_once(self, timeout_sec=.1)
            if self.gripper_width() is None: raise RuntimeError("gripper finger feedback unavailable")
            width = self.gripper_width()
            self.commanded_gripper_open = gripper_open_from_width(
                width, self.args.gripper_state_closed_width, self.args.gripper_state_open_width) >= 0.5
            self.get_logger().info(f"gripper feedback width={width:.4f}m -> open={self.commanded_gripper_open}")
        if not self.args.execute: return
        if not self.target_client.wait_for_service(timeout_sec=max(1.0, deadline - time.monotonic())):
            raise RuntimeError("/set_target_pose unavailable; run_robot.sh (Servo stack) must be running")
        if self.args.enable_gripper:
            if not self.move_client.wait_for_server(timeout_sec=10) or not self.grasp_client.wait_for_server(timeout_sec=10):
                raise RuntimeError("gripper action server unavailable")

    def finish(self, reason: str) -> None:
        self.request_stop(); self.finished = True
        self.log("complete", reason=reason, actions_executed=self.executed, chunks=self.chunks_done)
        self.get_logger().warning(f"{reason}: executed {self.executed} actions in {self.chunks_done} chunks; hold requested")

    def latch(self, reason: str) -> None:
        if self.fault: return
        self.fault = True; self.chunk = []; self.request_stop()
        self.log("fault", reason=reason, actions_executed=self.executed)
        self.get_logger().error(f"FAULT LATCHED: {reason}; Servo hold requested; restart required")

    def log(self, event: str, **fields: Any) -> None:
        if self.log_file:
            self.log_file.write(json.dumps({"monotonic_s": time.monotonic(), "event": event, **fields}, default=str) + "\n")

    def destroy_node(self):
        self.request_stop()
        self.pool.shutdown(wait=False, cancel_futures=True)
        if self.policy is not None: self.policy.close()
        if self.log_file: self.log_file.close()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add = parser.add_argument
    add("--server-host", default="10.24.116.66"); add("--server-port", type=int, default=8095)
    add("--instruction", default="put the red test tube into the red basket")
    add("--exterior-image-topic", default="/camera_left/image_rect")
    add("--wrist-image-topic", default="/camera/camera_front/color/image_raw")
    add("--bounds-config", required=True, help="YAML with workspace_bounds_world.min/max")
    add("--rotation-frame", choices=("world", "tool"), default=None,
        help="optional cross-check of the server's rotation_execution; required for --test-axis")
    add("--chunk-start", choices=("measured", "last_target"), default="measured",
        help="start each chunk's accumulated target from the measured pose or the previous target")
    add("--action-hz", type=float, default=15.0); add("--execute-horizon", type=int, default=15)
    add("--max-actions", type=int, default=15); add("--max-chunks", type=int, default=1)
    add("--max-translation", type=float, default=0.006, help="per-step translation norm limit (m)")
    add("--max-rotation", type=float, default=0.035, help="per-step rotation norm limit (rad)")
    add("--max-target-lead", type=float, default=0.06, help="abort if target leads measured pose by more (m)")
    add("--max-target-lead-rotation", type=float, default=0.35, help="same for rotation (rad)")
    add("--max-policy-latency", type=float, default=5.0); add("--max-observation-age", type=float, default=.2)
    add("--max-camera-skew", type=float, default=.05); add("--max-tf-age", type=float, default=.1)
    add("--startup-timeout", type=float, default=60.0)
    add("--gripper-state-source", choices=("measured", "command"), default="measured",
        help="state[6]: measured finger width mapped linearly, or the last commanded open/close")
    add("--gripper-state-closed-width", type=float, default=0.01)
    add("--gripper-state-open-width", type=float, default=0.08)
    add("--enable-gripper", action="store_true")
    add("--gripper-open-threshold", type=float, default=.6); add("--gripper-close-threshold", type=float, default=.4)
    add("--gripper-open-width", type=float, default=.08); add("--gripper-closed-width", type=float, default=.01)
    add("--gripper-speed", type=float, default=.05); add("--gripper-force", type=float, default=20.)
    add("--gripper-epsilon-inner", type=float, default=.1); add("--gripper-epsilon-outer", type=float, default=.2)
    add("--test-axis", default="", help="six-direction commissioning: +x,-x,+y,-y,+z,-z,+rx,...,-rz (no policy)")
    add("--test-magnitude", type=float, default=0.04, help="command units; 0.04 -> 3 mm or 0.006 rad per step")
    add("--test-steps", type=int, default=5)
    add("--jsonl-log", default=""); add("--execute", action="store_true"); add("--confirm-real-robot", default="")
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if not 1 <= args.execute_horizon <= ACTION_SHAPE[0]: parser.error(f"execute-horizon must be in [1, {ACTION_SHAPE[0]}]")
    if not 1 <= args.max_actions <= 2000 or not 1 <= args.max_chunks <= 200: parser.error("action/chunk budget out of range")
    if args.action_hz != 15.0: parser.error("the EEF protocol is 15 Hz; do not execute it at another rate")
    if not 0 < args.max_translation <= 0.02 or not 0 < args.max_rotation <= 0.15:
        parser.error("per-step limits must be in (0, 0.02] m and (0, 0.15] rad")
    if not 0 < args.gripper_speed <= .1 or not 1 <= args.gripper_force <= 40: parser.error("gripper speed/force out of range")
    if not 0 <= args.gripper_closed_width < args.gripper_open_width <= .08: parser.error("gripper widths must satisfy 0 <= closed < open <= 0.08")
    if args.test_axis and not args.rotation_frame: parser.error("--test-axis requires an explicit --rotation-frame")
    if args.execute and args.confirm_real_robot != CONFIRM_TOKEN: parser.error(f"execute requires --confirm-real-robot {CONFIRM_TOKEN}")
    return args


def main() -> None:
    args = parse_args(); rclpy.init(); node = EefAdapter(args)
    try:
        node.arm()
        while rclpy.ok() and not node.finished and not node.fault: rclpy.spin_once(node, timeout_sec=.1)
        # Let the hold request leave before shutdown.
        for _ in range(5): rclpy.spin_once(node, timeout_sec=.02)
    finally: node.destroy_node(); rclpy.shutdown()
    if node.fault: raise SystemExit(1)


if __name__ == "__main__": main()
