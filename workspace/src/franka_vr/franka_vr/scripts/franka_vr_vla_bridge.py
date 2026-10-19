#!/usr/bin/env python3
"""Safety bridge for the three-view real-robot FrankaVR ActionCodec model."""
import argparse
import concurrent.futures
import collections
import functools
import json
from pathlib import Path
import threading
import time

import cv2
import msgpack
import numpy as np
import websockets.exceptions
import websockets.sync.client
import yaml
import rclpy
from cv_bridge import CvBridge
from franka_msgs.action import Move
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply
from franka_vr.srv import SetTargetPose

EXPECTED = "qwen3vl2b_tube_pick_simulation_state_first_step6000"
JOINTS = tuple(f"fr3_joint{i}" for i in range(1, 8))
# Keep this exactly aligned with lerobot_episode_recorder.py: the recorded
# state stores the measured *total* opening, not one finger displacement.
GRIPPER_JOINTS = ("fr3_finger_joint1", "fr3_finger_joint2")

def _pack(value):
    if isinstance(value, np.ndarray):
        return {b"__ndarray__": True, b"data": value.tobytes(), b"dtype": value.dtype.str, b"shape": value.shape}
    if isinstance(value, np.generic):
        return {b"__npgeneric__": True, b"data": value.item(), b"dtype": value.dtype.str}
    return value

def _unpack(value):
    if b"__ndarray__" in value:
        return np.ndarray(buffer=value[b"data"], dtype=np.dtype(value[b"dtype"]), shape=value[b"shape"])
    if b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value

PACK = functools.partial(msgpack.Packer, default=_pack)
UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack)

class Client:
    def __init__(self, host, port):
        self.uri, self.lock, self.ws = f"ws://{host}:{port}", threading.Lock(), None
        self.metadata = self.connect()
    def connect(self):
        self.ws = websockets.sync.client.connect(self.uri, compression=None, max_size=None, open_timeout=5, close_timeout=2, proxy=None)
        return UNPACK(self.ws.recv(timeout=5))
    def act(self, observation):
        with self.lock:
            payload = PACK().pack(observation)
            try:
                self.ws.send(payload); response = self.ws.recv(timeout=60)
            except websockets.exceptions.ConnectionClosed:
                self.metadata = self.connect(); self.ws.send(payload); response = self.ws.recv(timeout=60)
        if isinstance(response, str): raise RuntimeError(response)
        action = np.asarray(UNPACK(response)["action"], dtype=np.float64)
        if action.shape != (15, 7) or not np.isfinite(action).all():
            raise ValueError(f"Expected finite FrankaVR action chunk (15,7), got {action.shape}")
        return action

class Bridge(Node):
    def __init__(self, args):
        super().__init__("franka_vr_vla_bridge")
        self.args, self.cv, self.joints = args, CvBridge(), {}
        # The policy was trained on a single, synchronized 30 Hz observation.
        # Keep short timestamped histories rather than combining whichever ROS
        # callbacks happened to arrive most recently.
        self.images = {key: collections.deque(maxlen=30) for key in ("front", "left", "right")}
        self.state_history = collections.deque(maxlen=200)
        self.joint_stamps = {}
        self.tf, self.listener = Buffer(), None
        self.listener = TransformListener(self.tf, self)
        self.client = Client(args.server_host, args.server_port)
        if self.client.metadata.get("model_family") != EXPECTED:
            raise RuntimeError(f"Wrong server metadata: {self.client.metadata}")
        bounds = yaml.safe_load(open(args.bounds_config))["workspace_bounds_world"]
        self.minimum, self.maximum = np.asarray(bounds["min"], float), np.asarray(bounds["max"], float)
        if not np.isfinite(self.minimum).all() or not np.isfinite(self.maximum).all() or np.any(self.maximum <= self.minimum):
            raise RuntimeError("Finite world workspace bounds are required")
        for key, topic in (("front", args.front_topic), ("left", args.left_topic), ("right", args.right_topic)):
            self.create_subscription(Image, topic, lambda msg, key=key: self.image(key, msg), qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.joint, 10)
        self.create_subscription(JointState, "/franka/joint_states", self.joint, 10)
        self.create_subscription(JointState, "/fr3_gripper/franka_gripper/joint_states", self.joint, 10)
        self.target = self.create_client(SetTargetPose, "/set_target_pose")
        self.gripper = ActionClient(self, Move, "/fr3_gripper/franka_gripper/move")
        self.gripper_closed = None
        self.pending_gripper_closed = None
        self.pending_gripper_goal_finished = False
        self.pending_gripper_started_at = 0.0
        self.last_gripper_width = None
        self.last_safe_target = None
        self.last_target_sent = 0.0
        self.observation_state = args.observation_state
        self.observation_save_dir = Path(args.save_observation_dir).resolve() if args.save_observation_dir else None
        self.observation_sequence = 0
        if self.observation_save_dir is not None:
            self.observation_save_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Saving exact policy observations under {self.observation_save_dir}")
        self.pool, self.future, self.last_request = concurrent.futures.ThreadPoolExecutor(max_workers=1), None, 0.0
        self.next_chunk, self.active_actions, self.next_action_at = None, collections.deque(), 0.0
        self.next_observation_at = 0.0
        self.create_timer(.01, self.tick)
        state_source = "fixed VLA_OBSERVATION_STATE" if self.observation_state is not None else "measured joint/gripper state"
        self.get_logger().info(f"Connected to FrankaVR policy: {self.client.metadata}; mode={'EXECUTE' if args.execute else 'DRY RUN'}; observation state={state_source}")
    def stamp(self, msg):
        stamp = msg.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else self.get_clock().now().nanoseconds * 1e-9
    def image(self, key, msg):
        try: self.images[key].append((self.stamp(msg), np.ascontiguousarray(self.cv.imgmsg_to_cv2(msg, desired_encoding="rgb8"), dtype=np.uint8)))
        except Exception as error: self.get_logger().warning(f"Cannot decode {key}: {error}")
    def joint(self, msg):
        stamp = self.stamp(msg)
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position
            self.joint_stamps[name] = stamp
        if any(name not in self.joints for name in (*JOINTS, *GRIPPER_JOINTS)):
            return
        component_stamps = [self.joint_stamps[name] for name in (*JOINTS, *GRIPPER_JOINTS)]
        if max(component_stamps) - min(component_stamps) > self.args.observation_sync_tolerance:
            return
        # The recorder stores total measured opening in metres as
        # abs(finger_joint1) + abs(finger_joint2).  Preserve its continuous
        # state representation, then map its observed fully-open width to the
        # model's 0=closed / 1=open gripper-position convention.
        gripper_width = sum(abs(float(self.joints[name])) for name in GRIPPER_JOINTS)
        gripper_position = np.float32(np.clip(gripper_width / self.args.gripper_state_open_width, 0.0, 1.0))
        self.last_gripper_width = gripper_width
        if self.pending_gripper_closed is not None:
            tolerance = max(0.003, 0.05 * self.args.gripper_state_open_width)
            reached = gripper_width <= tolerance if self.pending_gripper_closed else gripper_width >= self.args.gripper_state_open_width - tolerance
            if reached:
                self.gripper_closed = self.pending_gripper_closed
                self.pending_gripper_closed = None
                self.pending_gripper_goal_finished = False
                self.get_logger().info(f"Measured VLA gripper {'closed' if self.gripper_closed else 'open'} at width={gripper_width:.5f} m")
            elif self.pending_gripper_goal_finished and time.monotonic() - self.pending_gripper_started_at > 2.0:
                self.get_logger().warning(
                    f"VLA gripper goal completed but measured width is {gripper_width:.5f} m; "
                    f"waiting for {'closed' if self.pending_gripper_closed else 'open'} feedback",
                    throttle_duration_sec=2.0)
        self.state_history.append((max(component_stamps), np.array([*(self.joints[n] for n in JOINTS), gripper_position], np.float32)))
    def pose(self):
        try:
            t = self.tf.lookup_transform("world", "fr3_link8", Time(), timeout=Duration(seconds=.05)).transform
            return np.array([t.translation.x,t.translation.y,t.translation.z]), np.array([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
        except Exception as error:
            self.get_logger().info(f"Waiting for world -> fr3_link8: {error}", throttle_duration_sec=2.0); return None
    @staticmethod
    def nearest(samples, timestamp):
        return min(samples, key=lambda sample: abs(sample[0] - timestamp))
    def save_observation(self, observation, timestamps):
        """Persist the exact RGB arrays selected for one policy request."""
        if self.observation_save_dir is None:
            return
        self.observation_sequence += 1
        batch_dir = self.observation_save_dir / f"{self.observation_sequence:06d}"
        try:
            batch_dir.mkdir()
            for view in ("front", "left", "right"):
                # OpenCV writes BGR, whereas the model receives RGB_uint8.
                image = observation[f"{view}_image"]
                if not cv2.imwrite(str(batch_dir / f"{view}.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError(f"could not write {view}.png")
            (batch_dir / "metadata.json").write_text(json.dumps({
                "instruction": observation["instruction"],
                "joint_state": observation["joint_state"].tolist(),
                "image_timestamps_sec": timestamps,
            }, indent=2) + "\n")
        except Exception as error:
            self.get_logger().error(f"Could not save policy observation {batch_dir}: {error}")
    def observation(self):
        if not all(self.images[key] for key in ("front", "left", "right")):
            self.get_logger().info("Waiting for front/left/right images", throttle_duration_sec=2.0); return None
        if self.observation_state is None and not self.state_history:
            self.get_logger().info("Waiting for timestamped seven-arm-joint and gripper state", throttle_duration_sec=2.0); return None
        # Use the most recent front image for which all other policy inputs have
        # a nearest timestamp inside the configured synchronization window.
        for front in reversed(self.images["front"]):
            aligned = [front, self.nearest(self.images["left"], front[0]), self.nearest(self.images["right"], front[0])]
            if self.observation_state is None:
                aligned.append(self.nearest(self.state_history, front[0]))
            timestamps = [sample[0] for sample in aligned]
            if max(timestamps) - min(timestamps) <= self.args.observation_sync_tolerance:
                joint_state = (self.observation_state.copy() if self.observation_state is not None else aligned[3][1].copy())
                observation = {"front_image":aligned[0][1].copy(),"left_image":aligned[1][1].copy(),"right_image":aligned[2][1].copy(),"joint_state":joint_state,"instruction":self.args.instruction}
                self.save_observation(observation, {
                    "front": aligned[0][0], "left": aligned[1][0], "right": aligned[2][0],
                    **({"joint_state": aligned[3][0]} if self.observation_state is None else {}),
                })
                return observation
        self.get_logger().info(f"Waiting for timestamp-aligned observation (max skew {self.args.observation_sync_tolerance:.3f}s)", throttle_duration_sec=2.0)
        return None
    def tick(self):
        if self.future is not None:
            if self.future.done():
                done, self.future = self.future, None
                try:
                    # Keep only the most recent result while a full action block is
                    # being executed. This prevents old observations from being
                    # replayed after a slow policy response.
                    self.next_chunk = done.result()[:self.args.actions_per_chunk].copy()
                    self.get_logger().info(
                        f"Queued {len(self.next_chunk)} actions from FrankaVR 15-step chunk; "
                        f"first={np.array2string(self.next_chunk[0], precision=4)}")
                    # Dry-run keeps requesting observations for inspection but
                    # must not retain an unexecuted block forever.
                    if not self.args.execute:
                        self.next_chunk = None
                except Exception as error:
                    self.get_logger().error(f"Policy request failed: {error}")

        now = time.monotonic()
        if self.args.execute and not self.active_actions and self.next_chunk is not None:
            self.active_actions = collections.deque(self.next_chunk)
            self.next_chunk = None
            self.next_action_at = now
            self.get_logger().info(f"Starting {len(self.active_actions)}-action VLA block at {self.args.action_hz:g} Hz")

        if self.args.execute and self.active_actions and now >= self.next_action_at:
            raw = self.active_actions.popleft()
            self.execute_policy_action(raw)
            self.next_action_at += 1.0 / self.args.action_hz
            # Do not try to catch up by bursting several Cartesian targets into
            # one controller cycle after host scheduling jitter.
            self.next_action_at = max(self.next_action_at, now)
            if not self.active_actions:
                # The final target represents the last action interval.  Do
                # not sample the next policy input until that interval ends.
                self.next_observation_at = now + 1.0 / self.args.action_hz

        # One observation belongs to exactly one action block: acquire it only
        # at a block boundary, wait for inference while holding pose, then
        # execute all 15 actions without sampling a competing next block.
        if (self.future is None and not self.active_actions and self.next_chunk is None
                and now >= self.next_observation_at and now - self.last_request >= 1.0 / self.args.policy_hz):
            obs = self.observation()
            if obs is not None:
                self.last_request = now
                self.future = self.pool.submit(self.client.act, obs)
        if not self.active_actions:
            self.refresh_hold()

    def send_target(self, position, quaternion, *, remember=True):
        if not self.target.service_is_ready():
            self.get_logger().warning("/set_target_pose is unavailable; retaining the previous hold", throttle_duration_sec=2.0)
            return False
        request = SetTargetPose.Request(); request.target_pose = PoseStamped()
        request.target_pose.header.stamp=self.get_clock().now().to_msg(); request.target_pose.header.frame_id="world"; request.stop=False
        request.target_pose.pose.position.x,request.target_pose.pose.position.y,request.target_pose.pose.position.z=position
        request.target_pose.pose.orientation=Quaternion(x=float(quaternion[0]),y=float(quaternion[1]),z=float(quaternion[2]),w=float(quaternion[3]))
        future = self.target.call_async(request)
        future.add_done_callback(self.target_response)
        if remember:
            self.last_safe_target = (np.asarray(position, dtype=float).copy(), np.asarray(quaternion, dtype=float).copy())
        self.last_target_sent = time.monotonic()
        return True

    def target_response(self, future):
        try:
            response = future.result()
            if not response.success: self.get_logger().error(f"Servo rejected VLA target: {response.message}")
        except Exception as error:
            self.get_logger().error(f"Servo target request failed: {error}")

    def refresh_hold(self):
        """Keep the arm controller supplied with a valid pose between policy replies."""
        if not self.args.execute or time.monotonic() - self.last_target_sent < self.args.hold_refresh_sec: return
        if self.last_safe_target is None:
            pose = self.pose()
            if pose is None: return
            self.last_safe_target = pose
            self.get_logger().info("Arming Cartesian hold at the measured end-effector pose")
        self.send_target(*self.last_safe_target, remember=False)

    def send_gripper(self, closed):
        if self.gripper_closed is closed or self.pending_gripper_closed is not None:
            return
        if not self.gripper.server_is_ready():
            self.get_logger().warning("Franka gripper action server is unavailable; gripper command skipped", throttle_duration_sec=2.0)
            return
        goal = Move.Goal(); goal.width = self.args.gripper_closed_width if closed else self.args.gripper_open_width; goal.speed = self.args.gripper_speed
        self.pending_gripper_closed = closed
        self.pending_gripper_goal_finished = False
        self.pending_gripper_started_at = time.monotonic()
        future = self.gripper.send_goal_async(goal)
        future.add_done_callback(lambda result: self.gripper_response(result, closed))

    def gripper_response(self, future, closed):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.pending_gripper_closed = None
                self.get_logger().error("Franka gripper rejected VLA command")
                return
            self.get_logger().info(f"VLA gripper goal accepted: {'close' if closed else 'open'}")
            goal_handle.get_result_async().add_done_callback(lambda result: self.gripper_result(result, closed))
        except Exception as error:
            self.pending_gripper_closed = None
            self.get_logger().error(f"Franka gripper command failed: {error}")
    def gripper_result(self, future, closed):
        try:
            result = future.result().result
            if not result.success:
                self.pending_gripper_closed = None
                self.get_logger().error(f"Franka gripper {'close' if closed else 'open'} failed: {result.error}")
                return
            self.pending_gripper_goal_finished = True
            self.get_logger().info(f"VLA gripper goal completed: {'close' if closed else 'open'}; awaiting measured width")
        except Exception as error:
            self.pending_gripper_closed = None
            self.get_logger().error(f"Franka gripper result failed: {error}")
    def execute_policy_action(self, raw):
        pose = self.pose()
        if pose is None: return
        position, quaternion = pose
        delta = np.clip(raw[:3]*.075, -self.args.max_translation, self.args.max_translation)
        rpy = np.clip(raw[3:6]*.15, -self.args.max_rotation, self.args.max_rotation)
        target_position = position + delta
        if np.any(target_position < self.minimum) or np.any(target_position > self.maximum):
            self.get_logger().error(f"Rejected FrankaVR target outside workspace: {target_position}"); return
        target_quaternion = quaternion_multiply(quaternion_from_euler(*rpy), quaternion)
        if not self.send_target(target_position, target_quaternion): return
        if self.args.enable_gripper:
            # The final action component is the binary gripper state from the
            # FrankaVR dataset: 0 means closed and 1 means open.  The input
            # joint-state uses the same convention; do not treat it as a
            # "closed probability" and invert the physical gripper command.
            if raw[6] >= self.args.gripper_close_threshold:
                self.send_gripper(closed=False)
            elif raw[6] <= self.args.gripper_close_threshold - self.args.gripper_hysteresis:
                self.send_gripper(closed=True)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--server-host",default="10.24.116.29"); parser.add_argument("--server-port",type=int,default=8093); parser.add_argument("--front-topic",default="/camera/camera_front/color/image_raw"); parser.add_argument("--left-topic",default="/camera_left/image_rect"); parser.add_argument("--right-topic",default="/camera_right/image_rect"); parser.add_argument("--instruction",required=True); parser.add_argument("--bounds-config",required=True); parser.add_argument("--observation-state",default=None,help="optional JSON float32[8] model-state override"); parser.add_argument("--save-observation-dir",default=None,help="directory to save exact observation batches sent to the policy"); parser.add_argument("--policy-hz",type=float,default=30.0); parser.add_argument("--actions-per-chunk",type=int,default=15); parser.add_argument("--action-hz",type=float,default=15.0); parser.add_argument("--observation-sync-tolerance",type=float,default=.030); parser.add_argument("--max-translation",type=float,default=.010); parser.add_argument("--max-rotation",type=float,default=.08); parser.add_argument("--hold-refresh-sec",type=float,default=.20); parser.add_argument("--enable-gripper",action="store_true"); parser.add_argument("--gripper-close-threshold",type=float,default=.50); parser.add_argument("--gripper-hysteresis",type=float,default=.10); parser.add_argument("--gripper-state-open-width",type=float,default=.10300,help="measured total finger opening corresponding to model gripper state 1"); parser.add_argument("--gripper-open-width",type=float,default=.10300,help="physical total opening target for a VLA open command"); parser.add_argument("--gripper-closed-width",type=float,default=.0); parser.add_argument("--gripper-speed",type=float,default=.02); parser.add_argument("--execute",action="store_true"); args=parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if args.observation_state is not None and args.observation_state.strip():
        try: args.observation_state = np.asarray(json.loads(args.observation_state), dtype=np.float32)
        except (TypeError, ValueError, json.JSONDecodeError) as error: parser.error(f"--observation-state must be JSON float32[8]: {error}")
        if args.observation_state.shape != (8,) or not np.isfinite(args.observation_state).all(): parser.error("--observation-state must be finite float32[8]")
    else:
        args.observation_state = None
    if args.policy_hz <= 0 or args.action_hz <= 0 or args.actions_per_chunk != 15 or args.observation_sync_tolerance <= 0 or args.max_translation <= 0 or args.max_rotation <= 0 or args.hold_refresh_sec <= 0 or args.gripper_hysteresis <= 0 or args.gripper_state_open_width <= 0 or args.gripper_open_width < 0 or args.gripper_closed_width < 0 or args.gripper_open_width < args.gripper_closed_width or args.gripper_speed <= 0:
        parser.error("policy, limits, hold refresh, and a complete 15-action chunk are required")
    rclpy.init(); node=Bridge(args)
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
