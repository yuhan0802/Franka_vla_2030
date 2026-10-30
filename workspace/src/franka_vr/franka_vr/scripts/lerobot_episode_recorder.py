#!/usr/bin/env python3
"""Write synchronized FR3 teleoperation episodes in a LeRobot v2.1-style layout."""
import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

CAMERAS = ("front", "left", "right")
CAMERA_FIELDS = {"left": "exterior_image_left", "right": "exterior_image_right", "front": "wrist_image"}
JOINTS = tuple(f"fr3_joint{i}" for i in range(1, 8))
# The real FR3 gripper publishes its feedback separately from the arm on
# /fr3_gripper/franka_gripper/joint_states.
GRIPPER_JOINTS = ("fr3_finger_joint1", "fr3_finger_joint2")
STATE_LAYOUT = "fr3_joint_positions_plus_measured_gripper_width_v2"
ACTION_LAYOUT = "sent_joint_velocity_plus_gripper_target_width_v2_missing_target_is_nan"
LEGACY_ACTION_LAYOUT = "cartesian_target_pose_plus_measured_gripper_width_v1"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class LeRobotEpisodeWriter:
    """Append one episode using Parquet + one MP4 per camera."""
    def __init__(self, root: Path, fps: int, action_fps: int, task: str, episode_index: int):
        self.root, self.fps, self.action_fps, self.task = root, fps, action_fps, task
        self.episode_index, self.frames, self.action_frames, self.video_writers = episode_index, [], [], {}
        self.features = {
            "observation.state": {"dtype": "float32", "shape": [8],
                                  "names": [*JOINTS, "gripper_width"]},
            # Training action is always the post-limit command sent to the
            # controllers, not the operator's requested command.
            "action": {"dtype": "float32", "shape": [8],
                       "names": [*JOINTS, "gripper_target_width"]},
            "action_legacy_pose": {"dtype": "float32", "shape": [8],
                       "names": ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper_width"]},
            "joint_position": {"dtype": "float32", "shape": [7], "names": list(JOINTS)},
            "joint_velocity": {"dtype": "float32", "shape": [7], "names": list(JOINTS)},
            "gripper_width": {"dtype": "float32", "shape": [1], "names": ["metres"]},
            "gripper_position": {"dtype": "float32", "shape": [1], "names": ["metres"]},
            "eef_position": {"dtype": "float32", "shape": [3], "names": ["x", "y", "z"]},
            "eef_quaternion": {"dtype": "float32", "shape": [4], "names": ["x", "y", "z", "w"]},
            "joint_velocity_command_requested": {"dtype": "float32", "shape": [7], "names": list(JOINTS)},
            "joint_velocity_command_sent": {"dtype": "float32", "shape": [7], "names": list(JOINTS)},
            "gripper_target_width": {"dtype": "float32", "shape": [1], "names": ["metres"]},
            # False means no raw target command had arrived yet. In that case
            # gripper_target_width and action[-1] are NaN, never a fabricated
            # closed-gripper command.
            "gripper_target_width_valid": {"dtype": "bool", "shape": [1], "names": ["valid"]},
            "eef_delta_position_command": {"dtype": "float32", "shape": [3], "names": ["x", "y", "z"]},
            "eef_delta_rotation_command": {"dtype": "float32", "shape": [3], "names": ["roll", "pitch", "yaw"]},
            "eef_delta_position_command_requested": {"dtype": "float32", "shape": [3], "names": ["x", "y", "z"]},
            "eef_delta_rotation_command_requested": {"dtype": "float32", "shape": [3], "names": ["roll", "pitch", "yaw"]},
        }
        for name in CAMERAS:
            self.features[f"observation.images.{name}"] = {
                "dtype": "video", "shape": [IMAGE_HEIGHT, IMAGE_WIDTH, 3],
                "names": ["height", "width", "channel"],
            }
            # Canonical collection names for downstream consumers. They point
            # at the same MP4 frame as the retained historical view names.
            self.features[CAMERA_FIELDS[name]] = {
                "dtype": "video", "shape": [IMAGE_HEIGHT, IMAGE_WIDTH, 3],
                "names": ["height", "width", "channel"],
            }

    def video_path(self, camera: str):
        return self.root / "videos" / "chunk-000" / f"observation.images.{camera}" / (
            f"episode_{self.episode_index:06d}.mp4"
        )

    def raw_video_path(self, camera: str):
        return self.video_path(camera).with_suffix(".raw.mp4")

    def _video_writer(self, camera: str):
        if camera not in self.video_writers:
            path = self.raw_video_path(camera)
            path.parent.mkdir(parents=True, exist_ok=True)
            writer = cv2.VideoWriter(
                str(path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps,
                (IMAGE_WIDTH, IMAGE_HEIGHT)
            )
            if not writer.isOpened():
                raise RuntimeError(f"Cannot open MP4 writer for {path}")
            self.video_writers[camera] = writer
        return self.video_writers[camera]

    def _encode_videos(self):
        """Transcode OpenCV's broadly available MP4V output to browser-safe H.264."""
        for camera in CAMERAS:
            raw_path = self.raw_video_path(camera)
            final_path = self.video_path(camera)
            subprocess.run([
                "ffmpeg", "-y", "-loglevel", "error", "-i", str(raw_path),
                "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(final_path),
            ], check=True)
            raw_path.unlink()

    def _write_stats(self, meta: Path):
        """Rebuild dataset-level normalization statistics across every episode."""
        totals = {}
        for data_path in sorted((self.root / "data").glob("chunk-*/episode_*.parquet")):
            table = pq.read_table(data_path, columns=["observation.state", "action"])
            for key in ("observation.state", "action"):
                values = np.asarray(table[key].to_pylist(), dtype=np.float64)
                if values.size == 0:
                    continue
                current = totals.setdefault(key, {
                    "count": 0, "sum": np.zeros(values.shape[1]), "sum_sq": np.zeros(values.shape[1]),
                    "min": np.full(values.shape[1], np.inf), "max": np.full(values.shape[1], -np.inf),
                })
                finite = np.isfinite(values)
                if "valid_count" not in current:
                    current["valid_count"] = np.zeros(values.shape[1], dtype=np.int64)
                current["count"] += len(values)
                current["valid_count"] += finite.sum(axis=0)
                current["sum"] += np.where(finite, values, 0.0).sum(axis=0)
                current["sum_sq"] += np.where(finite, np.square(values), 0.0).sum(axis=0)
                current["min"] = np.minimum(
                    current["min"], np.where(finite, values, np.inf).min(axis=0)
                )
                current["max"] = np.maximum(
                    current["max"], np.where(finite, values, -np.inf).max(axis=0)
                )
        stats = {}
        for key, current in totals.items():
            valid_count = current["valid_count"]
            mean = np.divide(
                current["sum"], valid_count, out=np.full_like(current["sum"], np.nan), where=valid_count > 0
            )
            variance = np.maximum(
                np.divide(current["sum_sq"], valid_count, out=np.full_like(current["sum_sq"], np.nan),
                          where=valid_count > 0) - np.square(mean),
                0.0,
            )
            minimum = np.where(np.isfinite(current["min"]), current["min"], np.nan)
            maximum = np.where(np.isfinite(current["max"]), current["max"], np.nan)
            stats[key] = {
                "min": minimum.tolist(), "max": maximum.tolist(),
                "mean": mean.tolist(), "std": np.sqrt(variance).tolist(),
                "count": current["count"], "valid_count": valid_count.tolist(),
            }
        (meta / "stats.json").write_text(json.dumps(stats, indent=2))

    def add_frame(self, sample, images):
        index = len(self.frames)
        row = {"index": index, "episode_index": self.episode_index, "task_index": 0,
               "timestamp": sample["record_timestamp"],
               "observation.state": sample["state"].tolist(), "action": sample["action"].tolist(),
               "action_legacy_pose": sample["legacy_action"].tolist(),
               "joint_position": sample["joint_position"].tolist(), "joint_velocity": sample["joint_velocity"].tolist(),
               "gripper_width": [sample["gripper_width"]], "gripper_position": [sample["gripper_width"]],
               "eef_position": sample["eef_position"].tolist(), "eef_quaternion": sample["eef_quaternion"].tolist(),
               "joint_velocity_command_requested": sample["joint_velocity_command_requested"].tolist(),
               "joint_velocity_command_sent": sample["joint_velocity_command_sent"].tolist(),
               "gripper_target_width": [sample["gripper_target_width"]],
               "gripper_target_width_valid": [sample["gripper_target_width_valid"]],
               "eef_delta_position_command": sample["eef_delta_position_command"].tolist(),
               "eef_delta_rotation_command": sample["eef_delta_rotation_command"].tolist(),
               "eef_delta_position_command_requested": sample["eef_delta_position_command_requested"].tolist(),
               "eef_delta_rotation_command_requested": sample["eef_delta_rotation_command_requested"].tolist(),
               "language_instruction": self.task, "success": True,
               "episode_id": f"episode_{self.episode_index:06d}", "timestamps": sample["timestamps"]}
        for name, (image, image_timestamp) in images.items():
            self._video_writer(name).write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            row[f"observation.images.{name}"] = {
                "path": str(self.video_path(name).relative_to(self.root)),
                "timestamp": image_timestamp,
            }
            row[CAMERA_FIELDS[name]] = row[f"observation.images.{name}"]
        self.frames.append(row)

    def add_action(self, sample, observation_index: int):
        """Append one controller-sent action with its preceding observation."""
        self.action_frames.append({
            "action_index": len(self.action_frames),
            "episode_index": self.episode_index,
            "episode_id": f"episode_{self.episode_index:06d}",
            "observation_index": observation_index,
            "timestamp": sample["record_timestamp"],
            "action": sample["action"].tolist(),
            "joint_velocity_command_requested": sample["joint_velocity_command_requested"].tolist(),
            "joint_velocity_command_sent": sample["joint_velocity_command_sent"].tolist(),
            "gripper_target_width": [sample["gripper_target_width"]],
            "gripper_target_width_valid": [sample["gripper_target_width_valid"]],
            "timestamps": sample["timestamps"],
        })

    @staticmethod
    def _read_jsonl(path: Path):
        if not path.is_file():
            return []
        return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]

    @staticmethod
    def _write_jsonl(path: Path, rows):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows))

    def finalize(self):
        if not self.frames:
            raise RuntimeError("No frames were captured")
        self.root.mkdir(parents=True, exist_ok=True)
        for writer in self.video_writers.values():
            writer.release()
        self._encode_videos()
        meta = self.root / "meta"
        meta.mkdir(exist_ok=True)
        tasks_path = meta / "tasks.jsonl"
        tasks = self._read_jsonl(tasks_path)
        task_to_index = {row["task"]: row["task_index"] for row in tasks}
        if self.task not in task_to_index:
            task_to_index[self.task] = len(tasks)
            tasks.append({"task_index": task_to_index[self.task], "task": self.task})
            self._write_jsonl(tasks_path, tasks)
        for row in self.frames:
            row["task_index"] = task_to_index[self.task]
        data_path = self.root / "data/chunk-000" / f"episode_{self.episode_index:06d}.parquet"
        data_path.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pylist(self.frames), data_path)
        action_path = self.root / "actions/chunk-000" / f"episode_{self.episode_index:06d}.parquet"
        action_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.action_frames:
            raise RuntimeError("No controller-sent actions were captured")
        pq.write_table(pa.Table.from_pylist(self.action_frames), action_path)
        episodes_path = meta / "episodes.jsonl"
        episodes = self._read_jsonl(episodes_path)
        episodes.append({"episode_index": self.episode_index, "episode_id": f"episode_{self.episode_index:06d}",
                         "tasks": [self.task], "language_instruction": self.task, "success": True,
                         "length": len(self.frames), "action_length": len(self.action_frames)})
        self._write_jsonl(episodes_path, episodes)
        info = {
            "codebase_version": "v2.4", "robot_type": "franka_fr3",
            "observation_state_layout": STATE_LAYOUT,
            "action_layout": ACTION_LAYOUT,
            "legacy_action_layout": LEGACY_ACTION_LAYOUT,
            "total_episodes": len(episodes),
            "total_frames": sum(row["length"] for row in episodes),
            "total_tasks": len(tasks), "fps": self.fps, "action_fps": self.action_fps,
            "data_path": "data/chunk-{chunk_index:03d}/episode_{episode_index:06d}.parquet",
            "action_path": "actions/chunk-{chunk_index:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{chunk_index:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": self.features,
        }
        (meta / "info.json").write_text(json.dumps(info, indent=2))
        self._write_stats(meta)

    def discard(self):
        """Release open files and remove artifacts for this uncommitted episode."""
        for writer in self.video_writers.values():
            writer.release()
        self.video_writers.clear()
        for camera in CAMERAS:
            self.raw_video_path(camera).unlink(missing_ok=True)
            self.video_path(camera).unlink(missing_ok=True)
        data_path = self.root / "data/chunk-000" / f"episode_{self.episode_index:06d}.parquet"
        data_path.unlink(missing_ok=True)
        action_path = self.root / "actions/chunk-000" / f"episode_{self.episode_index:06d}.parquet"
        action_path.unlink(missing_ok=True)
        self.frames.clear()
        self.action_frames.clear()


class EpisodeRecorder(Node):
    def __init__(self, root: Path, fps: int, action_fps: int, duration: float, task: str):
        super().__init__("lerobot_episode_recorder")
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("ffmpeg is required for MP4 finalization; rebuild the franka_vr image first")
        self.bridge, self.fps, self.action_fps, self.duration, self.task = CvBridge(), fps, action_fps, duration, task
        self.images, self.joints, self.joint_velocities = {}, {}, {}
        self.joint_timestamp = 0.0
        self.eef_position = self.eef_quaternion = None
        self.eef_timestamp = 0.0
        self.target_timestamp = 0.0
        self.command_requested = np.zeros(7, dtype=np.float32)
        self.command_sent = np.zeros(7, dtype=np.float32)
        self.command_requested_timestamp = self.command_sent_timestamp = 0.0
        # A target is an action command, not a measurement. Until the bridge
        # actually publishes one, preserve the missing value instead of
        # mislabelling an already-open gripper as commanded closed.
        self.gripper_target_width = float("nan")
        self.gripper_target_timestamp = 0.0
        self.previous_target = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target = np.zeros(7, dtype=np.float32)
        self.target[6] = 1.0
        self.start = None
        self.save_requested = False
        self.recording_complete = False
        self.finished = False
        self.x_button_pressed = False
        info_path = root / "meta" / "info.json"
        if info_path.is_file():
            existing_info = json.loads(info_path.read_text())
            if existing_info.get("codebase_version") != "v2.4":
                raise RuntimeError(
                    f"{root} is an existing {existing_info.get('codebase_version', 'unknown')} dataset. "
                    "Use a new dataset name; do not mix schemas with different gripper-target validity semantics."
                )
            old_state_shape = existing_info.get("features", {}).get("observation.state", {}).get("shape")
            if (
                old_state_shape != [8]
                or existing_info.get("observation_state_layout") != STATE_LAYOUT
                or existing_info.get("features", {}).get("action", {}).get("shape") != [8]
                or existing_info.get("action_layout") != ACTION_LAYOUT
                or existing_info.get("features", {}).get("gripper_target_width_valid", {}).get("dtype") != "bool"
            ):
                raise RuntimeError(
                    f"{root} has incompatible observation.state metadata (shape={old_state_shape}, "
                    f"layout={existing_info.get('observation_state_layout')!r}); this recorder writes "
                    "state=[fr3_joint1..7, measured gripper_width] and "
                    "action=[sent joint_velocity(7), gripper_target_width] (both shape [8]) with "
                    "gripper_target_width_valid. "
                    "Use a new dataset name rather than mixing incompatible episodes."
                )
        episode_indices = []
        for path in root.glob("data/chunk-*/episode_*.parquet"):
            try:
                episode_indices.append(int(path.stem.removeprefix("episode_")))
            except ValueError:
                continue
        episode_index = max(episode_indices, default=-1) + 1
        self.dataset = LeRobotEpisodeWriter(root, fps, action_fps, task, episode_index)
        self.get_logger().info(f"Saving as episode_{episode_index:06d} under {root}")
        self.create_subscription(Image, "/camera/camera_front/color/image_raw", self.image_cb("front"), 10)
        self.create_subscription(Image, "/camera_left/image_rect", self.image_cb("left"), 10)
        self.create_subscription(Image, "/camera_right/image_rect", self.image_cb("right"), 10)
        self.create_subscription(JointState, "/franka/joint_states", self.joint_cb, 20)
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 20)
        # On the real robot, the gripper driver does not add its feedback to
        # /joint_states.  Without this subscription the dataset silently
        # stored a zero in observation.state[-1].
        self.create_subscription(
            JointState, "/fr3_gripper/franka_gripper/joint_states", self.joint_cb, 20
        )
        self.create_subscription(Bool, "/vr/x_button", self.x_button_cb, 10)
        self.create_subscription(Float64, "/franka_vr/gripper_target_width_requested", self.gripper_target_cb, 10)
        self.create_subscription(Float64MultiArray, "/franka_vr/joint_velocity_command_requested", self.command_requested_cb, 20)
        self.create_subscription(Float64MultiArray, "/franka_vr/joint_velocity_command_sent", self.command_sent_cb, 20)
        # /tf uses best-effort QoS; the default reliable subscription does not
        # match it and silently receives no manual/VR action transforms.
        self.create_subscription(TFMessage, "/tf", self.tf_cb, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / fps, self.record_frame)
        self.get_logger().info(
            "Press A on the right Quest controller to save the current episode"
        )

    def x_button_cb(self, message):
        """Save once per right-controller face-button press, without motion."""
        pressed = bool(message.data)
        if pressed and not self.x_button_pressed and not self.save_requested:
            self.save_requested = True
            self.get_logger().info("Quest right-controller A button received: saving the current episode")
            if self.dataset.frames:
                self.finish_recording()
        self.x_button_pressed = pressed

    def image_cb(self, name):
        def callback(message):
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if frame.shape[:2] != (IMAGE_HEIGHT, IMAGE_WIDTH):
                frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
            self.images[name] = (frame, self.message_timestamp(message))
        return callback

    def joint_cb(self, message):
        self.joint_timestamp = self.message_timestamp(message)
        for index, (name, position) in enumerate(zip(message.name, message.position)):
            self.joints[name] = position
            if index < len(message.velocity):
                self.joint_velocities[name] = message.velocity[index]

    def message_timestamp(self, message):
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else self.get_clock().now().nanoseconds * 1e-9

    def gripper_target_cb(self, message):
        # FR3 Move uses total opening; retain the physical value and protect
        # against malformed producers before it becomes a training label.
        self.gripper_target_width = float(np.clip(message.data, 0.0, 0.08))
        self.gripper_target_timestamp = self.get_clock().now().nanoseconds * 1e-9

    def command_requested_cb(self, message):
        if len(message.data) == 8:
            self.command_requested_timestamp = float(message.data[0])
            self.command_requested = np.asarray(message.data[1:], dtype=np.float32)
        elif len(message.data) == 7:
            self.command_requested = np.asarray(message.data, dtype=np.float32)
            self.command_requested_timestamp = self.get_clock().now().nanoseconds * 1e-9

    def command_sent_cb(self, message):
        if len(message.data) == 8:
            self.command_sent_timestamp = float(message.data[0])
            self.command_sent = np.asarray(message.data[1:], dtype=np.float32)
        elif len(message.data) == 7:
            self.command_sent = np.asarray(message.data, dtype=np.float32)
            self.command_sent_timestamp = self.get_clock().now().nanoseconds * 1e-9
        else:
            return
        self.record_action(self.command_sent_timestamp)

    def tf_cb(self, message):
        for transform in message.transforms:
            if transform.child_frame_id in ("fr3_vr_target", "fr3_manual_target"):
                t = transform.transform.translation
                q = transform.transform.rotation
                self.target = np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w], dtype=np.float32)
                self.target_timestamp = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9
            elif transform.child_frame_id == "fr3_link8" and transform.header.frame_id == "world":
                t = transform.transform.translation
                q = transform.transform.rotation
                self.eef_position = np.array([t.x, t.y, t.z], dtype=np.float32)
                self.eef_quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float32)
                self.eef_timestamp = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9

    def ready(self):
        return (
            len(self.images) == len(CAMERAS)
            and all(name in self.joints for name in JOINTS)
            and all(name in self.joints for name in GRIPPER_JOINTS)
            and self.actual_eef_pose() is not None
        )

    def missing_inputs(self):
        """Name unavailable prerequisites so startup failures are actionable."""
        missing = []
        missing.extend(f"camera:{name}" for name in CAMERAS if name not in self.images)
        arm_joints = [name for name in JOINTS if name not in self.joints]
        if arm_joints:
            missing.append("arm_joint_state")
        gripper_joints = [name for name in GRIPPER_JOINTS if name not in self.joints]
        if gripper_joints:
            missing.append("gripper_joint_state")
        if self.actual_eef_pose() is None:
            missing.append("tf:world->fr3_link8")
        return missing

    def actual_eef_pose(self):
        """Look up the composed world->EEF feedback transform, not a target."""
        try:
            transform = self.tf_buffer.lookup_transform("world", "fr3_link8", Time(), timeout=Duration(seconds=0.02))
        except Exception:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        position = np.array([t.x, t.y, t.z], dtype=np.float32)
        quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float32)
        timestamp = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9
        return position, quaternion, timestamp

    def gripper_width(self):
        """Return the measured physical opening between the two fingers, in metres."""
        if not all(name in self.joints for name in GRIPPER_JOINTS):
            raise RuntimeError("gripper_width requested before both finger feedback values arrived")
        # The two positions are the displacement of each finger from the
        # centre line.  Summing their magnitudes supports both common URDF
        # sign conventions and uses feedback from both physical fingers.
        return sum(abs(float(self.joints[name])) for name in GRIPPER_JOINTS)

    @staticmethod
    def target_delta(previous, current):
        """Return requested Cartesian delta and xyz rotation-vector (radians)."""
        if previous is None:
            return np.zeros(3, dtype=np.float32), np.zeros(3, dtype=np.float32)
        position = current[:3] - previous[:3]
        # q_delta = inverse(q_previous) * q_current (xyzw): a TOOL-frame (fr3_link8) delta, so
        # q_current = q_previous * q_delta. Translation stays a world-frame difference.
        px, py, pz, pw = previous[3:]
        cx, cy, cz, cw = current[3:]
        x = cw * -px + cx * pw - cy * -pz + cz * -py
        y = cw * -py + cx * -pz + cy * pw - cz * -px
        z = cw * -pz - cx * -py + cy * -px + cz * pw
        w = cw * pw + cx * px + cy * py + cz * pz
        if w < 0.0:
            x, y, z, w = -x, -y, -z, -w
        norm = float(np.linalg.norm([x, y, z]))
        rotation = np.zeros(3, dtype=np.float32) if norm < 1e-8 else np.asarray(
            [x, y, z], dtype=np.float32) * (2.0 * np.arctan2(norm, w) / norm)
        return position.astype(np.float32), rotation

    def record_frame(self):
        if self.recording_complete:
            return
        if not self.ready():
            self.get_logger().info(
                "Waiting for required inputs; missing: " + ", ".join(self.missing_inputs()),
                throttle_duration_sec=2.0,
            )
            return
        if self.start is None:
            self.start = time.monotonic()
            self.get_logger().info(f"Recording '{self.task}' for {self.duration:.1f} s")
        width = self.gripper_width()
        actual_eef = self.actual_eef_pose()
        if actual_eef is None:
            return
        self.eef_position, self.eef_quaternion, self.eef_timestamp = actual_eef
        joint_position = np.array([self.joints[name] for name in JOINTS], dtype=np.float32)
        # Velocity is optional in ROS JointState. Persist NaN rather than
        # silently claiming zero velocity when a driver omits it.
        joint_velocity = np.array([self.joint_velocities.get(name, np.nan) for name in JOINTS], dtype=np.float32)
        delta_position_requested, delta_rotation_requested = self.target_delta(self.previous_target, self.target)
        self.previous_target = self.target.copy()
        # The command generator applies the same per-15-Hz limits before the
        # controller. Keep requested and sent deltas separately for auditing.
        delta_position_sent = delta_position_requested.copy()
        position_norm = np.linalg.norm(delta_position_sent)
        if position_norm > 0.003:
            delta_position_sent *= 0.003 / position_norm
        delta_rotation_sent = delta_rotation_requested.copy()
        rotation_norm = np.linalg.norm(delta_rotation_sent)
        if rotation_norm > 0.0175:
            delta_rotation_sent *= 0.0175 / rotation_norm
        state = np.concatenate((joint_position, [width])).astype(np.float32)
        action = np.concatenate((self.command_sent, [self.gripper_target_width])).astype(np.float32)
        legacy_action = np.concatenate((self.target, [width])).astype(np.float32)
        sample = {
            "record_timestamp": self.get_clock().now().nanoseconds * 1e-9,
            "state": state, "action": action, "legacy_action": legacy_action,
            "joint_position": joint_position, "joint_velocity": joint_velocity,
            "gripper_width": width, "eef_position": self.eef_position,
            "eef_quaternion": self.eef_quaternion,
            "joint_velocity_command_requested": self.command_requested,
            "joint_velocity_command_sent": self.command_sent,
            "gripper_target_width": self.gripper_target_width,
            "gripper_target_width_valid": bool(
                self.gripper_target_timestamp > 0.0 and np.isfinite(self.gripper_target_width)
            ),
            "eef_delta_position_command": delta_position_sent,
            "eef_delta_rotation_command": delta_rotation_sent,
            "timestamps": {
                "joint_state": self.joint_timestamp, "eef": self.eef_timestamp,
                "target": self.target_timestamp, "joint_velocity_command_requested": self.command_requested_timestamp,
                "joint_velocity_command_sent": self.command_sent_timestamp,
                "gripper_target_width": self.gripper_target_timestamp,
                "exterior_image_left": self.images["left"][1], "exterior_image_right": self.images["right"][1],
                "wrist_image": self.images["front"][1],
            },
            # Retain unbounded EEF operator intent as separate columns.
            "eef_delta_position_command_requested": delta_position_requested,
            "eef_delta_rotation_command_requested": delta_rotation_requested,
        }
        self.dataset.add_frame(sample, self.images)
        if self.save_requested:
            self.finish_recording()
        elif time.monotonic() - self.start >= self.duration:
            self.recording_complete = True
            self.timer.cancel()
            self.get_logger().info(
                "Maximum recording duration reached. Press A to save, or Ctrl+C to discard this episode"
            )

    def record_action(self, send_timestamp):
        """Record exactly one event for each command sent by the 80 Hz layer."""
        if self.recording_complete or self.start is None or not self.dataset.frames:
            return
        observation_index = -1
        for index in range(len(self.dataset.frames) - 1, -1, -1):
            if self.dataset.frames[index]["timestamp"] <= send_timestamp:
                observation_index = index
                break
        if observation_index < 0:
            return
        gripper_target_valid = bool(
            self.gripper_target_timestamp > 0.0 and np.isfinite(self.gripper_target_width)
        )
        action = np.concatenate((self.command_sent, [self.gripper_target_width])).astype(np.float32)
        self.dataset.add_action({
            "record_timestamp": send_timestamp,
            "action": action,
            "joint_velocity_command_requested": self.command_requested.copy(),
            "joint_velocity_command_sent": self.command_sent.copy(),
            "gripper_target_width": self.gripper_target_width,
            "gripper_target_width_valid": gripper_target_valid,
            "timestamps": {
                "joint_velocity_command_requested": self.command_requested_timestamp,
                "joint_velocity_command_sent": self.command_sent_timestamp,
                "gripper_target_width": self.gripper_target_timestamp,
            },
        }, observation_index)

    def finish_recording(self):
        """Commit the episode after the Quest A-button confirmation."""
        if self.finished:
            return
        self.timer.cancel()
        self.dataset.finalize()
        self.finished = True
        self.get_logger().info(
            f"Episode {self.dataset.episode_index:06d} saved; shutting down"
        )
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-name", default="fr3_teleop")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--action-fps", type=int, default=15)
    parser.add_argument(
        "--task",
        default="Move the FR3 end effector along a small smooth random trajectory",
    )
    # ros2 launch appends its own --ros-args. Keep those available to rclpy
    # while parsing only the recorder-specific arguments here.
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = EpisodeRecorder(
        Path("/datasets") / args.dataset_name, args.fps, args.action_fps, args.duration, args.task
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt received: discarding the unconfirmed episode")
    finally:
        if not node.finished:
            node.dataset.discard()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
