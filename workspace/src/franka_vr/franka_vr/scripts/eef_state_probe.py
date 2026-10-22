#!/usr/bin/env python3
"""Record what the EEF policy would observe, without commanding the arm.

Saves the exterior/wrist images exactly as eef_pose_policy_adapter.py prepares
them, and logs every gripper/arm feedback source separately so a wrong state
(e.g. width ~0 while the fingers are visibly open) can be traced to its topic:

  /fr3_gripper/franka_gripper/joint_states   franka_gripper node (real fingers)
  /joint_states                              joint_state_publisher merge (drives TF)
  /franka/joint_states                       ros2_control arm feedback

Optional --gripper-cycle (needs --confirm-gripper GRIPPER_MOTION) sends
[homing], open, close, open to the gripper and records the width throughout.
The arm is never commanded: no /set_target_pose client is created.
"""

from __future__ import annotations

import argparse
import functools
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from franka_msgs.action import Grasp, Homing, Move
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eef_pose_safety import build_state  # noqa: E402

CONFIRM_TOKEN = "GRIPPER_MOTION"
FINGERS = ("fr3_finger_joint1", "fr3_finger_joint2")
ARM_JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))
JOINT_SOURCES = {"gripper": "/fr3_gripper/franka_gripper/joint_states",
                 "joint_states": "/joint_states", "franka": "/franka/joint_states"}


class StateProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("eef_state_probe")
        self.args = args
        self.out = Path(args.out_dir); (self.out / "images").mkdir(parents=True, exist_ok=True)
        self.log_file = open(self.out / "probe.jsonl", "a", buffering=1)
        self.bridge = CvBridge()
        self.images: Dict[str, np.ndarray] = {}; self.image_info: Dict[str, Dict[str, Any]] = {}
        self.joints: Dict[str, Dict[str, Any]] = {}
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        for key, topic in (("exterior", args.exterior_image_topic), ("wrist", args.wrist_image_topic)):
            self.create_subscription(Image, topic, functools.partial(self.image_cb, key), qos_profile_sensor_data)
        for source, topic in JOINT_SOURCES.items():
            self.create_subscription(JointState, topic, functools.partial(self.joint_cb, source), qos_profile_sensor_data)
        self.started = time.monotonic(); self.saved = 0; self.next_image_at = self.started
        self.samples: List[Dict[str, Any]] = []; self.actions: List[Dict[str, Any]] = []
        self.steps = self.gripper_plan() if args.gripper_cycle else []
        self.step_index = 0; self.step_started = self.started; self.pending = False; self.sent = False
        self.done = False
        self.create_timer(1.0 / args.rate, self.sample)
        self.log("start", args=vars(args), topics=JOINT_SOURCES)
        self.get_logger().warning(f"state probe: {args.duration:.0f}s, images every {args.image_period}s -> {self.out}; "
                                  f"gripper cycle={'ON' if args.gripper_cycle else 'off'}; the arm is never commanded")

    # ----------------------------------------------------------------- inputs
    def image_cb(self, key: str, message: Image) -> None:
        try:
            # Identical preprocessing to eef_pose_policy_adapter.image_cb.
            image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(message, "bgr8"), cv2.COLOR_BGR2RGB)
            if image.shape[:2] != (480, 640): image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
            self.images[key] = np.ascontiguousarray(image, dtype=np.uint8)
            info = self.image_info.setdefault(key, {"count": 0})
            info.update(count=info["count"] + 1, time=time.monotonic(), encoding=message.encoding,
                        raw_size=[message.width, message.height], frame_id=message.header.frame_id)
        except Exception as error:
            self.get_logger().warning(f"cannot decode {key} image: {error}", throttle_duration_sec=2)

    def joint_cb(self, source: str, message: JointState) -> None:
        entry = self.joints.setdefault(source, {"count": 0, "names": []})
        entry.update(count=entry["count"] + 1, time=time.monotonic(), names=list(message.name),
                     position=dict(zip(message.name, map(float, message.position))),
                     stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    def width(self, source: str) -> Optional[float]:
        position = self.joints.get(source, {}).get("position", {})
        if any(name not in position for name in FINGERS): return None
        # Same total-opening definition as the adapter and lerobot_episode_recorder.
        return sum(abs(position[name]) for name in FINGERS)

    def pose(self) -> Optional[Dict[str, Any]]:
        try: transform = self.tf.lookup_transform("world", "fr3_link8", Time())
        except Exception as error: return {"error": str(error)}
        age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds * 1e-9
        t, q = transform.transform.translation, transform.transform.rotation
        return {"position": [t.x, t.y, t.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w], "age_s": age}

    # ----------------------------------------------------------------- gripper
    def gripper_plan(self) -> List[tuple]:
        a = self.args
        plan = [("baseline", None, a.baseline)]
        if a.homing: plan.append(("homing", "homing", a.settle))
        plan += [("open", "move", a.settle), ("close", "grasp", a.settle), ("open_again", "move", a.settle)]
        self.clients = {"homing": ActionClient(self, Homing, "/fr3_gripper/franka_gripper/homing"),
                        "move": ActionClient(self, Move, "/fr3_gripper/franka_gripper/move"),
                        "grasp": ActionClient(self, Grasp, "/fr3_gripper/franka_gripper/grasp")}
        return plan

    def phase(self) -> str:
        if not self.steps: return "observe"
        return self.steps[self.step_index][0] if self.step_index < len(self.steps) else "after_cycle"

    def advance_gripper(self, now: float) -> None:
        # One step at a time: send its goal, wait for the result, then observe `settle` seconds.
        if self.step_index >= len(self.steps) or self.pending: return
        name, kind, settle = self.steps[self.step_index]
        # A freshly started robot stack: the baseline only counts once the gripper node publishes.
        if kind is None and self.width("gripper") is None: self.step_started = now; return
        if kind is not None and not self.sent: self.send(now); return
        if now - self.step_started >= settle:
            self.step_index += 1; self.step_started = now; self.sent = False

    def send(self, now: float) -> None:
        name, kind, _ = self.steps[self.step_index]; a = self.args
        client = self.clients[kind]; self.sent = True
        if not client.wait_for_server(timeout_sec=2.0):
            self.record_action(name, kind, now, accepted=False, error="action server unavailable"); return
        if kind == "homing": goal = Homing.Goal()
        elif kind == "move": goal = Move.Goal(); goal.width = a.open_width; goal.speed = a.speed
        else:
            goal = Grasp.Goal(); goal.width = a.closed_width; goal.speed = a.speed; goal.force = a.force
            goal.epsilon.inner = a.epsilon_inner; goal.epsilon.outer = a.epsilon_outer
        self.pending = True
        entry = {"phase": name, "kind": kind, "sent_s": now - self.started, "width_before": self.width("gripper"),
                 "goal_width": getattr(goal, "width", None)}
        self.log("gripper_goal", **entry)
        self.get_logger().warning(f"gripper {name}: {kind} goal width={entry['goal_width']} (width now {entry['width_before']})")
        client.send_goal_async(goal).add_done_callback(functools.partial(self.goal_response, entry))

    def goal_response(self, entry: Dict[str, Any], future) -> None:
        try: handle = future.result()
        except Exception as error: self.finish_action(entry, accepted=False, error=str(error)); return
        if not handle.accepted: self.finish_action(entry, accepted=False, error="rejected"); return
        handle.get_result_async().add_done_callback(functools.partial(self.result_response, entry))

    def result_response(self, entry: Dict[str, Any], future) -> None:
        try: result = future.result().result
        except Exception as error: self.finish_action(entry, accepted=True, error=str(error)); return
        self.finish_action(entry, accepted=True, success=bool(result.success), error=str(result.error))

    def finish_action(self, entry: Dict[str, Any], accepted: bool, success: bool = False, error: str = "") -> None:
        now = time.monotonic()
        entry.update(accepted=accepted, success=success, error=error, done_s=now - self.started,
                     duration_s=now - self.started - entry["sent_s"], width_after=self.width("gripper"))
        self.record_action(entry.pop("phase"), entry.pop("kind"), now, **entry)

    def record_action(self, name: str, kind: str, now: float, **fields: Any) -> None:
        fields = {"phase": name, "kind": kind, **fields}
        self.actions.append(fields); self.log("gripper_result", **fields)
        self.get_logger().warning(f"gripper {name} result: {fields}")
        self.pending = False; self.step_started = now   # settle time counts from the result

    # ----------------------------------------------------------------- record
    def sample(self) -> None:
        now = time.monotonic(); elapsed = now - self.started
        if self.steps: self.advance_gripper(now)
        pose = self.pose()
        record: Dict[str, Any] = {"t": elapsed, "phase": self.phase(), "pose": pose, "sources": {}}
        for source in JOINT_SOURCES:
            entry = self.joints.get(source)
            if entry is None: record["sources"][source] = None; continue
            record["sources"][source] = {
                "age_s": now - entry["time"], "count": entry["count"], "stamp": entry["stamp"],
                "fingers": {name: entry["position"].get(name) for name in FINGERS},
                "width": self.width(source),
                "arm": [entry["position"].get(name) for name in ARM_JOINTS] if source != "gripper" else None}
        record["images"] = {key: {"age_s": now - info["time"], "count": info["count"], "encoding": info["encoding"],
                                  "raw_size": info["raw_size"]} for key, info in self.image_info.items()}
        if pose and "position" in pose:
            # The 7-D state the adapter would send, once per width source (binary Task0 gripper).
            midpoint = 0.5 * (self.args.gripper_state_closed_width + self.args.gripper_state_open_width)
            record["state"] = {source: build_state(pose["position"], pose["quaternion_xyzw"],
                                                   1.0 if self.width(source) >= midpoint else 0.0).tolist()
                               for source in ("gripper", "joint_states") if self.width(source) is not None}
        if now >= self.next_image_at and set(self.images) == {"exterior", "wrist"}:
            stem = f"img_{self.saved:04d}_{elapsed:06.2f}s_{record['phase']}"
            for key, image in self.images.items():
                cv2.imwrite(str(self.out / "images" / f"{stem}_{key}.jpg"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
            record["saved_images"] = stem; self.saved += 1; self.next_image_at = now + self.args.image_period
        self.samples.append(record); self.log("sample", **record)
        cycle_over = self.step_index >= len(self.steps) and not self.pending
        if elapsed >= self.args.duration and cycle_over: self.done = True

    def summary(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"duration_s": time.monotonic() - self.started, "images_saved": self.saved,
                               "gripper_actions": self.actions, "sources": {}}
        for source, topic in JOINT_SOURCES.items():
            widths = [s["sources"][source]["width"] for s in self.samples
                      if s["sources"].get(source) and s["sources"][source]["width"] is not None]
            entry = self.joints.get(source, {})
            out["sources"][source] = {
                "topic": topic, "messages": entry.get("count", 0),
                "rate_hz": entry.get("count", 0) / max(out["duration_s"], 1e-6),
                "names": entry.get("names", []),
                "width_first": widths[0] if widths else None, "width_last": widths[-1] if widths else None,
                "width_min": min(widths) if widths else None, "width_max": max(widths) if widths else None}
        pairs = [(s["sources"]["gripper"]["width"], s["sources"]["joint_states"]["width"]) for s in self.samples
                 if s["sources"].get("gripper") and s["sources"].get("joint_states")
                 and None not in (s["sources"]["gripper"]["width"], s["sources"]["joint_states"]["width"])]
        out["gripper_vs_joint_states_max_diff_m"] = max((abs(a - b) for a, b in pairs), default=None)
        out["images"] = self.image_info
        poses = [s["pose"] for s in self.samples if s["pose"] and "position" in s["pose"]]
        out["pose_first"] = poses[0] if poses else None; out["pose_last"] = poses[-1] if poses else None
        return out

    def log(self, event: str, **fields: Any) -> None:
        self.log_file.write(json.dumps({"monotonic_s": time.monotonic(), "event": event, **fields}, default=str) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add = parser.add_argument
    add("--out-dir", required=True)
    add("--exterior-image-topic", default="/camera_left/image_rect")
    add("--wrist-image-topic", default="/camera/camera_front/color/image_raw")
    add("--duration", type=float, default=20.0); add("--rate", type=float, default=20.0)
    add("--image-period", type=float, default=0.5, help="seconds between saved image pairs")
    add("--gripper-state-closed-width", type=float, default=0.01)
    add("--gripper-state-open-width", type=float, default=0.08)
    add("--gripper-cycle", action="store_true", help="[homing], open, close, open while recording")
    add("--confirm-gripper", default="")
    add("--homing", action="store_true", help="home the gripper first (fingers travel fully)")
    add("--baseline", type=float, default=3.0, help="seconds observed before the first gripper goal")
    add("--settle", type=float, default=2.0, help="seconds observed after each gripper result")
    add("--open-width", type=float, default=0.08); add("--closed-width", type=float, default=0.01)
    add("--speed", type=float, default=0.05); add("--force", type=float, default=20.0)
    add("--epsilon-inner", type=float, default=0.1); add("--epsilon-outer", type=float, default=0.2)
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if args.gripper_cycle and args.confirm_gripper != CONFIRM_TOKEN:
        parser.error(f"--gripper-cycle moves the gripper; add --confirm-gripper {CONFIRM_TOKEN}")
    if args.homing and not args.gripper_cycle: parser.error("--homing is part of --gripper-cycle")
    if not 0 <= args.closed_width < args.open_width <= .08: parser.error("gripper widths must satisfy 0 <= closed < open <= 0.08")
    if not 0 < args.speed <= .1 or not 1 <= args.force <= 40: parser.error("gripper speed/force out of range")
    return args


def main() -> None:
    args = parse_args(); rclpy.init(); node = StateProbe(args)
    try:
        while rclpy.ok() and not node.done: rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt: pass
    finally:
        summary = node.summary()
        (node.out / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
        print(json.dumps(summary, indent=2, default=str)); node.log_file.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
