#!/usr/bin/env python3
"""Offline test of eef_state_probe.py with fake ROS modules (needs numpy + cv2).

Reproduces eef_task1_act_20261022_154131: the gripper topic reports width ~5e-5
and every goal succeeds without the width changing. The probe must save the
image pairs, keep each feedback source separate and record the gripper cycle.
"""

from __future__ import annotations

import json
import pathlib
import sys
import tempfile
import types
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "workspace/src/franka_vr/franka_vr/scripts"))

ARM = [f"fr3_joint{i}" for i in range(1, 8)]
FINGERS = ["fr3_finger_joint1", "fr3_finger_joint2"]
POSE = ((0.5549, -0.0019, 0.3056), (0.925, -0.379, -0.025, 0.009))


class NS(types.SimpleNamespace):
    pass


class Clock:
    now = 0.0


def fake_monotonic():
    return Clock.now


class FakeTime:
    def __init__(self, nanoseconds=0): self.nanoseconds = nanoseconds
    @classmethod
    def from_msg(cls, stamp): return cls(stamp.nanoseconds)
    def __sub__(self, other): return FakeTime(self.nanoseconds - other.nanoseconds)


class FakeNode:
    def __init__(self, name): self.subs, self.timers = {}, []
    def create_subscription(self, kind, topic, callback, qos): self.subs.setdefault(topic, []).append(callback)
    def create_timer(self, period, callback): self.timers.append(callback)
    def get_logger(self): return NS(info=lambda *a, **k: None, warning=lambda *a, **k: None, error=lambda *a, **k: None)
    def get_clock(self): return NS(now=lambda: FakeTime(int(Clock.now * 1e9)))
    def destroy_node(self): pass

    def publish(self, topic, message):
        for callback in self.subs.get(topic, []): callback(message)


class Goal(NS):
    pass


GOALS = []


class FakeFuture:
    def __init__(self, value): self.value = value
    def add_done_callback(self, callback): callback(self)
    def result(self): return self.value


class FakeActionClient:
    def __init__(self, node, kind, name): self.name = name
    def wait_for_server(self, timeout_sec=None): return True

    def send_goal_async(self, goal):
        GOALS.append((self.name, getattr(goal, "width", None)))
        result = NS(result=NS(success=True, error=""))
        return FakeFuture(NS(accepted=True, get_result_async=lambda: FakeFuture(result)))


def install_fake_ros():
    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.action", "rclpy.node", "rclpy.qos", "rclpy.time", "rclpy.utilities", "cv_bridge",
        "franka_msgs", "franka_msgs.action", "sensor_msgs", "sensor_msgs.msg", "tf2_ros")}
    modules["rclpy"].utilities = modules["rclpy.utilities"]
    modules["rclpy.utilities"].remove_ros_args = lambda: list(sys.argv)
    modules["rclpy.action"].ActionClient = FakeActionClient
    modules["rclpy.node"].Node = FakeNode
    modules["rclpy.qos"].qos_profile_sensor_data = None
    modules["rclpy.time"].Time = lambda *a: None
    modules["rclpy.time"].Time.from_msg = FakeTime.from_msg
    modules["cv_bridge"].CvBridge = lambda: NS(imgmsg_to_cv2=lambda message, encoding: message.data)
    for name in ("Grasp", "Homing", "Move"):
        setattr(modules["franka_msgs.action"], name, NS(Goal=lambda: Goal(epsilon=NS())))
    modules["sensor_msgs.msg"].Image = modules["sensor_msgs.msg"].JointState = object
    (x, y, z), (qx, qy, qz, qw) = POSE
    transform = NS(header=NS(stamp=NS(nanoseconds=0)),
                   transform=NS(translation=NS(x=x, y=y, z=z), rotation=NS(x=qx, y=qy, z=qz, w=qw)))
    modules["tf2_ros"].Buffer = lambda: NS(lookup_transform=lambda *a: transform)
    modules["tf2_ros"].TransformListener = lambda *a: None
    sys.modules.update(modules)


def joint_msg(names, positions):
    return NS(name=list(names), position=list(positions), header=NS(stamp=NS(sec=0, nanosec=0)))


def image_msg(value, shape):
    return NS(data=np.full(shape, value, np.uint8), encoding="rgb8", width=shape[1], height=shape[0],
              header=NS(frame_id="cam"))


class ProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        install_fake_ros()
        import eef_state_probe
        eef_state_probe.time.monotonic = fake_monotonic
        cls.probe = eef_state_probe

    def run_probe(self, *extra, steps=2000):
        GOALS.clear(); Clock.now = 100.0
        out = tempfile.mkdtemp()
        sys.argv = ["probe", "--out-dir", out, "--duration", "2", "--image-period", "0.5", *extra]
        node = self.probe.StateProbe(self.probe.parse_args())
        for _ in range(steps):
            if node.done: break
            Clock.now += 0.05
            node.publish("/camera_left/image_rect", image_msg(80, (480, 640, 3)))
            node.publish("/camera/camera_front/color/image_raw", image_msg(160, (720, 1280, 3)))
            # As in the failed run: the real gripper topic says ~closed, the merge disagrees.
            node.publish("/fr3_gripper/franka_gripper/joint_states", joint_msg(FINGERS, [2.6e-5, 2.6e-5]))
            node.publish("/joint_states", joint_msg(ARM + FINGERS, [0.1] * 7 + [0.035, 0.035]))
            node.publish("/franka/joint_states", joint_msg(ARM, [0.1] * 7))
            for timer in node.timers: timer()
        self.assertTrue(node.done)
        summary = node.summary(); node.log_file.close()
        return pathlib.Path(out), summary

    def test_observe_only_saves_images_and_separates_sources(self):
        out, summary = self.run_probe()
        self.assertEqual(GOALS, [])
        images = sorted((out / "images").glob("*.jpg"))
        self.assertGreaterEqual(len(images), 6)
        self.assertTrue(any(p.name.endswith("_exterior.jpg") for p in images))
        self.assertTrue(any(p.name.endswith("_wrist.jpg") for p in images))
        import cv2
        wrist = cv2.imread(str(next(p for p in images if p.name.endswith("_wrist.jpg"))))
        self.assertEqual(wrist.shape, (480, 640, 3))      # resized exactly like the adapter
        self.assertAlmostEqual(summary["sources"]["gripper"]["width_max"], 5.2e-5)
        self.assertAlmostEqual(summary["sources"]["joint_states"]["width_max"], 0.07)
        self.assertAlmostEqual(summary["gripper_vs_joint_states_max_diff_m"], 0.07 - 5.2e-5)
        sample = next(json.loads(line) for line in open(out / "probe.jsonl") if '"event": "sample"' in line
                      and '"state"' in line)
        self.assertEqual(sample["state"]["gripper"][6], 0.0)
        self.assertEqual(sample["state"]["joint_states"][6], 1.0)

    def test_gripper_cycle_records_each_goal(self):
        out, summary = self.run_probe("--gripper-cycle", "--confirm-gripper", "GRIPPER_MOTION", "--homing",
                                      "--baseline", "0.5", "--settle", "0.5")
        self.assertEqual([name.rsplit("/", 1)[1] for name, _ in GOALS], ["homing", "move", "grasp", "move"])
        phases = [a["phase"] for a in summary["gripper_actions"]]
        self.assertEqual(phases, ["homing", "open", "close", "open_again"])
        for action in summary["gripper_actions"]:
            self.assertTrue(action["success"]); self.assertAlmostEqual(action["width_after"], 5.2e-5)

    def test_gripper_cycle_requires_confirmation(self):
        sys.argv = ["probe", "--out-dir", tempfile.mkdtemp(), "--gripper-cycle"]
        with self.assertRaises(SystemExit): self.probe.parse_args()


if __name__ == "__main__":
    unittest.main()
