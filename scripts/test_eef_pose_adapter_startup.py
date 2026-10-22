#!/usr/bin/env python3
"""Offline regression tests for the EEF adapter startup order (no ROS needed).

eef_pose_test_report_20261022: while arm() waited for TF, spin_once() fired the
tick timer and the axis test latched before TF existed.

eef_pose_tf_stability_report_20261022: right after launch, TF was driven by
joint_state_publisher's default joints; the adapter armed on that pose
([0.607, 0, 0.729]) and sent a target 0.42 m away from the real arm.
"""

from __future__ import annotations

import pathlib
import sys
import tempfile
import types
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "workspace/src/franka_vr/franka_vr/scripts"
sys.path.insert(0, str(SCRIPTS))

ARM = [f"fr3_joint{i}" for i in range(1, 8)]
DEFAULT_JOINTS = [0.0] * 7
REAL_JOINTS = [0.0, -0.4, 0.0, -2.4, 0.0, 2.0, 0.8]
# Poses taken from the TF stability report.
DEFAULT_POSE = ((0.6068832062, 0.0, 0.7285231268), (0.8929792022, 0.0, 0.4500979277, 0.0))
REAL_POSE = ((0.5555923705, -0.0011345801, 0.3160732120), (0.9250127665, -0.3794856352, -0.0176199627, 0.0056188382))
FAR_POSE = ((0.30, 0.20, 0.50), (1.0, 0.0, 0.0, 0.0))


class _Namespace(types.SimpleNamespace):
    """Attribute bag that also accepts keyword construction like ROS messages."""


class FakeTime:
    def __init__(self, nanoseconds: int = 0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp):
        return cls(stamp.nanoseconds)

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return None


class World:
    """Shared fake robot: clock, TF source and which joint feedback exists."""
    now_ns = 10**12
    spins = 0
    tf_after = 5            # spins before any TF exists
    real_after = 5          # spins before real /franka/joint_states arrive
    jump_at = None          # spin at which TF jumps to FAR_POSE
    targets: list = []

    @classmethod
    def reset(cls, tf_after=5, real_after=5, jump_at=None):
        cls.spins, cls.tf_after, cls.real_after, cls.jump_at, cls.targets = 0, tf_after, real_after, jump_at, []

    @classmethod
    def real(cls):
        return cls.spins >= cls.real_after

    @classmethod
    def pose(cls):
        if cls.jump_at is not None and cls.spins >= cls.jump_at:
            return FAR_POSE
        return REAL_POSE if cls.real() else DEFAULT_POSE


class FakeNode:
    def __init__(self, name):
        self._timers, self._subscriptions = [], {}

    def create_subscription(self, message_type, topic, callback, qos):
        self._subscriptions.setdefault(topic, []).append(callback)

    def create_timer(self, period, callback): self._timers.append(callback)
    def create_client(self, *args): return FakeServiceClient()
    def get_logger(self): return _Namespace(info=lambda *a, **k: None, warning=lambda *a, **k: None,
                                            error=lambda *a, **k: None)
    def get_clock(self): return _Namespace(now=lambda: FakeTime(World.now_ns))
    def destroy_node(self): return None

    def publish(self, topic, names, positions):
        for callback in self._subscriptions.get(topic, []):
            callback(_Namespace(name=list(names), position=list(positions)))


class FakeFuture:
    def __init__(self, value): self.value = value
    def add_done_callback(self, callback): callback(self)
    def result(self): return self.value


class FakeServiceClient:
    def service_is_ready(self): return True
    def wait_for_service(self, timeout_sec=None): return True

    def call_async(self, request):
        if not request.stop: World.targets.append(request)
        return FakeFuture(_Namespace(success=True, message="ok"))


class FakeBuffer:
    def lookup_transform(self, target, source, time):
        if World.spins < World.tf_after:
            raise RuntimeError('"world" passed to lookupTransform argument target_frame does not exist')
        (x, y, z), (qx, qy, qz, qw) = World.pose()
        return _Namespace(
            header=_Namespace(stamp=_Namespace(nanoseconds=World.now_ns)),
            transform=_Namespace(translation=_Namespace(x=x, y=y, z=z), rotation=_Namespace(x=qx, y=qy, z=qz, w=qw)))


def spin_once(node, timeout_sec=None):
    World.spins += 1
    World.now_ns += 10_000_000
    joints = REAL_JOINTS if World.real() else DEFAULT_JOINTS
    if World.real():
        node.publish("/franka/joint_states", ARM, REAL_JOINTS)
    # joint_state_publisher always publishes, with URDF defaults until real feedback arrives.
    node.publish("/joint_states", ARM + ["fr3_finger_joint1", "fr3_finger_joint2"], joints + [0.04, 0.04])
    node.publish("/fr3_gripper/franka_gripper/joint_states", ["fr3_finger_joint1", "fr3_finger_joint2"], [0.04, 0.04])
    for callback in list(node._timers):
        callback()


def _message(**defaults):
    def factory(**kwargs):
        values = {key: (value() if callable(value) else value) for key, value in defaults.items()}
        values.update(kwargs)
        return _Namespace(**values)
    return factory


def install_fake_ros() -> None:
    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.action", "rclpy.node", "rclpy.qos", "rclpy.time", "rclpy.utilities", "cv2",
        "cv_bridge", "franka_msgs", "franka_msgs.action", "geometry_msgs", "geometry_msgs.msg",
        "sensor_msgs", "sensor_msgs.msg", "tf2_ros", "franka_vr", "franka_vr.srv")}
    modules["rclpy"].spin_once = spin_once
    modules["rclpy"].utilities = modules["rclpy.utilities"]
    modules["rclpy.utilities"].remove_ros_args = lambda: list(sys.argv)
    modules["rclpy.action"].ActionClient = lambda *a: _Namespace(wait_for_server=lambda timeout_sec=None: True)
    modules["rclpy.node"].Node = FakeNode
    modules["rclpy.qos"].qos_profile_sensor_data = None
    modules["rclpy.time"].Time = FakeTime
    modules["cv_bridge"].CvBridge = lambda: None
    modules["franka_msgs.action"].Grasp = modules["franka_msgs.action"].Move = _Namespace
    modules["geometry_msgs.msg"].PoseStamped = _message(
        header=lambda: _Namespace(frame_id="", stamp=None),
        pose=lambda: _Namespace(position=_Namespace(x=0, y=0, z=0), orientation=None))
    modules["geometry_msgs.msg"].Quaternion = _message()
    modules["sensor_msgs.msg"].Image = modules["sensor_msgs.msg"].JointState = object
    modules["tf2_ros"].Buffer = FakeBuffer
    modules["tf2_ros"].TransformListener = lambda *a: None
    modules["franka_vr.srv"].SetTargetPose = _Namespace(Request=_message(stop=False, target_pose=None))
    sys.modules.update(modules)


def target_xyz(request):
    p = request.target_pose.pose.position
    return np.array([p.x, p.y, p.z])


class AdapterStartupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        install_fake_ros()
        import eef_pose_policy_adapter
        cls.adapter = eef_pose_policy_adapter

    def make_node(self, *extra, steps="1", magnitude="0.01", timeout="5"):
        bounds = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        bounds.write("workspace_bounds_world:\n  min: [0.2, -0.4, 0.1]\n  max: [0.8, 0.4, 0.9]\n"); bounds.close()
        sys.argv = ["adapter", "--bounds-config", bounds.name, "--test-axis", "+x", "--rotation-frame", "tool",
                    "--test-magnitude", magnitude, "--test-steps", steps, "--max-chunks", "1", "--max-actions", "15",
                    "--gripper-state-source", "binary", "--startup-timeout", timeout, *extra]
        return self.adapter.EefAdapter(self.adapter.parse_args())

    def run_until_done(self, node):
        node.arm()
        for _ in range(500):
            if node.finished or node.fault: break
            spin_once(node)

    EXECUTE = ("--execute", "--confirm-real-robot", "EEF_POSE_TARGET")

    def test_report_default_pose_is_never_used(self):
        # Reproduces the TF stability report: defaults for 0.5 s, then real feedback.
        World.reset(tf_after=2, real_after=50)
        node = self.make_node(*self.EXECUTE)
        self.run_until_done(node)
        self.assertFalse(node.fault)
        self.assertEqual(node.executed, 1)
        np.testing.assert_allclose(node.armed_pose[0], REAL_POSE[0])
        hold, motion = World.targets
        np.testing.assert_allclose(target_xyz(hold), REAL_POSE[0])
        np.testing.assert_allclose(target_xyz(motion), np.add(REAL_POSE[0], [0.01 * 0.075, 0, 0]), atol=1e-9)
        for request in World.targets:
            self.assertGreater(np.linalg.norm(target_xyz(request) - DEFAULT_POSE[0]), 0.3)

    def test_arming_requires_real_joint_feedback(self):
        World.reset(tf_after=2, real_after=10**9)   # TF from defaults, real feedback never arrives
        node = self.make_node(*self.EXECUTE, timeout="0.3")
        with self.assertRaisesRegex(RuntimeError, "franka/joint_states"):
            node.arm()
        self.assertEqual(World.targets, [])

    def test_late_tf_does_not_latch_before_arm(self):
        World.reset(tf_after=30, real_after=5)
        node = self.make_node(*self.EXECUTE)
        self.run_until_done(node)
        self.assertFalse(node.fault)
        self.assertEqual(node.executed, 1)

    def test_pose_jump_latches_before_next_target(self):
        World.reset(tf_after=2, real_after=5)
        node = self.make_node(*self.EXECUTE, steps="5", magnitude="0.04")
        node.arm()
        World.jump_at = World.spins + 10            # TF source switches mid-chunk
        for _ in range(500):
            if node.finished or node.fault: break
            spin_once(node)
        self.assertTrue(node.fault)
        self.assertLess(node.executed, 5)
        for request in World.targets:
            self.assertLess(np.linalg.norm(target_xyz(request) - REAL_POSE[0]), 0.02)

    def test_tick_is_inert_until_armed(self):
        World.reset(tf_after=10**9, real_after=10**9)
        node = self.make_node(*self.EXECUTE)
        for _ in range(20): spin_once(node)
        self.assertFalse(node.fault)
        self.assertEqual(node.executed, 0)
        self.assertEqual(World.targets, [])

    def test_dry_run_sends_nothing(self):
        World.reset(tf_after=2, real_after=40)
        node = self.make_node()
        self.run_until_done(node)
        self.assertFalse(node.fault)
        self.assertEqual(node.executed, 1)
        self.assertIsNone(node.target_client)
        self.assertEqual(World.targets, [])


if __name__ == "__main__":
    unittest.main()
