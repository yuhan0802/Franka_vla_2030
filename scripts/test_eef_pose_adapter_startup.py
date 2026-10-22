#!/usr/bin/env python3
"""Offline regression test for the EEF adapter startup order (no ROS needed).

Reproduces eef_pose_test_report_20261022: while arm() waits for world ->
fr3_link8, rclpy.spin_once() also fires the 100 Hz tick timer. Before the fix
the axis-test chunk started immediately and latched "end-effector pose
unavailable at chunk start" with zero actions executed.
"""

from __future__ import annotations

import pathlib
import sys
import tempfile
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "workspace/src/franka_vr/franka_vr/scripts"
sys.path.insert(0, str(SCRIPTS))


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
    """Shared fake clock and TF availability."""
    now_ns = 10**12
    spins = 0
    tf_ready_after_spins = 5
    targets: list = []


class FakeNode:
    def __init__(self, name):
        self._timers = []

    def create_subscription(self, *args, **kwargs): return None
    def create_timer(self, period, callback): self._timers.append(callback)
    def create_client(self, *args): return FakeServiceClient()
    def get_logger(self): return _Namespace(info=lambda *a, **k: None, warning=lambda *a, **k: None,
                                            error=lambda *a, **k: None)
    def get_clock(self): return _Namespace(now=lambda: FakeTime(World.now_ns))
    def destroy_node(self): return None


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
        if World.spins < World.tf_ready_after_spins:
            raise RuntimeError('"world" passed to lookupTransform argument target_frame does not exist')
        return _Namespace(
            header=_Namespace(stamp=_Namespace(nanoseconds=World.now_ns)),
            transform=_Namespace(translation=_Namespace(x=0.5, y=0.0, z=0.4),
                                 rotation=_Namespace(x=1.0, y=0.0, z=0.0, w=0.0)))


def spin_once(node, timeout_sec=None):
    World.spins += 1
    World.now_ns += 10_000_000
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


class AdapterStartupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        install_fake_ros()
        import eef_pose_policy_adapter
        cls.adapter = eef_pose_policy_adapter

    def make_node(self, *extra):
        bounds = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        bounds.write("workspace_bounds_world:\n  min: [0.3, -0.3, 0.1]\n  max: [0.8, 0.3, 0.8]\n"); bounds.close()
        sys.argv = ["adapter", "--bounds-config", bounds.name, "--test-axis", "+x", "--rotation-frame", "tool",
                    "--test-magnitude", "0.01", "--test-steps", "1", "--max-chunks", "1", "--max-actions", "15",
                    "--gripper-state-source", "command", "--startup-timeout", "5", *extra]
        World.spins = 0; World.targets = []
        return self.adapter.EefAdapter(self.adapter.parse_args())

    def run_until_done(self, node):
        node.arm()
        for _ in range(200):
            if node.finished or node.fault: break
            spin_once(node)

    def test_late_tf_does_not_latch_before_arm(self):
        World.tf_ready_after_spins = 5
        node = self.make_node("--execute", "--confirm-real-robot", "EEF_POSE_TARGET")
        self.run_until_done(node)
        self.assertFalse(node.fault)
        self.assertTrue(node.finished)
        self.assertEqual(node.executed, 1)
        self.assertEqual(len(World.targets), 1)
        target = World.targets[0].target_pose
        self.assertEqual(target.header.frame_id, "world")
        self.assertAlmostEqual(target.pose.position.x, 0.5 + 0.01 * 0.075)

    def test_tick_is_inert_until_armed(self):
        World.tf_ready_after_spins = 10**9
        node = self.make_node("--execute", "--confirm-real-robot", "EEF_POSE_TARGET")
        for _ in range(20): spin_once(node)
        self.assertFalse(node.fault)
        self.assertEqual(node.executed, 0)
        self.assertEqual(World.targets, [])

    def test_dry_run_sends_nothing(self):
        World.tf_ready_after_spins = 3
        node = self.make_node()
        self.run_until_done(node)
        self.assertFalse(node.fault)
        self.assertEqual(node.executed, 1)
        self.assertIsNone(node.target_client)
        self.assertEqual(World.targets, [])


if __name__ == "__main__":
    unittest.main()
