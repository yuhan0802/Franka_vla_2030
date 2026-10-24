#!/usr/bin/env python3
"""Closed-loop offline tests for eef_pose_direct_adapter (Servo-free EEF path).

Reuses the fake ROS layer of test_eef_pose_adapter_startup.py, but the simulated
arm is driven by the published JointJog velocities and TF is FK of its joints.
"""

from __future__ import annotations

import pathlib
import sys
import tempfile
import types
import unittest
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "workspace/src/franka_vr/franka_vr/scripts"))

import test_eef_pose_adapter_startup as harness  # noqa: E402
from test_eef_direct_kinematics import REAL_JOINTS  # noqa: E402

ARM = [f"fr3_joint{i}" for i in range(1, 8)]


class Sim:
    """Velocity-controlled FR3 fed by /pi05/direct_joint_velocity_cmd."""
    now_ns = 10**12
    joints = REAL_JOINTS.copy()
    velocity = np.zeros(7)
    commands: list = []
    executor_level = 0          # DiagnosticStatus.OK
    executor_message = "armed"
    feedback = True
    finger = 0.04
    exterior = True

    @classmethod
    def reset(cls):
        cls.joints, cls.velocity, cls.commands = REAL_JOINTS.copy(), np.zeros(7), []
        cls.executor_level, cls.executor_message, cls.feedback, cls.finger = 0, "armed", True, 0.04
        cls.exterior = True
        harness.World.now_ns = cls.now_ns


class FakePublisher:
    def publish(self, message):
        Sim.commands.append(np.array(message.velocities)); Sim.velocity = np.array(message.velocities)


class SimBuffer:
    def lookup_transform(self, target, source, time):
        from eef_direct_kinematics import forward_kinematics
        (x, y, z), (qx, qy, qz, qw), _ = forward_kinematics(Sim.joints)
        n = harness._Namespace
        return n(header=n(stamp=n(nanoseconds=Sim.now_ns)),
                 transform=n(translation=n(x=x, y=y, z=z), rotation=n(x=qx, y=qy, z=qz, w=qw)))


def sim_spin(node, timeout_sec=None):
    Sim.now_ns += 10_000_000
    harness.World.now_ns = Sim.now_ns      # the node's ROS clock
    Sim.joints = Sim.joints + Sim.velocity * 0.01
    if Sim.feedback:
        node.publish("/franka/joint_states", ARM, Sim.joints.tolist())
    node.publish("/joint_states", ARM + ["fr3_finger_joint1", "fr3_finger_joint2"], Sim.joints.tolist() + [Sim.finger] * 2)
    node.publish("/fr3_gripper/franka_gripper/joint_states", ["fr3_finger_joint1", "fr3_finger_joint2"], [Sim.finger] * 2)
    status = harness._Namespace(name="pi05_direct_velocity_executor", level=Sim.executor_level, message=Sim.executor_message)
    for callback in node._subscriptions.get("/pi05/direct_executor/status", []):
        callback(harness._Namespace(status=[status]))
    keys = ("exterior", "wrist") if Sim.exterior else ("wrist",)
    node.images = {key: np.zeros((480, 640, 3), np.uint8) for key in keys}
    node.image_times = {key: Sim.now_ns * 1e-9 for key in keys}
    for callback in list(node._timers):
        callback()


def install_direct_fakes():
    harness.install_fake_ros()
    for name in ("control_msgs", "control_msgs.msg", "diagnostic_msgs", "diagnostic_msgs.msg"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["control_msgs.msg"].JointJog = harness._message(
        header=lambda: harness._Namespace(stamp=None), joint_names=list, velocities=list, duration=0.0)
    sys.modules["diagnostic_msgs.msg"].DiagnosticArray = object
    sys.modules["diagnostic_msgs.msg"].DiagnosticStatus = harness._Namespace(OK=0, WARN=1, ERROR=2)
    sys.modules["rclpy"].spin_once = sim_spin
    sys.modules["tf2_ros"].Buffer = SimBuffer
    harness.FakeNode.create_publisher = lambda self, *args: FakePublisher()
    harness.FakeNode.destroy_client = lambda self, client: None


class DirectAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        install_direct_fakes()
        import eef_pose_policy_adapter, eef_pose_direct_adapter
        clock = types.SimpleNamespace(monotonic=lambda: Sim.now_ns * 1e-9, sleep=lambda s: None)
        eef_pose_policy_adapter.time = eef_pose_direct_adapter.time = clock
        eef_pose_policy_adapter.EefPolicyClient = harness.FakePolicy
        cls.base, cls.direct = eef_pose_policy_adapter, eef_pose_direct_adapter

    def setUp(self):
        Sim.reset(); harness.FakePolicy.chunks = []

    def node(self, *extra, execute=True):
        bounds = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        bounds.write("workspace_bounds_world:\n  min: [0.2, -0.4, 0.1]\n  max: [0.8, 0.4, 0.9]\n"); bounds.close()
        sys.argv = ["adapter", "--bounds-config", bounds.name, "--gripper-state-source", "binary",
                    "--max-chunks", "2", "--max-actions", "30", "--startup-timeout", "5", *extra]
        if execute:
            sys.argv += ["--execute", "--confirm-real-robot", "EEF_POSE_TARGET", "--confirm-direct", "DIRECT_EEF_VELOCITY"]
        direct = self.direct.parse_direct_args(); args = self.base.parse_args()
        return self.direct.EefDirectAdapter(args, direct)

    def run_node(self, node, spins=2000):
        node.arm()
        for _ in range(spins):
            if node.finished or node.fault: break
            sim_spin(node)
        for _ in range(5): sim_spin(node)
        node.destroy_node()

    def fk(self):
        from eef_direct_kinematics import forward_kinematics
        return forward_kinematics(Sim.joints)

    def test_policy_chunks_move_the_arm_along_the_commanded_direction(self):
        start, _, _ = self.fk()
        harness.FakePolicy.chunks = [harness.idle_chunk(1.0, dx=0.04)] * 2     # 3 mm/step along +x
        node = self.node()
        self.run_node(node)
        self.assertFalse(node.fault)
        self.assertEqual(node.chunks_done, 2)
        end, _, _ = self.fk()
        moved = end - start
        # 2 chunks x 15 steps x 3 mm = 90 mm commanded. Each chunk restarts from the measured pose once
        # settled within 3 mm (--chunk-start measured), so up to 3 mm per chunk is not travelled, as with Servo.
        self.assertGreater(moved[0], 0.09 - 2 * 0.003 - 0.0005); self.assertLess(moved[0], 0.09)
        self.assertLess(np.linalg.norm(moved[1:]), 0.002)
        self.assertTrue(np.allclose(Sim.commands[-1], 0.0), "must end with an explicit zero command")
        self.assertLessEqual(np.abs(np.array(Sim.commands)).max(), 0.5 + 1e-9)

    def test_last_target_chunk_start_keeps_every_commanded_millimetre(self):
        start, _, _ = self.fk()
        harness.FakePolicy.chunks = [harness.idle_chunk(1.0, dx=0.04)] * 2
        node = self.node("--chunk-start", "last_target")
        self.run_node(node)
        self.assertFalse(node.fault)
        moved = self.fk()[0] - start
        # only the final settle tolerance (3 mm) is left, never a per-chunk loss
        self.assertGreater(moved[0], 0.09 - 0.003 - 0.0005)
        np.testing.assert_allclose(node.integrator.position[0] - start[0], 0.09, atol=1e-6)

    def test_missing_camera_is_reported_instead_of_waiting_silently(self):
        Sim.exterior = False                       # 2026-10-23 20:09: rectify never got camera_info
        node = self.node(execute=False)
        node.arm()
        for _ in range(700): sim_spin(node)        # 7 s
        self.assertEqual(node.observation_count, 0)
        self.assertIn("no image yet on /camera_left/image_rect", node.observation_blocker)
        self.assertIsNotNone(node.observation_wait_since)
        self.assertGreater(node.observation_warned_at, 0)

    def test_dry_run_publishes_nothing(self):
        harness.FakePolicy.chunks = [harness.idle_chunk(1.0, dx=0.04)]
        node = self.node(execute=False)
        self.run_node(node)
        self.assertEqual(Sim.commands, [])
        np.testing.assert_allclose(Sim.joints, REAL_JOINTS)

    def test_execute_requires_direct_confirmation(self):
        sys.argv = ["adapter", "--bounds-config", "x.yaml", "--execute", "--confirm-real-robot", "EEF_POSE_TARGET"]
        with self.assertRaisesRegex(SystemExit, "confirm-direct"):
            self.direct.main()

    def test_executor_fault_latches_and_stops(self):
        harness.FakePolicy.chunks = [harness.idle_chunk(1.0, dx=0.04)] * 2
        node = self.node()
        node.arm()
        for _ in range(40): sim_spin(node)
        self.assertGreater(len(Sim.commands), 10)
        Sim.executor_level, Sim.executor_message = 2, "hard tracking error"
        for _ in range(3): sim_spin(node)
        self.assertTrue(node.fault)
        self.assertTrue(np.allclose(Sim.commands[-1], 0.0))
        count = len(Sim.commands)
        for _ in range(20): sim_spin(node)
        self.assertTrue(all(np.allclose(c, 0.0) for c in Sim.commands[count:]))

    def test_stale_joint_feedback_stops_tracking(self):
        harness.FakePolicy.chunks = [harness.idle_chunk(1.0, dx=0.04)] * 2
        node = self.node()
        node.arm()
        for _ in range(40): sim_spin(node)
        Sim.feedback = False
        for _ in range(30): sim_spin(node)
        self.assertTrue(node.fault)
        self.assertTrue(np.allclose(Sim.commands[-1], 0.0))

    def test_executor_not_armed_refuses_to_start(self):
        Sim.executor_level, Sim.executor_message = 1, "disarmed_no_velocity_publisher"
        node = self.node("--startup-timeout", "3")
        with self.assertRaisesRegex(RuntimeError, "set_target_pose unavailable"):
            node.arm()
        self.assertEqual(Sim.commands, [])


if __name__ == "__main__":
    unittest.main()
