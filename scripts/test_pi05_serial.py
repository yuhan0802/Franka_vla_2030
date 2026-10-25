"""Offline regression checks: no ROS publishers, hardware, or network."""
import ast
import collections
import os
import pathlib
import sys
import types
import unittest
from unittest.mock import Mock

import numpy as np

SCRIPTS = pathlib.Path(os.environ.get("PI05_SCRIPTS_DIR", str(
    pathlib.Path(__file__).resolve().parents[1] / "workspace/src/franka_vr/franka_vr/scripts"
)))
sys.path.insert(0, str(SCRIPTS))
from pi05_joint_velocity_safety import (
    JointVelocityGuard, validate_action_chunk, gripper_closedness, gripper_target_width,
)


class SerialTests(unittest.TestCase):
    def method(self, name, script="pi05_joint_velocity_adapter.py", class_name="Pi05JointVelocityAdapter"):
        tree = ast.parse((SCRIPTS / script).read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
        namespace = {
            "time": types.SimpleNamespace(monotonic=lambda: 10.0),
            "JointState": object,
            "JOINT_NAMES": tuple(f"fr3_joint{i}" for i in range(1, 8)),
            "closedness_from_width": lambda width, open_width, closed_width: max(
                0.0, min(1.0, (open_width - width) / (open_width - closed_width))),
        }
        exec(compile(ast.Module(body=[method], type_ignores=[]), "adapter_method", "exec"), namespace)
        return namespace[name]

    def direct_method(self, name):
        return self.method(name, "pi05_direct_policy_adapter.py", "DirectAdapter")

    def test_no_extra_scale_or_normalized_clipping(self):
        chunk = np.zeros((16, 8))
        chunk[0, 0] = 1.2
        self.assertEqual(validate_action_chunk(chunk)[0, 0], 1.2)
        action = np.array([0.01] * 7 + [0.5])
        guard = JointVelocityGuard(max_velocity=0.15, max_acceleration=0.5,
                                   action_units="direct-rad-s")
        result = guard.apply(action, [0, 0, 0, -1.5, 0, 2, 0], 1 / 15)
        np.testing.assert_allclose(result.raw_velocity, action[:7])
        np.testing.assert_allclose(result.velocity, action[:7])
        chunk[0, 0] = np.nan
        with self.assertRaises(ValueError):
            validate_action_chunk(chunk)

    def test_training_velocity_and_gripper_inverse(self):
        physical = np.array([0.01, -0.02, 0.03, -0.01, 0.02, -0.03, 0.01])
        action = np.r_[physical / 3.0, 0.75]
        result = JointVelocityGuard().apply(action, [0, 0, 0, -1.5, 0, 2, 0], 1 / 15)
        np.testing.assert_allclose(result.raw_velocity, physical)
        np.testing.assert_allclose(result.velocity, physical)
        for width in [0.0, 0.01, 0.02, 0.04, 0.08]:
            self.assertAlmostEqual(gripper_target_width(gripper_closedness(width)), width)

    def test_inference_waits_for_full_chunk_and_fresh_observations(self):
        node = types.SimpleNamespace(
            finished=False, fault_latched=False, gripper_busy=False,
            inference_future=None, args=types.SimpleNamespace(execute=True, max_actions=240, action_hz=15),
            executed_actions=16, actions=collections.deque([1]), last_action_time=9.0,
            gap_halted=True, joint_time=10.0, image_times={"left": 10.0, "wrist": 10.0},
            _observation=Mock(return_value={"fresh": True}),
            inference_pool=Mock(), policy=types.SimpleNamespace(infer=Mock()),
        )
        tick = self.method("_policy_tick")
        tick(node)
        node.inference_pool.submit.assert_not_called()
        node.actions.clear()
        node.last_action_time = 9.99
        tick(node)
        node.inference_pool.submit.assert_not_called()
        node.last_action_time = 9.0
        node.gap_halted = False
        tick(node)
        node.inference_pool.submit.assert_not_called()
        node.gap_halted = True
        node.image_times["left"] = 9.0
        tick(node)
        node.inference_pool.submit.assert_not_called()
        node.image_times["left"] = 10.0
        tick(node)
        node.inference_pool.submit.assert_called_once()

    def test_last_action_gets_full_period_before_stop(self):
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(execute=True, max_actions=16, action_hz=15),
            finished=False, fault_latched=False, gripper_busy=False,
            actions=collections.deque(), executed_actions=16, last_action_time=9.99,
            _finish_execution=Mock(),
        )
        tick = self.method("_action_tick")
        tick(node)
        node._finish_execution.assert_not_called()
        node.last_action_time = 9.0
        tick(node)
        node._finish_execution.assert_called_once()

    def test_direct_adapter_keeps_arm_and_gripper_feedback_sources_separate(self):
        arm_cb = self.direct_method("arm_joint_cb")
        gripper_cb = self.direct_method("gripper_joint_cb")
        arm_names = [f"fr3_joint{i}" for i in range(1, 8)]
        node = types.SimpleNamespace(joints={}, joint_time=0.0, fingers={}, gripper_time=0.0)

        # A merged/robot-state message containing misleading mimic fingers must
        # never overwrite the dedicated gripper measurement.
        arm_message = types.SimpleNamespace(
            name=arm_names + ["fr3_finger_joint1", "fr3_finger_joint2"],
            position=[float(i) for i in range(7)] + [0.035, 0.035],
        )
        arm_cb(node, arm_message)
        self.assertEqual(node.fingers, {})
        self.assertEqual(len(node.joints), 7)

        gripper_message = types.SimpleNamespace(
            name=["fr3_finger_joint1", "fr3_finger_joint2"],
            position=[0.071, 0.072],
        )
        gripper_cb(node, gripper_message)
        self.assertAlmostEqual(sum(node.fingers.values()), 0.143)
        arm_cb(node, arm_message)
        self.assertAlmostEqual(sum(node.fingers.values()), 0.143)

    def test_direct_adapter_replans_after_empty_grasp_instead_of_latching(self):
        result_cb = self.direct_method("gripper_result")
        node = types.SimpleNamespace(
            fingers={"fr3_finger_joint1": 0.001, "fr3_finger_joint2": 0.001},
            args=types.SimpleNamespace(
                gripper_open_width=0.08, gripper_closed_width=0.01,
                gripper_close_threshold=0.6, gripper_open_tolerance=0.01),
            pending_gripper_closed=True, last_gripper_closed=False, gripper_busy=True,
            queue=Mock(), need_fresh_after=0.0, log=Mock(), latch=Mock(),
            get_logger=lambda: types.SimpleNamespace(warning=Mock()),
        )
        future = Mock()
        future.result.return_value = types.SimpleNamespace(
            result=types.SimpleNamespace(success=False, error=""))
        result_cb(node, future)
        node.latch.assert_not_called()
        node.queue.clear_for_gripper.assert_called_once()
        self.assertFalse(node.gripper_busy)
        self.assertTrue(node.last_gripper_closed)
        self.assertIsNone(node.pending_gripper_closed)
        self.assertEqual(node.need_fresh_after, 10.0)


if __name__ == "__main__":
    unittest.main()
