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
    def method(self, name):
        tree = ast.parse((SCRIPTS / "pi05_joint_velocity_adapter.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Pi05JointVelocityAdapter")
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
        namespace = {"time": types.SimpleNamespace(monotonic=lambda: 10.0)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), "adapter_method", "exec"), namespace)
        return namespace[name]

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


if __name__ == "__main__":
    unittest.main()
