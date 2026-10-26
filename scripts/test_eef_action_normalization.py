"""Test server action decoding without Torch, model weights, ROS or hardware."""
import ast
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT / "deploy_franka_task0_qwen_actioncodec_eef.py").read_text())
function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "denormalize_actions")
namespace = {"np": np}
exec(compile(ast.Module(body=[function], type_ignores=[]), "server_action_decoder", "exec"), namespace)
decode = namespace["denormalize_actions"]


class ActionNormalizationTests(unittest.TestCase):
    def setUp(self):
        self.scale = np.array([.1179361343, .2155586034, .3562303483, .2308643013, .3947671354, .2936727107])

    def test_mulaw9_roundtrip_matches_training_transform(self):
        physical = np.random.default_rng(42).uniform(-1, 1, (15, 7))
        physical[:, :6] *= self.scale
        physical[:, 6] = np.random.default_rng(43).uniform(0, 1, 15)
        normalized = physical.copy()
        normalized[:, :6] /= self.scale
        normalized[:, :6] = np.sign(normalized[:, :6]) * np.log1p(9 * np.abs(normalized[:, :6])) / np.log1p(9)
        normalized[:, 6] = 2 * physical[:, 6] - 1
        result = decode(normalized, self.scale, "franka_mulaw", 9)
        np.testing.assert_allclose(result, physical, atol=2e-7)

    def test_inverse_is_nonlinear_and_does_not_apply_to_gripper(self):
        normalized = np.full((15, 7), .5)
        result = decode(normalized, np.ones(6), "franka_mulaw", 9)
        np.testing.assert_allclose(result[:, :6], (np.sqrt(10) - 1) / 9, atol=1e-7)
        np.testing.assert_allclose(result[:, 6], .75)

    def test_codec_output_is_clipped_before_mulaw_inverse(self):
        raw = np.array([[2, -2, 1, -1, 0, 0, 3]], dtype=float)
        result = decode(raw, self.scale, "franka_mulaw", 9)
        np.testing.assert_allclose(result[0, :6], self.scale * [1, -1, 1, -1, 0, 0], atol=1e-7)
        self.assertEqual(result[0, 6], 1)

    def test_existing_linear_models_retain_their_decoding(self):
        raw = np.random.default_rng(42).uniform(-1.2, 1.2, (15, 7)).astype(np.float32)
        expected = raw.copy()
        expected[:, :6] *= self.scale.astype(np.float32)
        expected[:, 6] = (expected[:, 6] + 1) / 2
        for method in ("task0_maxabs", "droid_gain", "legacy_droid_symmetric_scale", "franka_maxabs"):
            np.testing.assert_array_equal(decode(raw, self.scale, method), expected)

    def test_invalid_mu_or_nonfinite_actions_are_rejected(self):
        for mu in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError): decode(np.zeros((15, 7)), self.scale, "franka_mulaw", mu)
        with self.assertRaises(ValueError): decode(np.full((15, 7), np.nan), self.scale, "franka_mulaw", 9)


if __name__ == "__main__":
    unittest.main()
