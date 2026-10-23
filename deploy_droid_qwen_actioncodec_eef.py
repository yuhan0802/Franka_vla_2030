#!/usr/bin/env python3
"""WebSocket server for the DROID-pretrained Qwen3-VL ActionCodec EEF model (state-first BAR).

Same wire protocol as deploy_franka_task0_qwen_actioncodec_eef.py, so
eef_pose_policy_adapter.py / eef_pose_direct_adapter.py drive it unchanged. The
model contract is imported from or copied verbatim from the DROID training code:
  train/droid_qwen_actioncodec_state_first.py        (DROID_STATE_MODE=eef_axis_angle6_gripper1)
  droid_qwen3vl_actioncodec/droid_data.py            (DroidDataset._transform, pi05_droid_two_view)
  droid_qwen3vl_actioncodec/eef_state_model.py       (VLAWithState, 7-D EEF state)

Differences from the Task0 server, all following the DROID training data:
  * prompt: two unlabeled images (one exterior + wrist), "Current arm joints:" text, DROID system prompt
  * gripper state: continuous 1 - gripper_position (1=open), not Task0's binary width threshold
  * rotation: DROID cartesian_velocity is a base-frame command, executed by left multiplication
  * any natural-language instruction (DROID has many tasks; there is no single training prompt)
"""

import argparse
import json
import os
import sys
import time
import types
from pathlib import Path

os.environ["DROID_STATE_MODE"] = "eef_axis_angle6_gripper1"
os.environ["DROID_ACTION_PREDICTION_MODE"] = "block_ar_16"

import numpy as np
import torch
from PIL import Image
from transformers import AutoModel, AutoModelForImageTextToText, AutoProcessor

PROJECT_ROOT = Path(os.environ.get("VLA_PROJECT_ROOT", "/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA"))
sys.path.insert(0, str(PROJECT_ROOT))
# The training modules import W&B only for their training entry points.
sys.modules.setdefault("wandb", types.ModuleType("wandb"))
from franka_vr_3view_overfit.franka_vr_qwen_actioncodec import (  # noqa: E402
    DROID_Q01, DROID_Q99, ActionTokenizer, restore_actioncodec_derived_buffers, restore_legacy_wrapped_embeddings,
)
from droid_qwen3vl_actioncodec.eef_state_model import (  # noqa: E402
    FUTURE_STATE_END_TOKEN, FUTURE_STATE_START_TOKEN, FUTURE_STATE_TOKENS, STATE_TOKENS, VLAWithState,
)
from train.droid_qwen_actioncodec_state_first_bar import BAR_ACTION_TOKENS, generate_action_codes_bar  # noqa: E402

BAR_BOS_TOKEN = "<|action_bar_bos|>"  # train/droid_qwen_actioncodec_state_first.py
STATE_PROTOCOL = "droid_eef_axis_angle6_gripper_open1_singleturn_v1"
ACTION_REPRESENTATION = "droid_base_cartesian_velocity_plus_inverted_gripper_v1"
# DROID cartesian_velocity: base-frame linear/angular velocity per 15 Hz step.
ROTATION_EXECUTION = "relative_rotvec_left_multiply"
IMAGE_SHAPE = (480, 640, 3)   # adapter request shape
MODEL_IMAGE_SIZE = 256        # droid_data.decode_image: RGB, bilinear 256x256
SYSTEM_PROMPT = ("You are a robot policy. Given the images and current robot state, "
                 "predict the next state and action for the task.")


class DroidEefPolicy:
    def __init__(self, args: argparse.Namespace):
        self.device = torch.device(args.device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(f"CUDA unavailable: {self.device}")
        checkpoint = Path(args.checkpoint).resolve()
        self.training = json.loads((checkpoint / "training_config.json").read_text())
        expected = {
            "base_model": "qwen3vl2b", "state_protocol": STATE_PROTOCOL,
            "action_representation": ACTION_REPRESENTATION, "action_horizon": 15, "action_embodiment_id": 2,
            "action_prediction_mode": "block_ar_16", "output_protocol": "future_state_then_action_v1",
            "camera_mode": "pi05_droid_two_view",
        }
        mismatched = {key: self.training.get(key) for key, value in expected.items() if self.training.get(key) != value}
        if mismatched:
            raise RuntimeError(f"checkpoint is not the DROID EEF state-first BAR protocol: {mismatched}")

        self.processor = AutoProcessor.from_pretrained(checkpoint, trust_remote_code=True, local_files_only=True)
        vla = AutoModelForImageTextToText.from_pretrained(
            checkpoint, dtype=torch.bfloat16, low_cpu_mem_usage=True, trust_remote_code=True, local_files_only=True,
        ).to(self.device)
        self.codec_path = Path(args.actioncodec or self.training["actioncodec_path"])
        codec = AutoModel.from_pretrained(self.codec_path, trust_remote_code=True, local_files_only=True)
        restore_actioncodec_derived_buffers(codec)
        tokenizer = self.processor.tokenizer
        action_tokens = [f"<|action_{index}|>" for index in range(int(codec.config.vq_codebook_size))]
        required = (["<|action_start|>", "<|action_end|>", BAR_BOS_TOKEN, FUTURE_STATE_START_TOKEN,
                     FUTURE_STATE_END_TOKEN] + action_tokens + STATE_TOKENS + FUTURE_STATE_TOKENS)
        vocabulary = tokenizer.get_vocab()
        missing = [token for token in required if token not in vocabulary]
        if missing:
            raise RuntimeError(f"checkpoint tokenizer lacks training tokens: {missing[:5]}")
        if vla.get_input_embeddings().weight.shape[0] < len(tokenizer):
            raise RuntimeError("checkpoint embeddings are smaller than its tokenizer")
        restore_legacy_wrapped_embeddings(vla, checkpoint)
        self.action_token_ids = tokenizer.convert_tokens_to_ids(action_tokens)
        self.action_tokenizer = ActionTokenizer(codec, self.action_token_ids, 2)
        self.id_to_code = {token_id: code for code, token_id in enumerate(self.action_token_ids)}
        self.bos_id = tokenizer.convert_tokens_to_ids(BAR_BOS_TOKEN)
        self.action_start = tokenizer.convert_tokens_to_ids("<|action_start|>")
        self.model = VLAWithState(
            vla, tokenizer.convert_tokens_to_ids(STATE_TOKENS), tokenizer.convert_tokens_to_ids(FUTURE_STATE_TOKENS),
        ).to(device=self.device, dtype=torch.bfloat16)
        self.model.state_encoder.load_state_dict(
            torch.load(checkpoint / "state_encoder.pth", map_location="cpu", weights_only=True))
        self.model.future_state_predictor.load_state_dict(
            torch.load(checkpoint / "future_state_predictor.pth", map_location="cpu", weights_only=True))
        self.model.eval()

        self.stats_path = Path(args.state_stats or self.training["state_stats_path"])
        stats = json.loads(self.stats_path.read_text())["norm_stats"]["state"]
        self.q01 = np.asarray(stats["q01"], dtype=np.float32)
        self.q99 = np.asarray(stats["q99"], dtype=np.float32)
        if self.q01.shape != (7,) or self.q99.shape != (7,):
            raise RuntimeError(f"DROID EEF state stats must be 7D: {self.stats_path}")
        span = self.q99 - self.q01
        self.constant_dims = span < 1e-8   # DroidDataset._normalize_state
        self.safe_span = np.where(self.constant_dims, 1.0, span)
        self.scale = np.maximum(np.abs(DROID_Q01), np.abs(DROID_Q99))[:6]
        self.dummy_action = "<|action_0|>" * BAR_ACTION_TOKENS
        self.checkpoint = checkpoint

    def _prompt(self, raw):
        state = np.asarray(raw["eef_state"], dtype=np.float32).reshape(-1)
        if state.shape != (7,) or not np.isfinite(state).all() or not 0.0 <= state[6] <= 1.0:
            raise ValueError(f"eef_state must be finite (7,) with gripper in [0, 1], got {state!r}")
        images = []
        for key in ("exterior_image", "wrist_image"):
            image = np.asarray(raw[key], dtype=np.uint8)
            if image.shape != IMAGE_SHAPE:
                raise ValueError(f"{key} must be {IMAGE_SHAPE} RGB, got {image.shape}")
            images.append(Image.fromarray(image, "RGB").resize(
                (MODEL_IMAGE_SIZE, MODEL_IMAGE_SIZE), Image.Resampling.BILINEAR))
        instruction = str(raw.get("instruction", "")).strip()
        if not instruction:
            raise ValueError("DROID policy requires a non-empty instruction")
        normalized = 2.0 * (state - self.q01) / self.safe_span - 1.0
        normalized[self.constant_dims] = 0.0
        normalized = np.clip(normalized, -1.0, 1.0).astype(np.float32)
        # Identical to DroidDataset._transform with camera_mode=pi05_droid_two_view.
        user = [
            {"type": "image", "image": images[0]}, {"type": "image", "image": images[1]},
            {"type": "text", "text": (
                f"Task: {instruction.lower()}\n"
                f"Current arm joints: {STATE_TOKENS[0]}\n"
                f"Current gripper state: {STATE_TOKENS[1]}")},
        ]
        assistant = (f"Next state:\n{FUTURE_STATE_START_TOKEN}{''.join(FUTURE_STATE_TOKENS)}"
                     f"{FUTURE_STATE_END_TOKEN}\nAction:\n<|action_start|>{self.dummy_action}<|action_end|>")
        messages = [{"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": user}, {"role": "assistant", "content": assistant}]
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=False)
        encoded = self.processor(text=[text], images=images, return_tensors="pt")
        positions = torch.nonzero(encoded.input_ids[0] == self.action_start, as_tuple=False).flatten()
        if positions.numel() != 1:
            raise RuntimeError("could not locate <|action_start|> in the DROID prompt")
        return encoded, int(positions[0]) + 1, torch.from_numpy(normalized).unsqueeze(0)

    @torch.inference_mode()
    def predict(self, raw) -> np.ndarray:
        encoded, end, state = self._prompt(raw)
        prefix = encoded.input_ids[:, :end].to(self.device)
        with torch.autocast("cuda", dtype=torch.bfloat16, enabled=self.device.type == "cuda"):
            token_ids = generate_action_codes_bar(
                self.model, prefix, state.to(self.device), torch.ones_like(prefix),
                encoded.pixel_values.to(self.device, dtype=torch.bfloat16),
                encoded.image_grid_thw.to(self.device), self.action_token_ids, self.bos_id)
        codes = [self.id_to_code[int(token_id)] for token_id in token_ids]
        decoded, padding_mask = self.action_tokenizer.codec.decode([codes], embodiment_ids=2)
        normalized = np.asarray(decoded, dtype=np.float32)[0, :15, :7]
        if normalized.shape != (15, 7) or not np.all(np.asarray(padding_mask)[0, :15]):
            raise RuntimeError(f"unexpected ActionCodec decode shape {normalized.shape}")
        action = normalized.copy()
        action[:, :6] *= self.scale          # inverse of ActionTokenizer.normalize
        action[:, 6] = (action[:, 6] + 1.0) / 2.0
        if not np.isfinite(action).all():
            raise RuntimeError("decoded action contains NaN/Inf")
        return action

    def act(self, observation) -> torch.Tensor:
        return torch.from_numpy(self.predict(observation).astype(np.float32))

    def reset(self) -> None:
        pass

    def metadata(self, port: int) -> dict:
        return {
            "model_family": "qwen3vl2b_droid_state_first_bar_eef",
            "checkpoint": str(self.checkpoint),
            "actioncodec": str(self.codec_path), "state_stats": str(self.stats_path),
            "state_protocol": STATE_PROTOCOL,
            "action_representation": ACTION_REPRESENTATION,
            "action_shape": [15, 7], "action_hz": 15,
            "gripper_convention": "0_closed_1_open",
            # DROID state is the continuous opening (1 - gripper_position), not a threshold.
            "gripper_state_source": "measured",
            "rotation_execution": ROTATION_EXECUTION,
            "observation_format": "droid_exterior_wrist_640x480_eef7_v1",
            "model_image_size": [MODEL_IMAGE_SIZE, MODEL_IMAGE_SIZE],
            "training_instruction": "",   # DROID: any instruction
            "port": port,
        }


def warm_up(policy: DroidEefPolicy) -> tuple:
    blank = np.zeros(IMAGE_SHAPE, dtype=np.uint8)
    state = np.array([0.5, 0.0, 0.4, 3.1, 0.0, 0.0, 1.0], dtype=np.float32)
    started = time.monotonic()
    action = policy.predict({"eef_state": state, "exterior_image": blank, "wrist_image": blank,
                             "instruction": "place the red test tube in the red basket"})
    return time.monotonic() - started, action


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--actioncodec", default="", help="default: training_config actioncodec_path")
    parser.add_argument("--state-stats", default="", help="default: training_config state_stats_path")
    parser.add_argument("--host", default="0.0.0.0"); parser.add_argument("--port", type=int, default=8097)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--selftest", action="store_true", help="load, run one warm-up inference, print it, exit")
    args = parser.parse_args()
    policy = DroidEefPolicy(args)
    seconds, action = warm_up(policy)
    print(f"[DROID EEF] loaded {policy.checkpoint}; warm-up {seconds:.2f}s", flush=True)
    if args.selftest:
        print(f"[DROID EEF] selftest action shape {action.shape}, first row {np.round(action[0], 4).tolist()}", flush=True)
        return
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "franka_network_utils", PROJECT_ROOT / "omnigibson/learning/utils/network_utils.py")
    network_utils = importlib.util.module_from_spec(spec); spec.loader.exec_module(network_utils)
    metadata = policy.metadata(args.port)
    print(f"[DROID EEF] serving ws://{args.host}:{args.port} metadata={metadata}", flush=True)
    network_utils.WebsocketPolicyServer(policy=policy, host=args.host, port=args.port, metadata=metadata).serve_forever()


if __name__ == "__main__":
    main()
