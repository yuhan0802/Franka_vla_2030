#!/usr/bin/env python3
"""WebSocket server for the Franka Task0 DROID-BAR end-effector model.

Runs on the training machine with PYTHONPATH=<Ours_VQAQ_VLA>. Everything that
defines the model contract (prompt, state encoder, ActionCodec tokenizer, BAR
decoder, DROID scales) is imported from the training code rather than copied:
  train/franka_task0_qwen_actioncodec_droid_pretrained_bar.py
  train/droid_qwen_actioncodec_state_first_bar.py

Wire protocol (client: eef_pose_policy_adapter.py)
  request : eef_state float32[7] = xyz, absolute rotvec, gripper open 0/1
            exterior_image / wrist_image uint8 480x640x3 RGB, instruction str
  response: action float32[15, 7] = linear/0.075, angular/0.15, gripper open 0..1
"""

import argparse
import importlib.util
import json
import os
import sys
import time
import types
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from transformers import AutoModel, AutoModelForImageTextToText, AutoProcessor

PROJECT_ROOT = Path(os.environ.get("VLA_PROJECT_ROOT", "/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA"))
sys.path.insert(0, str(PROJECT_ROOT))
# The training module imports W&B only for its training entry point.  The
# inference server does not use it, and should not depend on the health of the
# optional W&B/Pydantic installation on the serving host.
sys.modules.setdefault("wandb", types.ModuleType("wandb"))
from train.franka_task0_qwen_actioncodec_droid_pretrained_bar import (  # noqa: E402
    ACTION_REPRESENTATION, BAR_BOS_TOKEN, DROID_Q01, DROID_Q99, FUTURE_STATE_END_TOKEN,
    FUTURE_STATE_START_TOKEN, FUTURE_STATE_TOKENS, STATE_TOKENS, ActionTokenizer, VLAWithState,
    restore_actioncodec_derived_buffers, restore_legacy_wrapped_embeddings,
)
from train.droid_qwen_actioncodec_state_first_bar import BAR_ACTION_TOKENS, generate_action_codes_bar  # noqa: E402

try:  # the training module imports TensorFlow; keep it off the GPU
    import tensorflow as tf
    tf.config.set_visible_devices([], "GPU")
except Exception:
    pass

spec = importlib.util.spec_from_file_location(
    "franka_network_utils", PROJECT_ROOT / "omnigibson/learning/utils/network_utils.py")
network_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(network_utils)

STATE_PROTOCOL = "droid_eef_axis_angle6_gripper_open1_singleturn_v1"
# Task0 labels come from lerobot_episode_recorder eef_delta_rotation_command,
# which is inv(q_prev) * q_cur (tool frame) despite the "base_frame" wording in
# training_config. Verified on the Task0 data: command vs measured orientation
# change has mean cosine +0.42 in the tool frame and -0.36 in the world frame.
ROTATION_EXECUTION = "relative_rotvec_right_multiply"
IMAGE_SHAPE = (480, 640, 3)  # Camera/request shape before training-compatible preprocessing.
MODEL_IMAGE_SIZE = 256


class Task0EefPolicy:
    def __init__(self, args: argparse.Namespace):
        self.device = torch.device(args.device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(f"CUDA unavailable: {self.device}")
        checkpoint = Path(args.checkpoint).resolve()
        self.training = json.loads((checkpoint / "training_config.json").read_text())
        expected = {
            "base_model": "qwen3vl2b", "state_protocol": STATE_PROTOCOL,
            "action_representation": ACTION_REPRESENTATION, "action_horizon": 15,
            "action_frequency_hz": 15, "action_embodiment_id": 2, "action_prediction_mode": "block_ar_16",
            "predict_future_state": True, "prediction_order": "state_then_action",
            "image_size": MODEL_IMAGE_SIZE,
        }
        mismatched = {key: self.training.get(key) for key, value in expected.items() if self.training.get(key) != value}
        if mismatched:
            raise RuntimeError(f"checkpoint is not the Task0 EEF BAR protocol: {mismatched}")
        self.default_instruction = str(self.training.get("task_instruction", "")).strip().lower()

        self.processor = AutoProcessor.from_pretrained(checkpoint, trust_remote_code=True, local_files_only=True)
        vla = AutoModelForImageTextToText.from_pretrained(
            checkpoint, dtype=torch.bfloat16, low_cpu_mem_usage=True, trust_remote_code=True, local_files_only=True,
        ).to(self.device)
        codec_path = Path(args.actioncodec or self.training["actioncodec_path"])
        self.codec_path = codec_path
        codec = AutoModel.from_pretrained(codec_path, trust_remote_code=True, local_files_only=True)
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
            torch.load(checkpoint / "franka_state_encoder.pth", map_location="cpu", weights_only=True))
        self.model.future_state_predictor.load_state_dict(
            torch.load(checkpoint / "franka_future_state_predictor.pth", map_location="cpu", weights_only=True))
        self.model.eval()

        stats_path = Path(args.state_stats or self.training["state_stats_path"])
        self.stats_path = stats_path
        stats = json.loads(stats_path.read_text())["norm_stats"]["state"]
        self.q01 = np.asarray(stats["q01"], dtype=np.float32)
        self.q99 = np.asarray(stats["q99"], dtype=np.float32)
        if self.q01.shape != (7,) or self.q99.shape != (7,):
            raise RuntimeError(f"DROID EEF state stats must be 7D: {stats_path}")
        self.span = np.maximum(self.q99 - self.q01, 1e-8)
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
                raise ValueError(f"{key} must be {IMAGE_SHAPE} RGB like the Task0 recordings, got {image.shape}")
            # Match FrankaTask0Dataset validation/inference preprocessing exactly:
            # no augmentation or random crop, only a bilinear square resize.
            images.append(
                Image.fromarray(image, "RGB").resize(
                    (MODEL_IMAGE_SIZE, MODEL_IMAGE_SIZE), Image.Resampling.BILINEAR
                )
            )
        instruction = str(raw.get("instruction", "")).strip().lower() or self.default_instruction
        normalized = np.clip(2.0 * (state - self.q01) / self.span - 1.0, -1.0, 1.0).astype(np.float32)
        # Identical to FrankaTask0Dataset.transform with predict_future_state=True.
        user = [
            {"type": "text", "text": "Main camera:"}, {"type": "image", "image": images[0]},
            {"type": "text", "text": "Wrist camera:"}, {"type": "image", "image": images[1]},
            {"type": "text", "text": (
                f"Task: {instruction}\n"
                "Robot state:\n"
                f"End-effector pose: {STATE_TOKENS[0]}\n"
                f"Gripper state: {STATE_TOKENS[1]}\n"
                "Based on the current observations and robot state, what is the next action?")},
        ]
        assistant = (f"Next state:\n{FUTURE_STATE_START_TOKEN}{''.join(FUTURE_STATE_TOKENS)}"
                     f"{FUTURE_STATE_END_TOKEN}\nAction:\n<|action_start|>{self.dummy_action}<|action_end|>")
        messages = [
            {"role": "system", "content": (
                "You are a robot manipulation policy. Based on the main camera, wrist camera, "
                "current end-effector pose, gripper state, and task instruction, "
                "first predict the robot state after the next one-second action chunk, "
                "then predict that action chunk.")},
            {"role": "user", "content": user}, {"role": "assistant", "content": assistant},
        ]
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=False)
        encoded = self.processor(text=[text], images=images, return_tensors="pt")
        positions = torch.nonzero(encoded.input_ids[0] == self.action_start, as_tuple=False).flatten()
        if positions.numel() != 1:
            raise RuntimeError("could not locate <|action_start|> in the Task0 prompt")
        return encoded, int(positions[0]) + 1, torch.from_numpy(normalized).unsqueeze(0), instruction

    @torch.inference_mode()
    def predict(self, raw) -> np.ndarray:
        encoded, end, state, _ = self._prompt(raw)
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
        action[:, :6] *= self.scale          # inverse of ActionTokenizer.normalize, applied once
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
            "model_family": "qwen3vl2b_franka_task0_droid_bar_eef",
            "checkpoint": str(self.checkpoint),
            # Declared so the adapter can refuse a reused server started with other assets.
            "actioncodec": str(self.codec_path), "state_stats": str(self.stats_path),
            "state_protocol": STATE_PROTOCOL,
            "action_representation": ACTION_REPRESENTATION,
            "action_shape": [15, 7], "action_hz": 15,
            "gripper_convention": "0_closed_1_open",
            "gripper_state_source": "binary",
            "rotation_execution": ROTATION_EXECUTION,
            "observation_format": "franka_task0_main_wrist_640x480_eef7_v1",
            "model_image_size": [MODEL_IMAGE_SIZE, MODEL_IMAGE_SIZE],
            "training_instruction": self.default_instruction,
            "port": port,
        }


def warm_up(policy: Task0EefPolicy) -> float:
    blank = np.zeros(IMAGE_SHAPE, dtype=np.uint8)
    state = np.array([0.5, 0.0, 0.4, 3.1, 0.0, 0.0, 1.0], dtype=np.float32)
    started = time.monotonic()
    policy.predict({"eef_state": state, "exterior_image": blank, "wrist_image": blank, "instruction": ""})
    return time.monotonic() - started


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--actioncodec", default="", help="default: training_config actioncodec_path")
    parser.add_argument("--state-stats", default="", help="default: training_config state_stats_path")
    parser.add_argument("--host", default="0.0.0.0"); parser.add_argument("--port", type=int, default=8095)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    policy = Task0EefPolicy(args)
    print(f"[Task0 EEF] loaded {policy.checkpoint}; warm-up {warm_up(policy):.2f}s", flush=True)
    metadata = policy.metadata(args.port)
    print(f"[Task0 EEF] serving ws://{args.host}:{args.port} metadata={metadata}", flush=True)
    network_utils.WebsocketPolicyServer(policy=policy, host=args.host, port=args.port, metadata=metadata).serve_forever()


if __name__ == "__main__":
    main()
