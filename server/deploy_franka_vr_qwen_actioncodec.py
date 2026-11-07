#!/usr/bin/env python3
"""WebSocket deployment server for the real-robot FrankaVR fine-tune.

Run this file on mac20 inside Ours_VQAQ_VLA.  It intentionally uses the
training pipeline's tokenizer, state encoder and ActionCodec embodiment 2 so
the wire protocol cannot accidentally fall back to the LIBERO convention.
"""
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict

import draccus
import numpy as np
import torch
from PIL import Image
from transformers import AutoModel, AutoModelForImageTextToText, AutoProcessor, LogitsProcessor, LogitsProcessorList

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from franka_vr_3view_overfit.franka_vr_qwen_actioncodec import (  # noqa: E402
    ACTION_REPRESENTATION, DROID_MAX_LINEAR_DELTA, DROID_MAX_ROTATION_DELTA,
    DROID_Q01, DROID_Q99, FUTURE_STATE_TOKENS, STATE_TOKENS, ActionTokenizer,
    VLAWithState, restore_actioncodec_derived_buffers,
)

spec = importlib.util.spec_from_file_location(
    "franka_network_utils", ROOT / "omnigibson/learning/utils/network_utils.py"
)
module = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(module)
WebsocketPolicyServer = module.WebsocketPolicyServer


class ActionTokenOnly(LogitsProcessor):
    def __init__(self, token_ids):
        self.token_ids = torch.as_tensor(token_ids, dtype=torch.long)

    def __call__(self, input_ids, scores):
        allowed = self.token_ids.to(scores.device)
        output = torch.full_like(scores, -torch.inf)
        output[:, allowed] = scores[:, allowed]
        return output


def _numpy(value: Any) -> Any:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    if isinstance(value, dict):
        return {key: _numpy(item) for key, item in value.items()}
    return value


@dataclass
class Config:
    checkpoint: Path = ROOT / "runs/franka_vr_actionfix_v2/Qwen3VL2B+FrankaVR+lr6e-05+franka_vr_actionfix_v2_8gpu-singleturn-20260928_161451_f9998af9/save_5000_chkpt"
    actioncodec: Path = ROOT / "actioncodec_libero/weights/ActionCodec-Base-RVQft"
    state_stats: Path = ROOT / "norm_stats/franka_vr_state.json"
    host: str = "0.0.0.0"
    port: int = 8093
    device: str = "cuda:0"


class FrankaVRPolicy:
    def __init__(self, cfg: Config):
        self.device = torch.device(cfg.device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(f"CUDA unavailable: {self.device}")
        checkpoint = cfg.checkpoint.resolve()
        training = json.loads((checkpoint / "training_config.json").read_text())
        if training.get("action_representation") != ACTION_REPRESENTATION:
            raise RuntimeError(f"Unexpected action representation: {training.get('action_representation')}")
        if (training.get("observation_horizon"), training.get("action_horizon")) != (10, 15):
            raise RuntimeError("This server only supports the FrankaVR 10 Hz -> 15 Hz checkpoint")
        self.processor = AutoProcessor.from_pretrained(checkpoint, trust_remote_code=True, local_files_only=True)
        vla = AutoModelForImageTextToText.from_pretrained(
            checkpoint, dtype=torch.bfloat16, low_cpu_mem_usage=True,
            trust_remote_code=True, local_files_only=True,
        ).to(self.device)
        codec = AutoModel.from_pretrained(cfg.actioncodec.resolve(), trust_remote_code=True, local_files_only=True)
        restore_actioncodec_derived_buffers(codec)
        action_ids = self.processor.tokenizer.convert_tokens_to_ids(
            [f"<|action_{index}|>" for index in range(int(codec.config.vq_codebook_size))]
        )
        self.tokenizer = ActionTokenizer(codec, action_ids, embodiment_id=2)
        state_ids = self.processor.tokenizer.convert_tokens_to_ids(STATE_TOKENS)
        future_ids = self.processor.tokenizer.convert_tokens_to_ids(FUTURE_STATE_TOKENS)
        self.model = VLAWithState(vla, state_ids, future_ids).to(device=self.device, dtype=torch.bfloat16)
        self.model.state_encoder.load_state_dict(torch.load(checkpoint / "franka_state_encoder.pth", map_location="cpu", weights_only=True))
        self.model.future_state_predictor.load_state_dict(torch.load(checkpoint / "franka_future_state_predictor.pth", map_location="cpu", weights_only=True))
        self.model.eval()
        stats = json.loads(cfg.state_stats.read_text())["norm_stats"]["state"]
        self.q01 = np.asarray(stats["q01"], dtype=np.float32)
        self.q99 = np.asarray(stats["q99"], dtype=np.float32)
        self.span = np.where(self.q99 - self.q01 < 1e-8, 1.0, self.q99 - self.q01)
        self.constant = (self.q99 - self.q01) < 1e-8
        self.action_start = self.processor.tokenizer.convert_tokens_to_ids("<|action_start|>")
        self.dummy_action = "<|action_0|>" * self.tokenizer.num_tokens
        self.restriction = LogitsProcessorList([ActionTokenOnly(action_ids)])

    def _observation(self, raw: Dict[str, Any]):
        images = []
        for key in ("front_image", "left_image", "right_image"):
            image = np.asarray(raw[key], dtype=np.uint8)
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"{key} must be HxWx3 RGB, got {image.shape}")
            images.append(Image.fromarray(image, "RGB").resize((256, 256), Image.Resampling.BILINEAR))
        state = np.asarray(raw["joint_state"], dtype=np.float32).reshape(-1)
        if state.shape != (8,) or not np.isfinite(state).all():
            raise ValueError(f"joint_state must be finite (8,), got {state!r}")
        normalized = 2.0 * (state - self.q01) / self.span - 1.0
        normalized[self.constant] = 0.0
        task = str(raw.get("instruction", "")).strip().lower()
        if not task:
            raise ValueError("instruction is required")
        user = [
            {"type": "text", "text": "Current observation. Front camera:"}, {"type": "image", "image": images[0]},
            {"type": "text", "text": "Left camera:"}, {"type": "image", "image": images[1]},
            {"type": "text", "text": "Right camera:"}, {"type": "image", "image": images[2]},
            {"type": "text", "text": f"Task: {task}\nCurrent robot state:\nArm joints: {STATE_TOKENS[0]}\nGripper state: {STATE_TOKENS[1]}\nFirst predict the terminal state after the next one-second action chunk, then predict that action chunk."},
        ]
        prefix = f"Next state:\n<|future_state_start|>{''.join(FUTURE_STATE_TOKENS)}<|future_state_end|>\nAction:\n<|action_start|>{self.dummy_action}<|action_end|>"
        messages = [
            {"role": "system", "content": "You are a robot manipulation policy. Based on three camera views, current Franka joint state, gripper state, and task instruction, predict the next one-second action chunk."},
            {"role": "user", "content": user}, {"role": "assistant", "content": prefix},
        ]
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=False)
        encoded = self.processor(text=[text], images=images, return_tensors="pt")
        positions = torch.nonzero(encoded.input_ids[0] == self.action_start, as_tuple=False).flatten()
        if positions.numel() != 1:
            raise RuntimeError("Could not locate action-start token in FrankaVR prompt")
        end = int(positions[0]) + 1
        return encoded, torch.from_numpy(np.clip(normalized, -1.0, 1.0)).unsqueeze(0), end

    @torch.inference_mode()
    def act(self, observation: Dict[str, Any]) -> torch.Tensor:
        encoded, state, end = self._observation(_numpy(observation))
        ids = encoded.input_ids[:, :end].to(self.device)
        attention = torch.ones_like(ids)
        with torch.autocast("cuda", dtype=torch.bfloat16, enabled=self.device.type == "cuda"):
            generated = self.model.generate(
                input_ids=ids, states=state.to(self.device), attention_mask=attention,
                pixel_values=encoded.pixel_values.to(self.device, dtype=torch.bfloat16),
                image_grid_thw=encoded.image_grid_thw.to(self.device), logits_processor=self.restriction,
                min_new_tokens=self.tokenizer.num_tokens, max_new_tokens=self.tokenizer.num_tokens,
                do_sample=False, use_cache=True, eos_token_id=None,
                pad_token_id=self.processor.tokenizer.pad_token_id or self.processor.tokenizer.eos_token_id,
            )
        token_ids = generated[0, ids.shape[1]:].detach().cpu().tolist()
        normalized = self.tokenizer.decode(token_ids)
        action = normalized.copy()
        action[:, :6] *= np.maximum(np.abs(DROID_Q01), np.abs(DROID_Q99))
        action[:, 6] = (action[:, 6] + 1.0) / 2.0
        return torch.from_numpy(action.astype(np.float32))


@draccus.wrap()
def deploy(cfg: Config) -> None:
    policy = FrankaVRPolicy(cfg)
    WebsocketPolicyServer(policy=policy, host=cfg.host, port=cfg.port, metadata={
        "model_family": "qwen3vl2b_franka_vr_actionfix_v2",
        "observation_format": "franka_vr_front_left_right_and_joint_state_v1",
        "action_shape": [15, 7],
        "action_semantics": "droid_target_error_xyz_rpy_v2_and_gripper_closed_probability",
        "action_horizon_hz": 15,
    }).serve_forever()


if __name__ == "__main__":
    deploy()
