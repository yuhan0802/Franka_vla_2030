"""BAR fine-tuning on Franka Task0 v4 data, initialized from DROID weights.

The required offline cache converts measured 30 Hz poses into a 15 Hz stream.
State is measured world->fr3_link8 XYZ + absolute rotation-vector + continuous
gripper openness. Each arm action is the measured t->t+2 displacement in the
world frame: translation / 0.075 and rotvec(R[t+2] * inv(R[t])) / 0.15. Thus
deployment must apply rotation by left multiplication. The gripper action is
the operator target with 0=closed and 1=open.
"""

import os
import sys
from pathlib import Path

os.environ["LIBERO_ACTION_PREDICTION_MODE"] = "block_ar_16"

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))
os.environ.setdefault(
    "HF_HOME", str(_PROJECT_ROOT / "actioncodec_libero" / ".cache" / "huggingface")
)
os.environ.setdefault("USE_TF", "0")
os.environ.setdefault("TRANSFORMERS_NO_TF", "1")

import json
import math
import random
import re
import gc
import shutil
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Dict, Iterator, List, Optional, Tuple

import cv2
import draccus
import numpy as np
import pyarrow.parquet as pq
import torch
import torch.distributed as dist
import torch.nn.functional as F
import tqdm
import wandb
import tensorflow as tf
from accelerate import PartialState
from huggingface_hub import snapshot_download
from PIL import Image, ImageEnhance
from scipy.spatial.transform import Rotation
from torch import nn
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.optim import AdamW
from torch.optim.lr_scheduler import LambdaLR
from torch.utils.data import DataLoader, IterableDataset, get_worker_info
from transformers import AutoModel, AutoModelForImageTextToText, AutoProcessor

from prismatic.vla.datasets.rlds import make_interleaved_dataset
from prismatic.vla.datasets.rlds.oxe import OXE_NAMED_MIXTURES, get_oxe_dataset_kwargs_and_weights
from prismatic.vla.datasets.rlds.utils.data_utils import NormalizationType
from dlimp import augmentations as dlimp_augmentations

# The qwenvla PyTorch build predates this helper, while the installed
# Transformers image processor calls it unconditionally.
if hasattr(torch, "compiler") and not hasattr(torch.compiler, "is_compiling"):
    torch.compiler.is_compiling = lambda: False


IGNORE_INDEX = -100
PROJECT_ROOT = _PROJECT_ROOT
LIBERO_DATASETS = {
    "libero_spatial_no_noops",
    "libero_object_no_noops",
    "libero_goal_no_noops",
    "libero_10_no_noops",
}

# Second-stage replay recipe for the tasks that remain weak after the first
# LIBERO BAR run. Instructions are used as stable task identifiers because the
# four RLDS builders do not expose a common numeric task-id field.
TARGETED_LIBERO_WEAK_TASKS = {
    # LIBERO-Goal tasks 2, 3, 5, 9 (one-based benchmark numbering).
    "open the top drawer and put the bowl inside",
    "push the plate to the front of the stove",
    "put the bowl on the stove",
    "put the wine bottle on top of the cabinet",
    # LIBERO-10 tasks 4, 6, 7, 8, 9 (one-based benchmark numbering).
    "put both the alphabet soup and the tomato sauce in the basket",
    "put the black bowl in the bottom drawer of the cabinet and close it",
    "put the white mug on the left plate and put the yellow and white mug on the right plate",
    "put the white mug on the plate and put the chocolate pudding to the right of the plate",
    "put the yellow and white mug in the microwave and close it",
}
TASK_WEIGHTING_MODES = {
    "none",
    "targeted_weak_v1",
    "targeted_weak_v2",
    "targeted_weak_v2_gripper3x",
    "gripper_transition_3x",
}

# Exact statistics published in ZibinDong/ActionCodec-Base-RVQft's model card.
LIBERO_Q01 = np.asarray(
    [-0.87857145, -0.87589288, -0.9375, -0.15107143, -0.20678571, -0.27964285, 0.0],
    dtype=np.float32,
)
LIBERO_Q99 = np.asarray(
    [0.9375, 0.91071427, 0.9375, 0.20357142, 0.26357144, 0.375, 1.0],
    dtype=np.float32,
)
DROID_Q01 = np.asarray(
    [-0.7776297927, -0.5803514123, -0.5795090795,
     -0.6464047730, -0.7041108310, -0.8895104378, 0.0], dtype=np.float32
)
DROID_Q99 = np.asarray(
    [0.7597932070, 0.5726242214, 0.7351000607,
     0.6705610305, 0.6464948207, 0.8897542208, 1.0], dtype=np.float32
)

DROID_MAX_LINEAR_DELTA = 0.075
DROID_MAX_ROTATION_DELTA = 0.15
ACTION_REPRESENTATION = "droid_base_cartesian_velocity_plus_inverted_gripper_v1"
FRANKA_CACHE_VERSION = os.environ.get("FRANKA_CACHE_VERSION", "v4")
if FRANKA_CACHE_VERSION not in {"v4", "v5"}:
    raise ValueError("FRANKA_CACHE_VERSION must be v4 or v5")
FRANKA_V4_CACHE_NAME = f"franka_measured_eef_base_15hz_{FRANKA_CACHE_VERSION}.npz"
FRANKA_V4_ACTION_REPRESENTATION = "measured_base_eef_delta_plus_target_gripper_v4"
if FRANKA_CACHE_VERSION == "v5":
    FRANKA_V4_ACTION_REPRESENTATION = "measured_base_eef_delta_plus_measured_gripper_direction_initial_open_v5"
FRANKA_V4_ROTATION_EXECUTION = "relative_rotvec_left_multiply"
TASK0_GRIPPER_CLOSED_WIDTH = 0.01
TASK0_GRIPPER_OPEN_WIDTH = 0.08


def random_gaussian_noise(
    image: tf.Tensor,
    stddev: float,
    probability: float,
    seed: tf.Tensor,
) -> tf.Tensor:
    """Apply low-level stateless sensor noise with a per-image probability."""
    decision_seed, noise_seed = tf.unstack(
        tf.random.experimental.stateless_split(seed, 2)
    )
    apply_noise = tf.random.stateless_uniform((), decision_seed) < probability
    noise = tf.random.stateless_normal(
        tf.shape(image), noise_seed, mean=0.0, stddev=stddev, dtype=image.dtype
    )
    return tf.where(apply_noise, image + noise, image)


dlimp_augmentations.AUGMENT_OPS["random_gaussian_noise"] = random_gaussian_noise


def build_image_augment_kwargs(
    use_random_resized_crop: bool,
    use_robust_image_aug: bool = False,
    primary_only_random_resized_crop: bool = False,
) -> Dict[str, Any]:
    augmentations: Dict[str, Any] = {
        "random_brightness": [0.1],
        "random_contrast": [0.9, 1.1],
        "random_saturation": [0.9, 1.1],
    }
    augment_order = [
        "random_brightness",
        "random_contrast",
        "random_saturation",
    ]
    if use_robust_image_aug:
        augmentations["random_hue"] = [0.02]
        augmentations["random_gaussian_noise"] = {
            "stddev": 0.01,
            "probability": 0.2,
        }
        augment_order.extend(["random_hue", "random_gaussian_noise"])
    if use_random_resized_crop:
        crop_kwargs = {
            "scale": [0.9, 1.0],
            "ratio": [0.95, 1.05],
        }
        augmentations["random_resized_crop"] = crop_kwargs
        augment_order.insert(0, "random_resized_crop")
    augmentations["augment_order"] = augment_order
    if use_random_resized_crop and primary_only_random_resized_crop:
        wrist_augmentations = {
            key: value
            for key, value in augmentations.items()
            if key != "random_resized_crop"
        }
        wrist_augmentations["augment_order"] = [
            op for op in augment_order if op != "random_resized_crop"
        ]
        return {
            "primary": augmentations,
            "wrist": wrist_augmentations,
        }
    return augmentations


def barrier() -> None:
    if dist.is_available() and dist.is_initialized():
        dist.barrier()


def resolve_actioncodec(model_id: str, local_dir: str) -> str:
    """Always load from actioncodec_libero; download there only when incomplete."""
    path = Path(local_dir).expanduser().resolve()
    if not (path / "config.json").is_file() or not (path / "model.safetensors").is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        snapshot_download(repo_id=model_id, local_dir=str(path))
    return str(path)


def restore_actioncodec_derived_buffers(codec: nn.Module) -> Dict[str, List[int]]:
    """Rebuild ActionCodec lookup buffers after Transformers low-memory loading.

    ActionCodec marks ``_action_dim``, ``_freq`` and ``_duration`` as
    non-persistent buffers because they are derived from ``embodiment_config``.
    Transformers 5 can materialize those buffers from the meta device without
    preserving their initialized values, which silently corrupts action-token
    labels.  Always derive them again from the authoritative config.
    """
    embodiment_configs = list(codec.config.embodiment_config.values())
    if not embodiment_configs:
        raise ValueError("ActionCodec embodiment_config is empty")
    expected = {
        "_action_dim": [int(item["action_dim"]) for item in embodiment_configs],
        "_freq": [int(item["freq"]) for item in embodiment_configs],
        "_duration": [int(item["duration"]) for item in embodiment_configs],
    }
    for component_name in ("encoder", "decoder"):
        component = getattr(codec, component_name, None)
        if component is None:
            raise AttributeError(f"ActionCodec has no {component_name!r} component")
        component_device = next(component.parameters()).device
        for buffer_name, values in expected.items():
            setattr(
                component,
                buffer_name,
                torch.tensor(values, dtype=torch.long, device=component_device),
            )
            actual = getattr(component, buffer_name).detach().cpu().tolist()
            if actual != values:
                raise RuntimeError(
                    f"Failed to restore ActionCodec {component_name}.{buffer_name}: "
                    f"expected {values}, got {actual}"
                )
    return expected


def load_g05_qwen35_backbone(
    processor_path: str,
    checkpoint_path: str,
) -> nn.Module:
    """Build an HF Qwen3.5-2B model from G0.5's robot-pretrained backbone.

    G0.5 stores the Qwen3.5 vision tower, VLM, 27-D proprio encoder and action
    expert in one PyTorch checkpoint. This training pipeline deliberately loads
    only the architecture-compatible vision/VLM weights. Its own LIBERO state
    MLP, future-state head and ActionCodec token head remain authoritative.
    """
    try:
        from transformers import AutoConfig, AutoModelForMultimodalLM
    except ImportError as exc:
        raise RuntimeError(
            "BASE_MODEL=qwen35_g05 requires a Transformers release with "
            "Qwen3.5/AutoModelForMultimodalLM support. The current qwenvla "
            "environment is too old; use the Qwen3.5-compatible dependency "
            "set before launching this option."
        ) from exc

    checkpoint = Path(checkpoint_path).expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"G0.5 checkpoint does not exist: {checkpoint}")
    config = AutoConfig.from_pretrained(
        processor_path, trust_remote_code=True, local_files_only=True
    )
    # G0.5's custom Qwen3.5 backbone does not contain an MTP prediction head.
    text_config = getattr(config, "text_config", None)
    if text_config is not None and hasattr(text_config, "mtp_num_hidden_layers"):
        text_config.mtp_num_hidden_layers = 0
    model = AutoModelForMultimodalLM.from_config(config, dtype=torch.bfloat16)

    payload = torch.load(
        checkpoint,
        map_location="cpu",
        weights_only=True,
        mmap=True,
    )
    state = payload.get("model_state_dict", payload)
    if not isinstance(state, dict):
        raise TypeError(f"Unexpected G0.5 checkpoint payload: {type(state)!r}")

    prefix_map = (
        ("model.vision_tower.", "model.visual."),
        ("model.vlm.layers.", "model.language_model.layers."),
        ("model.vlm.norm.", "model.language_model.norm."),
    )
    target_state = model.state_dict()
    mapped = {}
    mismatched = []
    for source_key, tensor in state.items():
        target_key = None
        for source_prefix, target_prefix in prefix_map:
            if source_key.startswith(source_prefix):
                target_key = target_prefix + source_key[len(source_prefix):]
                break
        if target_key is None or target_key not in target_state:
            continue
        if tuple(tensor.shape) != tuple(target_state[target_key].shape):
            mismatched.append(
                (source_key, tuple(tensor.shape), target_key, tuple(target_state[target_key].shape))
            )
            continue
        mapped[target_key] = tensor

    # The exported G0.5 vocabulary also contains its 4096-code action language.
    # Copy only the original Qwen3.5 tokenizer rows; this pipeline initializes
    # its own, semantically different 2048 ActionCodec tokens below.
    source_embeddings = state.get("model.vlm.input_proj.weight")
    if source_embeddings is None:
        raise KeyError("G0.5 checkpoint is missing model.vlm.input_proj.weight")
    target_embeddings = model.get_input_embeddings().weight
    tokenizer_length = len(
        AutoProcessor.from_pretrained(
            processor_path, trust_remote_code=True, local_files_only=True
        ).tokenizer
    )
    copy_rows = min(tokenizer_length, source_embeddings.shape[0], target_embeddings.shape[0])
    with torch.no_grad():
        target_embeddings[:copy_rows].copy_(
            source_embeddings[:copy_rows].to(dtype=target_embeddings.dtype)
        )

    if mismatched:
        preview = "\n".join(map(str, mismatched[:8]))
        raise ValueError(f"G0.5/HF Qwen3.5 backbone shape mismatch:\n{preview}")
    if len(mapped) < 500:
        raise RuntimeError(
            f"Only {len(mapped)} G0.5 backbone tensors matched the HF Qwen3.5 model; "
            "refusing a partial initialization"
        )
    incompatible = model.load_state_dict(mapped, strict=False)
    model.tie_weights()
    print(
        f"[G0.5] loaded {len(mapped)} robot-pretrained vision/VLM tensors and "
        f"{copy_rows} base embedding rows from {checkpoint}"
    )
    print(
        f"[G0.5] skipped action_expert/proprio_embedder; "
        f"HF-only missing tensors={len(incompatible.missing_keys)}"
    )
    del mapped, target_state, state, payload
    gc.collect()
    return model


def initialize_new_token_rows(
    model: nn.Module,
    first_new_token_id: int,
    vocabulary_size: int,
) -> None:
    """Initialize newly assigned token IDs instead of reusing padded/G0.5 rows."""
    if vocabulary_size <= first_new_token_id:
        return
    embeddings = model.get_input_embeddings().weight
    initializer_range = float(
        getattr(getattr(model.config, "text_config", model.config), "initializer_range", 0.02)
    )
    with torch.no_grad():
        embeddings[first_new_token_id:vocabulary_size].normal_(
            mean=0.0, std=initializer_range
        )
    model.tie_weights()


def restore_legacy_wrapped_embeddings(vla: nn.Module, checkpoint: Path) -> bool:
    """Restore embeddings from checkpoints saved with StateInjectingEmbeddings.

    save_1000/save_2000 predate the unwrapped checkpoint writer below, so their
    Qwen embedding table lives under an extra ``original_embeddings`` level and
    is ignored by the vanilla Transformers loader.
    """
    wrapped_key = "model.language_model.embed_tokens.original_embeddings.weight"
    weight_file = checkpoint / "model.safetensors"
    if not weight_file.is_file():
        index_file = checkpoint / "model.safetensors.index.json"
        if not index_file.is_file():
            raise FileNotFoundError(f"No model safetensors found in {checkpoint}")
        weight_map = json.loads(index_file.read_text(encoding="utf-8"))["weight_map"]
        if wrapped_key not in weight_map:
            return False
        weight_file = checkpoint / weight_map[wrapped_key]

    from safetensors import safe_open

    with safe_open(weight_file, framework="pt", device="cpu") as handle:
        if wrapped_key not in handle.keys():
            return False
        saved_weight = handle.get_tensor(wrapped_key)
    target_weight = vla.get_input_embeddings().weight
    if (
        saved_weight.shape[1] != target_weight.shape[1]
        or saved_weight.shape[0] > target_weight.shape[0]
    ):
        raise ValueError(
            f"Resume embedding shape is incompatible: {tuple(saved_weight.shape)} vs "
            f"{tuple(target_weight.shape)}"
        )
    with torch.no_grad():
        target_weight[: saved_weight.shape[0]].copy_(
            saved_weight.to(dtype=target_weight.dtype)
        )
    vla.tie_weights()
    return True


def checkpoint_step(checkpoint: Path) -> int:
    state_file = checkpoint / "trainer_state.json"
    if state_file.is_file():
        return int(json.loads(state_file.read_text(encoding="utf-8"))["global_step"])
    match = re.fullmatch(r"save_(\d+)_chkpt", checkpoint.name)
    if match is None:
        raise ValueError(
            f"Cannot infer resume step from {checkpoint}; expected save_<step>_chkpt "
            "or trainer_state.json"
        )
    return int(match.group(1))


def position_scheduler(scheduler, step: int) -> None:
    """Place a fresh LambdaLR at a known global step without replaying updates."""
    scheduler.last_epoch = step
    learning_rates = [
        base_lr * schedule(step)
        for base_lr, schedule in zip(scheduler.base_lrs, scheduler.lr_lambdas)
    ]
    for parameter_group, learning_rate in zip(scheduler.optimizer.param_groups, learning_rates):
        parameter_group["lr"] = learning_rate
    scheduler._last_lr = learning_rates
    scheduler._step_count = step + 1


def save_unwrapped_vla(vla: nn.Module, output_dir: Path) -> None:
    """Save a standard Qwen checkpoint while retaining runtime state injection."""
    injected_embeddings = vla.get_input_embeddings()
    if not isinstance(injected_embeddings, StateInjectingEmbeddings):
        vla.save_pretrained(output_dir)
        return
    vla.set_input_embeddings(injected_embeddings.original_embeddings)
    try:
        vla.tie_weights()
        vla.save_pretrained(output_dir)
    finally:
        vla.set_input_embeddings(injected_embeddings)


class ActionTokenizer:
    def __init__(self, codec: Any, qwen_token_ids: List[int], embodiment_id: int, arm_scale=None, mulaw_mu=0.0):
        self.codec = codec.eval().to("cpu")
        self.mulaw_mu = float(mulaw_mu)
        if not np.isfinite(self.mulaw_mu) or self.mulaw_mu < 0:
            raise ValueError("mulaw_mu must be finite and nonnegative")
        self.arm_scale = np.asarray(
            arm_scale if arm_scale is not None else np.maximum(np.abs(DROID_Q01[:6]), np.abs(DROID_Q99[:6])),
            dtype=np.float32,
        )
        if self.arm_scale.shape != (6,) or not np.isfinite(self.arm_scale).all() or np.any(self.arm_scale <= 0):
            raise ValueError("Action arm_scale must be six finite positive values")
        self.embodiment_id = embodiment_id
        self.codebook_size = int(codec.config.vq_codebook_size)
        self.num_tokens = int(codec.config.n_tokens)
        embodiment = list(codec.config.embodiment_config.values())[embodiment_id]
        self.chunk_size = int(embodiment["freq"] * embodiment["duration"])
        self.action_dim = int(embodiment["action_dim"])
        if (self.chunk_size, self.action_dim) != (15, 7):
            raise ValueError(
                f"ActionCodec embodiment {embodiment_id} is {self.chunk_size}x{self.action_dim}; "
                "Franka/DROID requires 15x7"
            )
        if len(qwen_token_ids) != self.codebook_size:
            raise ValueError("Qwen action-token vocabulary does not match the ActionCodec codebook")
        self.qwen_token_ids = torch.tensor(qwen_token_ids, dtype=torch.long)
        self.valid_token_ids = set(qwen_token_ids)
        self.id_to_code = {token_id: code for code, token_id in enumerate(qwen_token_ids)}
        for parameter in self.codec.parameters():
            parameter.requires_grad = False

    def normalize(self, actions: np.ndarray) -> np.ndarray:
        actions = np.asarray(actions, dtype=np.float32).copy()
        scale = self.arm_scale
        actions[..., :6] /= scale[:6]
        actions[..., 6] = actions[..., 6] * 2.0 - 1.0
        actions = np.clip(actions, -1.0, 1.0)
        if getattr(self, 'mulaw_mu', 0.0):
            from train.franka_action_companding import mu_law
            actions[..., :6] = mu_law(actions[..., :6], self.mulaw_mu)
        return actions

    @torch.no_grad()
    def encode(self, actions: np.ndarray) -> List[int]:
        actions = np.asarray(actions, dtype=np.float32)
        if actions.shape != (self.chunk_size, self.action_dim):
            raise ValueError(
                f"Expected a {(self.chunk_size, self.action_dim)} DROID action chunk, got {actions.shape}"
            )
        codes = self.codec.encode(self.normalize(actions)[None], embodiment_ids=self.embodiment_id)[0]
        codes = torch.as_tensor(codes, dtype=torch.long).flatten()
        if codes.numel() != self.num_tokens:
            raise RuntimeError(f"ActionCodec emitted {codes.numel()} tokens instead of {self.num_tokens}")
        return self.qwen_token_ids[codes].tolist()


STATE_TOKENS = ["<|state_eef|>", "<|state_gripper|>"]
STATE_AUG = os.environ.get("FRANKA_STATE_AUG", "none")
STATE_DROP_P = float(os.environ.get("FRANKA_STATE_DROP_P", "0.5"))
STATE_NOISE_POS_M = float(os.environ.get("FRANKA_STATE_NOISE_POS_M", "0.03"))
STATE_NOISE_ROT_RAD = float(os.environ.get("FRANKA_STATE_NOISE_ROT_RAD", "0.1"))
STATE_NOISE_P = float(os.environ.get("FRANKA_STATE_NOISE_P", "1.0"))  # fraction of training samples that get pose noise
if STATE_AUG not in {"none", "mask", "drop", "noise"}:
    raise ValueError(f"FRANKA_STATE_AUG must be none/mask/drop/noise, got {STATE_AUG!r}")
# FRANKA_TARGETS_JSON: per-episode tube/basket base-frame xy (detected in the first exterior frame,
# ~/claude_tube/targets.py) appended to the instruction, so the policy is told where the tube is
# instead of recognising the scene. Training jitters them (uniform, metres) to keep the mapping smooth.
TARGETS = json.load(open(os.environ["FRANKA_TARGETS_JSON"])) if os.environ.get("FRANKA_TARGETS_JSON") else None
TARGET_JITTER_TUBE_M = float(os.environ.get("FRANKA_TARGET_JITTER_TUBE_M", "0.008"))
TARGET_JITTER_BASKET_M = float(os.environ.get("FRANKA_TARGET_JITTER_BASKET_M", "0.02"))


def target_instruction(instruction: str, tube, basket) -> str:
    """Must match eef_eval_common.target_instruction and the deploy client."""
    return (f"{instruction} (tube x={1000 * tube[0]:.0f} y={1000 * tube[1]:.0f}, "
            f"basket x={1000 * basket[0]:.0f} y={1000 * basket[1]:.0f}, mm)")
FUTURE_STATE_START_TOKEN = "<|future_state_start|>"
FUTURE_STATE_END_TOKEN = "<|future_state_end|>"
FUTURE_STATE_TOKENS = ["<|future_state_eef|>", "<|future_state_gripper|>"]
# Two fixed causal-computation slots used by the latent2_ar protocol.  Their
# token IDs are present in input_ids so later action positions can attend to
# them, but their labels are always IGNORE_INDEX: they have no direct target.
LATENT_PREFIX_TOKENS = ["<|action_latent_0|>", "<|action_latent_1|>"]
ACTION_PREDICTION_MODE = os.environ.get("LIBERO_ACTION_PREDICTION_MODE", "token_ar")
BAR_BOS_TOKEN = "<|action_bar_bos|>"


class LiberoStateEncoder(nn.Module):
    """DROID-compatible EEF(6-D) + scalar gripper-openness encoder."""

    def __init__(self, hidden_size: int):
        super().__init__()
        self.eef_mlp = self._make_mlp(6, hidden_size)
        self.gripper_mlp = self._make_mlp(1, hidden_size)
        self.type_embedding = nn.Parameter(torch.empty(2, hidden_size))
        nn.init.normal_(self.type_embedding, std=0.02)

    @staticmethod
    def _make_mlp(input_size: int, hidden_size: int) -> nn.Sequential:
        return nn.Sequential(
            nn.Linear(input_size, hidden_size),
            nn.SiLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.LayerNorm(hidden_size),
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        if state.ndim != 2 or state.shape[-1] != 7:
            raise ValueError(f"Expected normalized state shape (batch, 7), got {tuple(state.shape)}")
        dtype = self.eef_mlp[0].weight.dtype
        state = state.to(dtype=dtype)
        tokens = torch.stack(
            [self.eef_mlp(state[:, :6]), self.gripper_mlp(state[:, 6:])], dim=1
        )
        return tokens + self.type_embedding.unsqueeze(0)


class FutureStatePredictor(nn.Module):
    """Decode two causal hidden states into DROID-compatible 7D state."""

    def __init__(self, hidden_size: int):
        super().__init__()
        bottleneck = max(hidden_size // 4, 256)
        self.eef_head = nn.Sequential(
            nn.Linear(hidden_size, bottleneck),
            nn.SiLU(),
            nn.Linear(bottleneck, 6),
            nn.Tanh(),
        )
        self.gripper_head = nn.Sequential(
            nn.Linear(hidden_size, bottleneck),
            nn.SiLU(),
            nn.Linear(bottleneck, 1),
            nn.Tanh(),
        )

    def forward(self, eef_hidden: torch.Tensor, gripper_hidden: torch.Tensor) -> torch.Tensor:
        return torch.cat(
            (self.eef_head(eef_hidden), self.gripper_head(gripper_hidden)), dim=-1
        )


class StateInjectingEmbeddings(nn.Module):
    def __init__(self, original_embeddings: nn.Module, state_encoder: LiberoStateEncoder, token_ids: List[int]):
        super().__init__()
        self.original_embeddings = original_embeddings
        self.state_encoder = state_encoder
        self.token_ids = tuple(token_ids)
        self.current_state = None

    def forward(self, input_ids: torch.Tensor, *args, **kwargs) -> torch.Tensor:
        embeddings = self.original_embeddings(input_ids, *args, **kwargs).clone()
        if self.current_state is None:
            return embeddings
        state_embeddings = self.state_encoder(self.current_state)
        for part_index, token_id in enumerate(self.token_ids):
            mask = input_ids == token_id
            # Cached generation steps contain only the newly generated token,
            # so the prompt placeholders are absent after the first pass.
            if not mask.any():
                continue
            count = int(mask.sum().item())
            if count != input_ids.shape[0]:
                raise RuntimeError(
                    f"State token {STATE_TOKENS[part_index]} must occur once per sample; "
                    f"found {count} for batch {input_ids.shape[0]}"
                )
            embeddings[mask] = state_embeddings[:, part_index].to(embeddings.dtype)
        return embeddings


class VLAWithState(nn.Module):
    """Qwen wrapper using the same MLP-at-placeholder injection pattern as prior training."""

    def __init__(
        self,
        vla: nn.Module,
        state_token_ids: List[int],
        future_state_token_ids: List[int],
    ):
        super().__init__()
        self.vla = vla
        hidden_size = vla.get_input_embeddings().weight.shape[1]
        self.state_encoder = LiberoStateEncoder(hidden_size)
        self.injecting_embeddings = StateInjectingEmbeddings(
            vla.get_input_embeddings(), self.state_encoder, state_token_ids
        )
        self.vla.set_input_embeddings(self.injecting_embeddings)
        self.future_state_token_ids = tuple(future_state_token_ids)
        self.future_state_predictor = (
            FutureStatePredictor(hidden_size) if future_state_token_ids else None
        )

    def _predict_future_state(
        self, input_ids: torch.Tensor, hidden_states: torch.Tensor
    ) -> torch.Tensor:
        if self.future_state_predictor is None or len(self.future_state_token_ids) != 2:
            raise RuntimeError("Future-state prediction is not enabled")
        selected = []
        for token_id in self.future_state_token_ids:
            mask = input_ids == token_id
            if int(mask.sum().item()) != input_ids.shape[0]:
                raise RuntimeError(
                    f"Future-state token {token_id} must occur once per sample"
                )
            selected.append(hidden_states[mask])
        return self.future_state_predictor(selected[0], selected[1])

    def forward(self, input_ids: torch.Tensor, states: torch.Tensor, **kwargs):
        self.injecting_embeddings.current_state = states
        try:
            if self.future_state_predictor is not None:
                kwargs["output_hidden_states"] = True
            output = self.vla(input_ids=input_ids, **kwargs)
            if self.future_state_predictor is not None:
                output["predicted_future_states"] = self._predict_future_state(
                    input_ids, output.hidden_states[-1]
                )
            return output
        finally:
            self.injecting_embeddings.current_state = None

    def generate(self, input_ids: torch.Tensor, states: torch.Tensor, **kwargs):
        self.injecting_embeddings.current_state = states
        try:
            return self.vla.generate(input_ids=input_ids, **kwargs)
        finally:
            self.injecting_embeddings.current_state = None


def passthrough_statistics() -> Dict[str, Any]:
    """Avoid scanning multi-GB RLDS files; NormalizationType.NONE ignores these values."""
    zeros = np.zeros(7, dtype=np.float32)
    ones = np.ones(7, dtype=np.float32)
    state_zeros = np.zeros(8, dtype=np.float32)
    state_ones = np.ones(8, dtype=np.float32)
    return {
        "action": {
            "mean": zeros, "std": ones, "min": -ones, "max": ones,
            "q01": -ones, "q99": ones,
        },
        "proprio": {
            "mean": state_zeros, "std": state_ones, "min": -state_ones,
            "max": state_ones, "q01": -state_ones, "q99": state_ones,
        },
        "num_transitions": 1,
        "num_trajectories": 1,
    }


class LiberoRLDSDataset(IterableDataset):
    def __init__(
        self,
        data_root: Path,
        data_mix: str,
        processor: Any,
        action_tokenizer: ActionTokenizer,
        train: bool,
        use_image_aug: bool,
        use_random_resized_crop: bool,
        use_robust_image_aug: bool,
        primary_only_random_resized_crop: bool,
        shuffle_buffer_size: int,
        image_size: int,
        state_stats_path: Path,
        predict_future_state: bool,
        unsupervised_prefix_tokens: bool = False,
        standard_action_only: bool = False,
        task_weighting: str = "none",
        seed: int = 42,
    ):
        super().__init__()
        if image_size < 32 or image_size % 32 != 0:
            raise ValueError(
                f"image_size must be a positive multiple of 32 for Qwen3-VL, got {image_size}"
            )
        self.processor = processor
        self.action_tokenizer = action_tokenizer
        self.predict_future_state = predict_future_state
        self.unsupervised_prefix_tokens = unsupervised_prefix_tokens
        self.standard_action_only = standard_action_only
        if self.predict_future_state and self.unsupervised_prefix_tokens:
            raise ValueError(
                "predict_future_state and unsupervised_prefix_tokens are mutually exclusive"
            )
        if task_weighting not in TASK_WEIGHTING_MODES:
            raise ValueError(
                f"Unknown task_weighting={task_weighting!r}; "
                f"expected one of {sorted(TASK_WEIGHTING_MODES)}"
            )
        # Weighting is deliberately training-only. Rejection sampling with a
        # common divisor of 2 realizes relative weights weak:normal:object =
        # 2:1:0.5 without duplicating identical augmented frames.
        self.task_weighting = task_weighting if train else "none"
        self.seed = int(seed)
        if self.standard_action_only and self.predict_future_state:
            raise ValueError("standard_action_only cannot predict future state")
        stats = json.loads(state_stats_path.read_text(encoding="utf-8"))["norm_stats"]["state"]
        source_q01 = np.asarray(stats["q01"], dtype=np.float32)
        source_q99 = np.asarray(stats["q99"], dtype=np.float32)
        if source_q01.shape != (8,) or source_q99.shape != (8,):
            raise ValueError(f"Raw LIBERO state stats must contain 8 values: {state_stats_path}")
        # Finger joints have opposite signs. Their separation is a physical
        # aperture; map its robust closed/open endpoints to one scalar matching
        # DROID's 0=closed, 1=open convention.
        self.closed_width = float(source_q01[6] - source_q99[7])
        self.open_width = float(source_q99[6] - source_q01[7])
        if not self.open_width > self.closed_width:
            raise ValueError(
                f"Invalid LIBERO gripper width range: {self.closed_width}..{self.open_width}"
            )
        self.state_q01 = np.concatenate((source_q01[:6], np.asarray([0.0], np.float32)))
        self.state_q99 = np.concatenate((source_q99[:6], np.asarray([1.0], np.float32)))
        self.state_span = np.maximum(self.state_q99 - self.state_q01, 1e-8)

        if data_mix not in OXE_NAMED_MIXTURES:
            raise ValueError(f"Unknown RLDS mixture: {data_mix}")
        mixture = OXE_NAMED_MIXTURES[data_mix]
        actual = {name for name, _ in mixture}
        if not actual or not actual.issubset(LIBERO_DATASETS):
            raise ValueError(
                f"{data_mix} must contain only supported LIBERO suites; got {sorted(actual)}"
            )
        for dataset_name in sorted(actual):
            dataset_path = data_root / dataset_name / "1.0.0"
            if not (dataset_path / "dataset_info.json").is_file():
                raise FileNotFoundError(f"Missing LIBERO RLDS suite: {dataset_path}")

        dataset_kwargs, weights = get_oxe_dataset_kwargs_and_weights(
            data_root,
            mixture,
            load_camera_views=("primary", "wrist"),
            load_depth=False,
            load_proprio=True,
            load_language=True,
            action_proprio_normalization_type=NormalizationType.NONE,
        )
        for kwargs in dataset_kwargs:
            # OXE normally inserts a separator padding dimension between EEF and
            # gripper state. The state MLP consumes the physical 6+2 values only.
            kwargs["state_obs_keys"] = ["EEF_state", "gripper_state"]
            kwargs["dataset_statistics"] = passthrough_statistics()

        frame_kwargs: Dict[str, Any] = {
            "resize_size": (image_size, image_size),
            "num_parallel_calls": 4,
        }
        if use_image_aug:
            frame_kwargs["image_augment_kwargs"] = build_image_augment_kwargs(
                use_random_resized_crop,
                use_robust_image_aug,
                primary_only_random_resized_crop,
            )

        self.dataset, self.dataset_length, self.dataset_statistics = make_interleaved_dataset(
            dataset_kwargs_list=dataset_kwargs,
            sample_weights=weights,
            train=train,
            shuffle_buffer_size=shuffle_buffer_size,
            balance_weights=True,
            traj_transform_kwargs={
                "window_size": 1,
                "future_action_window_size": action_tokenizer.chunk_size - 1,
                "skip_unlabeled": True,
                "goal_relabeling_strategy": None,
            },
            frame_transform_kwargs=frame_kwargs,
            traj_transform_threads=4,
            traj_read_threads=4,
        )

    def __len__(self) -> int:
        return int(self.dataset_length)

    def collapse_gripper_state(self, raw_state: np.ndarray) -> np.ndarray:
        raw_state = np.asarray(raw_state, dtype=np.float32)
        if raw_state.shape != (8,) or not np.isfinite(raw_state).all():
            raise ValueError(f"Expected finite raw LIBERO state (8,), got {raw_state.shape}")
        width = float(raw_state[6] - raw_state[7])
        openness = np.clip(
            (width - self.closed_width) / (self.open_width - self.closed_width),
            0.0,
            1.0,
        )
        return np.concatenate((raw_state[:6], np.asarray([openness], np.float32)))

    def __iter__(self) -> Iterator[Dict[str, torch.Tensor]]:
        worker = get_worker_info()
        worker_id = worker.id if worker else 0
        worker_count = worker.num_workers if worker else 1
        rank = dist.get_rank() if dist.is_initialized() else 0
        world_size = dist.get_world_size() if dist.is_initialized() else 1
        shard_id = rank * worker_count + worker_id
        shard_count = world_size * worker_count
        rng = np.random.default_rng(self.seed + shard_id)
        for index, sample in enumerate(self.dataset.as_numpy_iterator()):
            if index % shard_count != shard_id:
                continue
            if self.task_weighting == "gripper_transition_3x":
                action = np.asarray(sample["action"], dtype=np.float32)
                if action.ndim != 2 or action.shape[-1] != 7:
                    raise ValueError(
                        "Expected LIBERO action window with shape [T, 7], "
                        f"got {action.shape}"
                    )
                gripper_transition = (
                    float(np.max(action[:, -1]) - np.min(action[:, -1])) >= 0.5
                )
                # Keep every transition window and one third of other windows.
                # This preserves natural suite-size sampling before applying a
                # task-agnostic 3x relative emphasis around gripper changes.
                if rng.random() >= (1.0 if gripper_transition else 1.0 / 3.0):
                    continue
            elif self.task_weighting in {
                "targeted_weak_v1",
                "targeted_weak_v2",
                "targeted_weak_v2_gripper3x",
            }:
                dataset_name = sample["dataset_name"]
                if isinstance(dataset_name, np.ndarray):
                    dataset_name = dataset_name.item()
                if isinstance(dataset_name, bytes):
                    dataset_name = dataset_name.decode("utf-8")
                instruction = sample["task"]["language_instruction"]
                if isinstance(instruction, np.ndarray):
                    instruction = instruction.item()
                if isinstance(instruction, bytes):
                    instruction = instruction.decode("utf-8")
                instruction = str(instruction).strip().lower()
                if self.task_weighting in {
                    "targeted_weak_v2",
                    "targeted_weak_v2_gripper3x",
                }:
                    from train.libero_targeted_weak_v2 import task_weight
                    base_weight = task_weight(str(dataset_name), instruction)
                    if self.task_weighting == "targeted_weak_v2_gripper3x":
                        action = np.asarray(sample["action"], dtype=np.float32)
                        if action.ndim != 2 or action.shape[-1] != 7:
                            raise ValueError(
                                "Expected LIBERO action window with shape [T, 7], "
                                f"got {action.shape}"
                            )
                        # Each sample contains a sliding 20-step future window.
                        # A window spanning both gripper states is therefore a
                        # temporal neighborhood around an open/close transition.
                        gripper_transition = (
                            float(np.max(action[:, -1]) - np.min(action[:, -1])) >= 0.5
                        )
                        # Combined relative weight is task_weight * {1, 3}.
                        # The maximum possible weight is 4 * 3 = 12.
                        accept_probability = base_weight * (
                            3.0 if gripper_transition else 1.0
                        ) / 12.0
                    else:
                        # Rejection sampling realizes relative 4:2:1:0.5 weights.
                        accept_probability = base_weight / 4.0
                elif instruction in TARGETED_LIBERO_WEAK_TASKS:
                    accept_probability = 1.0       # relative weight 2.0
                elif str(dataset_name) == "libero_object_no_noops":
                    accept_probability = 0.25      # relative weight 0.5
                else:
                    accept_probability = 0.5       # relative weight 1.0
                if rng.random() >= accept_probability:
                    continue
            yield self.transform(sample)

    def transform(self, sample: Dict[str, Any]) -> Dict[str, torch.Tensor]:
        action_ids = self.action_tokenizer.encode(sample["action"])
        action_text = "".join(self.processor.tokenizer.convert_ids_to_tokens(action_ids))
        instruction = sample["task"]["language_instruction"]
        if isinstance(instruction, np.ndarray):
            instruction = instruction.item()
        if isinstance(instruction, bytes):
            instruction = instruction.decode("utf-8")

        raw_state = self.collapse_gripper_state(sample["observation"]["proprio"][0])
        training = getattr(self, "train", True)
        if STATE_AUG == "noise" and training and np.random.random() < STATE_NOISE_P:
            # Coarse pose stays informative; fine localization must come from images.
            raw_state = np.array(raw_state, dtype=np.float32, copy=True)
            raw_state[:3] += np.random.normal(0.0, STATE_NOISE_POS_M, 3).astype(np.float32)
            raw_state[3:6] += np.random.normal(0.0, STATE_NOISE_ROT_RAD, 3).astype(np.float32)
        normalized_state = np.clip(
            2.0 * (raw_state - self.state_q01) / self.state_span - 1.0, -1.0, 1.0
        ).astype(np.float32)
        # Hide the absolute EEF pose (gripper kept) so the policy must localize from images.
        if STATE_AUG == "mask" or (STATE_AUG == "drop" and training and np.random.random() < STATE_DROP_P):
            normalized_state[:6] = 0.0

        if self.predict_future_state:
            raw_future_state = self.collapse_gripper_state(sample["future_proprio"])
            normalized_future_state = np.clip(
                2.0 * (raw_future_state - self.state_q01) / self.state_span - 1.0,
                -1.0,
                1.0,
            ).astype(np.float32)
            future_state_text = "".join(FUTURE_STATE_TOKENS)
            assistant_content = (
                f"Next state:\n{FUTURE_STATE_START_TOKEN}{future_state_text}"
                f"{FUTURE_STATE_END_TOKEN}\nAction:\n"
                f"<|action_start|>{action_text}<|action_end|>"
            )
        elif self.unsupervised_prefix_tokens:
            raw_future_state = raw_state.copy()
            normalized_future_state = normalized_state.copy()
            latent_text = "".join(LATENT_PREFIX_TOKENS)
            assistant_content = (
                f"Plan:\n{latent_text}\nAction:\n"
                f"<|action_start|>{action_text}<|action_end|>"
            )
        else:
            raw_future_state = raw_state.copy()
            normalized_future_state = normalized_state.copy()
            assistant_content = f"Action:\n<|action_start|>{action_text}<|action_end|>"

        main_image = Image.fromarray(sample["observation"]["image_primary"][0]).convert("RGB")
        wrist_image = Image.fromarray(sample["observation"]["image_wrist"][0]).convert("RGB")
        images = [main_image, wrist_image]
        user_content = [
            {"type": "text", "text": "Main camera:"},
            {"type": "image", "image": main_image},
            {"type": "text", "text": "Wrist camera:"},
            {"type": "image", "image": wrist_image},
            {
                "type": "text",
                "text": (
                    f"Task: {str(instruction).strip().lower()}\n"
                    "Robot state:\n"
                    f"End-effector pose: {STATE_TOKENS[0]}\n"
                    f"Gripper state: {STATE_TOKENS[1]}\n"
                    "Based on the current observations and robot state, what is the next action?"
                ),
            },
        ]
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a robot manipulation policy. Based on the main camera, wrist camera, "
                    "current end-effector pose, gripper state, and task instruction, "
                    + (
                        "first predict the robot state after the next one-second action chunk, "
                        "then predict that action chunk."
                        if self.predict_future_state
                        else (
                            "use two latent planning slots, then predict the next one-second "
                            "action chunk."
                            if self.unsupervised_prefix_tokens
                            else "predict the next one-second action chunk."
                        )
                    )
                ),
            },
            {"role": "user", "content": user_content},
            {
                "role": "assistant",
                "content": assistant_content,
            },
        ]
        full_text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=False)
        full = self.processor(text=[full_text], images=images, return_tensors="pt")

        input_ids = full.input_ids[0]
        labels = torch.full_like(input_ids, IGNORE_INDEX)
        action_mask = torch.zeros_like(input_ids, dtype=torch.bool)
        future_state_mask = torch.zeros_like(input_ids, dtype=torch.bool)
        im_start = self.processor.tokenizer.convert_tokens_to_ids("<|im_start|>")
        im_end = self.processor.tokenizer.convert_tokens_to_ids("<|im_end|>")
        assistant = self.processor.tokenizer.encode("assistant", add_special_tokens=False)[0]
        action_start = self.processor.tokenizer.convert_tokens_to_ids("<|action_start|>")
        action_end = self.processor.tokenizer.convert_tokens_to_ids("<|action_end|>")
        future_state_ids = set(
            self.processor.tokenizer.convert_tokens_to_ids(FUTURE_STATE_TOKENS)
        ) if self.predict_future_state else set()
        latent_prefix_ids = set(
            self.processor.tokenizer.convert_tokens_to_ids(LATENT_PREFIX_TOKENS)
        ) if self.unsupervised_prefix_tokens else set()
        in_assistant = False
        in_action = False
        ids = input_ids.tolist()
        for index, token_id in enumerate(ids):
            if token_id == im_start and index + 1 < len(ids) and ids[index + 1] == assistant:
                in_assistant = True
            if in_assistant:
                labels[index] = input_ids[index]
            if token_id == action_start:
                in_action = True
                if self.standard_action_only:
                    continue
            if token_id == action_end and self.standard_action_only:
                in_action = False
                continue
            # Boundaries remain supervised text, but only actual ActionCodec
            # code tokens belong to the action objective/BAR blocks.
            if in_action and token_id in self.action_tokenizer.valid_token_ids:
                action_mask[index] = True
            if token_id == action_end:
                in_action = False
            if token_id in future_state_ids:
                future_state_mask[index] = True
            if token_id in latent_prefix_ids:
                labels[index] = IGNORE_INDEX
            if token_id == im_end and in_assistant:
                in_assistant = False

        if self.unsupervised_prefix_tokens:
            latent_positions = torch.tensor(
                [token_id in latent_prefix_ids for token_id in ids], dtype=torch.bool
            )
            if int(latent_positions.sum().item()) != 2:
                raise RuntimeError(
                    "latent2_ar requires exactly two latent prefix tokens per sample; "
                    f"found {int(latent_positions.sum().item())}"
                )
            if not torch.all(labels[latent_positions] == IGNORE_INDEX):
                raise RuntimeError("latent2_ar prefix tokens must not have supervised labels")

        if self.standard_action_only:
            action_targets = int(action_mask.sum().item())
            if action_targets != self.action_tokenizer.num_tokens:
                raise RuntimeError(
                    f"Standard SFT requires exactly {self.action_tokenizer.num_tokens} "
                    f"ActionCodec targets, found {action_targets}"
                )
            if future_state_mask.any():
                raise RuntimeError("Standard SFT sample unexpectedly contains future-state targets")
            labels[~action_mask] = IGNORE_INDEX

        stitched = Image.new("RGB", (512, 256))
        stitched.paste(main_image.resize((256, 256)), (0, 0))
        stitched.paste(wrist_image.resize((256, 256)), (256, 0))
        log_image = torch.from_numpy(np.asarray(stitched).copy()).permute(2, 0, 1).float() / 255
        return {
            "input_ids": input_ids,
            "labels": labels,
            "attention_mask": torch.ones_like(input_ids),
            "pixel_values": full.pixel_values,
            "image_grid_thw": full.image_grid_thw,
            "action_token_mask": action_mask,
            "future_state_token_mask": future_state_mask,
            "state": torch.from_numpy(normalized_state),
            "raw_state": torch.from_numpy(raw_state.copy()),
            "future_state": torch.from_numpy(normalized_future_state),
            "raw_future_state": torch.from_numpy(raw_future_state.copy()),
            "log_image": log_image,
        }


@dataclass
class FrankaTask0Episode:
    name: str
    instruction: str
    states: np.ndarray
    gripper_target: np.ndarray
    action_chunks: np.ndarray
    valid_start_indices: np.ndarray
    video_left: Path
    video_right: Path
    video_wrist: Path


def _episode_instruction(episode: "FrankaTask0Episode", train: bool, np_rng) -> str:
    if TARGETS is None:
        return episode.instruction
    target = TARGETS[episode.name]
    tube, basket = np.asarray(target["tube"]), np.asarray(target["basket"])
    if train:
        tube = tube + np_rng.uniform(-TARGET_JITTER_TUBE_M, TARGET_JITTER_TUBE_M, 2)
        basket = basket + np_rng.uniform(-TARGET_JITTER_BASKET_M, TARGET_JITTER_BASKET_M, 2)
    return target_instruction(episode.instruction, tube, basket)


class VideoFrameReader:
    def __init__(self) -> None:
        self.path: Optional[Path] = None
        self.capture: Optional[cv2.VideoCapture] = None

    def read(self, path: Path, index: int) -> Image.Image:
        if self.path != path:
            self.close()
            self.capture = cv2.VideoCapture(str(path))
            self.path = path
        assert self.capture is not None
        self.capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = self.capture.read()
        if not ok:
            raise RuntimeError(f"Could not read frame {index} from {path}")
        return Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
        self.capture = None
        self.path = None


class FrankaTask0Dataset(LiberoRLDSDataset):
    """Task{x} parquet/video reader with the exact DROID 15x7 contract."""

    def __init__(
        self,
        data_root: Path,
        processor: Any,
        action_tokenizer: ActionTokenizer,
        train: bool,
        predict_future_state: bool,
        seed: int,
        gripper_transition_weight: int = 3,
        validation_episodes: int = 2,
        state_stats_path: Optional[Path] = None,
        tasks_path: Optional[Path] = None,
        image_size: int = 256,
        use_image_aug: bool = False,
        use_random_resized_crop: bool = False,
        use_robust_image_aug: bool = False,
        primary_only_random_resized_crop: bool = False,
        train_all_episodes: bool = False,
        trajectory_sampling: bool = False,
    ) -> None:
        IterableDataset.__init__(self)
        if action_tokenizer.chunk_size != 15 or action_tokenizer.action_dim != 7:
            raise ValueError("Task0 requires DROID ActionCodec embodiment 2 (15x7)")
        self.processor = processor
        self.action_tokenizer = action_tokenizer
        self.predict_future_state = predict_future_state
        self.unsupervised_prefix_tokens = False
        self.standard_action_only = False
        self.task_weighting = "none"
        self.train = train
        self.trajectory_sampling = trajectory_sampling
        self.seed = int(seed)
        self.iteration = 0
        self.image_size = int(image_size)
        self.use_image_aug = bool(use_image_aug and train)
        self.use_random_resized_crop = bool(
            self.use_image_aug and use_random_resized_crop
        )
        self.use_robust_image_aug = bool(
            self.use_image_aug and use_robust_image_aug
        )
        self.primary_only_random_resized_crop = bool(
            primary_only_random_resized_crop
        )
        if self.image_size != 256:
            raise ValueError(
                f"DROID-compatible Task{{x}} training requires image_size=256, got {image_size}"
            )
        task_match = re.fullmatch(r"Task(\d+)", data_root.name)
        if task_match is None:
            raise ValueError(
                f"Data root must be named Task{{index}}, got {data_root.name!r}: {data_root}"
            )
        self.task_index = int(task_match.group(1))
        tasks_path = tasks_path or data_root.parent / "tasks.json"
        if not tasks_path.is_file():
            raise FileNotFoundError(f"Task prompt mapping is missing: {tasks_path}")
        task_prompts = json.loads(tasks_path.read_text(encoding="utf-8"))
        task_key = str(self.task_index)
        if task_key not in task_prompts:
            raise KeyError(
                f"Task index {self.task_index} from {data_root.name!r} is absent in {tasks_path}"
            )
        self.task_instruction = str(task_prompts[task_key]).strip()
        if not self.task_instruction:
            raise ValueError(f"Empty prompt for task index {self.task_index} in {tasks_path}")
        self.episodes: List[FrankaTask0Episode] = []
        offline_cache_episodes = 0
        episode_dirs = sorted(path for path in data_root.iterdir() if path.is_dir())
        if len(episode_dirs) <= validation_episodes:
            raise RuntimeError(f"Need more than {validation_episodes} episodes below {data_root}")
        chosen = (episode_dirs if train_all_episodes else episode_dirs[:-validation_episodes]) if train else episode_dirs[-validation_episodes:]
        if train and train_all_episodes:
            print(f"[DataSplit] All {len(chosen)} episodes used for training; last {validation_episodes} also used for validation (overlapping, not held out).")
        for episode_dir in chosen:
            cache_path = episode_dir / "data" / FRANKA_V4_CACHE_NAME
            parquet_paths = sorted(episode_dir.glob("data/chunk-*/*.parquet"))
            if len(parquet_paths) != 1:
                continue
            if not cache_path.is_file():
                raise FileNotFoundError(
                    f"Missing required {FRANKA_CACHE_VERSION} cache: {cache_path}. Run "
                    f"preprocess_franka_task_measured_eef_{FRANKA_CACHE_VERSION}.py first."
                )
            source = parquet_paths[0]
            with np.load(cache_path, allow_pickle=False) as cache:
                states = cache["eef_state"].astype(np.float32, copy=False)
                target_open = cache["gripper_target_open"].astype(np.float32, copy=False)
                action_chunks = cache["droid_action_chunks"].astype(np.float32, copy=False)
                valid_start_indices = cache["valid_start_indices"].astype(np.int64, copy=False)
                action_representation = str(cache["action_representation"])
                rotation_execution = str(cache["rotation_execution"])
                source_size = int(cache["source_parquet_size"])
                source_mtime_ns = int(cache["source_parquet_mtime_ns"])
                if FRANKA_CACHE_VERSION == "v5" and str(cache["valid_start_semantics"]) != os.environ.get("FRANKA_WINDOW_FILTER_SEMANTICS", "state_first15_activity_or_gripper_hold_v5"):
                    raise ValueError(f"Wrong v5 sampling semantics for this run; rebuild matching caches: {cache_path}")
            if states.shape != (len(target_open), 7) or action_chunks.shape != (
                max(0, len(states) - 30), 15, 7
            ):
                raise ValueError(f"Invalid v4 cache shapes: {cache_path}")
            if (
                valid_start_indices.ndim != 1
                or np.any(valid_start_indices < 0)
                or np.any(valid_start_indices >= len(action_chunks))
                or np.any(valid_start_indices[1:] <= valid_start_indices[:-1])
            ):
                raise ValueError(f"Invalid v4 valid_start_indices: {cache_path}")
            if action_representation != FRANKA_V4_ACTION_REPRESENTATION:
                raise ValueError(
                    f"Wrong v4 action representation in {cache_path}: {action_representation!r}"
                )
            if rotation_execution != FRANKA_V4_ROTATION_EXECUTION:
                raise ValueError(
                    f"v4 cache is not world-frame left-multiply: {cache_path}: "
                    f"{rotation_execution!r}"
                )
            if source_size != source.stat().st_size or source_mtime_ns != source.stat().st_mtime_ns:
                raise ValueError(f"Stale v4 cache does not match source parquet: {cache_path}")
            if not np.isfinite(states).all() or not np.isfinite(action_chunks).all():
                raise ValueError(f"Non-finite v4 data: {cache_path}")
            offline_cache_episodes += 1
            videos = episode_dir / "videos" / "chunk-000"
            left = videos / "observation.images.left" / "episode_000000.mp4"
            right = videos / "observation.images.right" / "episode_000000.mp4"
            wrist = videos / "observation.images.front" / "episode_000000.mp4"
            if not left.is_file() or not right.is_file() or not wrist.is_file() or len(states) < 31:
                continue
            episode = FrankaTask0Episode(
                episode_dir.name, self.task_instruction, states,
                target_open, action_chunks, valid_start_indices,
                left, right, wrist,
            )
            self.episodes.append(episode)
        if not self.episodes:
            raise RuntimeError(f"No valid Task0 episodes found below {data_root}")
        if state_stats_path is None or not state_stats_path.is_file():
            raise FileNotFoundError(f"DROID EEF state statistics missing: {state_stats_path}")
        stats = json.loads(state_stats_path.read_text(encoding="utf-8"))["norm_stats"]["state"]
        self.state_q01 = np.asarray(stats["q01"], dtype=np.float32)
        self.state_q99 = np.asarray(stats["q99"], dtype=np.float32)
        if self.state_q01.shape != (7,) or self.state_q99.shape != (7,):
            raise ValueError(f"DROID EEF state stats must be 7D: {state_stats_path}")
        self.state_span = np.maximum(self.state_q99 - self.state_q01, 1e-8)
        self.valid_starts: List[Tuple[int, int]] = []
        weight = max(1, int(gripper_transition_weight))
        unweighted_windows = 0
        transition_windows = 0
        for episode_index, episode in enumerate(self.episodes):
            for current in episode.valid_start_indices.tolist():
                gripper_changes = np.ptp(episode.action_chunks[current, :, 6]) >= 0.5
                repeats = weight if gripper_changes else 1
                self.valid_starts.extend([(episode_index, current)] * repeats)
                unweighted_windows += 1
                transition_windows += int(gripper_changes)
        print(
            f"[Task{self.task_index}] train={train}, prompt={self.task_instruction!r}, "
            f"episodes={len(self.episodes)}, windows={len(self.valid_starts)}, "
            f"base_windows={unweighted_windows}, gripper_transition_windows={transition_windows}, "
            f"gripper_transition_weight={weight}, state=EEF-rotvec+open01, "
            f"images=left_or_right_p0.5+wrist resized_bilinear_"
            f"{self.image_size}x{self.image_size}, aug={self.use_image_aug}, "
            f"primary_rrc={self.use_random_resized_crop and self.primary_only_random_resized_crop}, "
            f"both_color_noise={self.use_robust_image_aug}, "
            f"offline_{FRANKA_CACHE_VERSION}_cache={offline_cache_episodes}/{len(self.episodes)}, "
            f"trajectory_sampling={self.trajectory_sampling}, "
            f"rotation=world_frame_left_multiply"
        )

    def __len__(self) -> int:
        return len(self.valid_starts)

    def collapse_gripper_state(self, raw_state: np.ndarray) -> np.ndarray:
        state = np.asarray(raw_state, dtype=np.float32)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError(f"Expected finite Task0 EEF state (7,), got {state.shape}")
        return state

    @staticmethod
    def _compose_action(episode: FrankaTask0Episode, current: int) -> np.ndarray:
        return episode.action_chunks[current]

    def _resize_or_crop(
        self, image: Image.Image, rng: random.Random, crop: bool
    ) -> Image.Image:
        if not crop:
            return image.resize(
                (self.image_size, self.image_size), Image.Resampling.BILINEAR
            )
        width, height = image.size
        area = width * height
        for _ in range(10):
            target_area = area * rng.uniform(0.9, 1.0)
            aspect_ratio = np.exp(rng.uniform(np.log(0.95), np.log(1.05)))
            crop_width = int(round(np.sqrt(target_area * aspect_ratio)))
            crop_height = int(round(np.sqrt(target_area / aspect_ratio)))
            if 0 < crop_width <= width and 0 < crop_height <= height:
                left = rng.randint(0, width - crop_width)
                top = rng.randint(0, height - crop_height)
                return image.crop(
                    (left, top, left + crop_width, top + crop_height)
                ).resize(
                    (self.image_size, self.image_size), Image.Resampling.BILINEAR
                )
        crop_size = min(width, height)
        left = (width - crop_size) // 2
        top = (height - crop_size) // 2
        return image.crop((left, top, left + crop_size, top + crop_size)).resize(
            (self.image_size, self.image_size), Image.Resampling.BILINEAR
        )

    def _instruction(self, episode: FrankaTask0Episode, np_rng) -> str:
        return _episode_instruction(episode, self.train, np_rng)

    def _augment_image(
        self,
        image: Image.Image,
        rng: random.Random,
        np_rng: np.random.Generator,
        crop: bool,
    ) -> np.ndarray:
        image = self._resize_or_crop(image.convert("RGB"), rng, crop)
        if self.use_image_aug:
            image = ImageEnhance.Brightness(image).enhance(rng.uniform(0.9, 1.1))
            image = ImageEnhance.Contrast(image).enhance(rng.uniform(0.9, 1.1))
            image = ImageEnhance.Color(image).enhance(rng.uniform(0.9, 1.1))
        if self.use_robust_image_aug:
            hsv = np.asarray(image.convert("HSV")).copy()
            hue_delta = int(round(rng.uniform(-0.02, 0.02) * 255.0))
            hsv[..., 0] = (
                hsv[..., 0].astype(np.int16) + hue_delta
            ).astype(np.uint8)
            image = Image.fromarray(hsv, mode="HSV").convert("RGB")
            pixels = np.asarray(image, dtype=np.float32) / 255.0
            if rng.random() < 0.2:
                pixels += np_rng.normal(0.0, 0.01, pixels.shape).astype(np.float32)
            return np.clip(pixels * 255.0, 0.0, 255.0).astype(np.uint8)
        return np.asarray(image)

    def __iter__(self) -> Iterator[Dict[str, torch.Tensor]]:
        worker = get_worker_info()
        worker_id = worker.id if worker else 0
        worker_count = worker.num_workers if worker else 1
        rank = dist.get_rank() if dist.is_initialized() else 0
        world = dist.get_world_size() if dist.is_initialized() else 1
        shard_id, shard_count = rank * worker_count + worker_id, world * worker_count
        indices = list(range(len(self.valid_starts)))
        iteration = self.iteration
        if self.train and not self.trajectory_sampling:
            random.Random(self.seed + iteration).shuffle(indices)
        if self.train:
            self.iteration += 1
        rng_seed = self.seed + 1_000_003 * iteration + 10_007 * shard_id
        rng = random.Random(rng_seed)
        np_rng = np.random.default_rng(rng_seed)
        if self.trajectory_sampling:
            from train.franka_trajectory_sampling import EpisodeFrameCache, trajectory_groups
            groups = trajectory_groups(self.valid_starts, shard_id, shard_count, rng_seed, self.train)
            indices = [item for _, items in groups for item in items]
            starts_by_episode = {
                episode: sorted({self.valid_starts[item][1] for item in items})
                for episode, items in groups
            }
            reader = EpisodeFrameCache()
        else:
            indices = indices[shard_id::shard_count]
            reader = VideoFrameReader()
        cached_episode = None
        try:
            for item_index in indices:
                episode_index, current = self.valid_starts[item_index]
                episode = self.episodes[episode_index]
                if self.trajectory_sampling and cached_episode != episode_index:
                    camera_paths = [episode.video_left, episode.video_wrist]
                    if self.train:
                        camera_paths.append(episode.video_right)
                    requests = {path: starts_by_episode[episode_index] for path in camera_paths}
                    resize_sizes = {}
                    for path in camera_paths:
                        crop = self.use_random_resized_crop and (
                            path != episode.video_wrist or not self.primary_only_random_resized_crop
                        )
                        if not crop:
                            resize_sizes[path] = (self.image_size, self.image_size)
                    reader.prepare(requests, resize_sizes)
                    cached_episode = episode_index
                primary_path = (
                    episode.video_left
                    if (not self.train or rng.random() < 0.5)
                    else episode.video_right
                )
                primary = self._augment_image(
                    reader.read(primary_path, current), rng, np_rng,
                    crop=self.use_random_resized_crop,
                )
                wrist = self._augment_image(
                    reader.read(episode.video_wrist, current), rng, np_rng,
                    crop=(
                        self.use_random_resized_crop
                        and not self.primary_only_random_resized_crop
                    ),
                )
                sample = {
                    "action": self._compose_action(episode, current),
                    "task": {"language_instruction": self._instruction(episode, np_rng)},
                    "observation": {
                        "proprio": episode.states[current][None, :],
                        "image_primary": primary[None, ...],
                        "image_wrist": wrist[None, ...],
                    },
                    "future_proprio": episode.states[current + 30],
                }
                yield self.transform(sample)
        finally:
            reader.close()


class Collator:
    def __init__(self, pad_token_id: int):
        self.pad_token_id = pad_token_id

    def __call__(self, items: List[Dict[str, torch.Tensor]]) -> Dict[str, torch.Tensor]:
        pad = torch.nn.utils.rnn.pad_sequence
        return {
            "input_ids": pad([x["input_ids"] for x in items], True, self.pad_token_id),
            "labels": pad([x["labels"] for x in items], True, IGNORE_INDEX),
            "attention_mask": pad([x["attention_mask"] for x in items], True, 0),
            "pixel_values": torch.cat([x["pixel_values"] for x in items]),
            "image_grid_thw": torch.cat([x["image_grid_thw"] for x in items]),
            "action_token_mask": pad([x["action_token_mask"] for x in items], True, False),
            "future_state_token_mask": pad(
                [x["future_state_token_mask"] for x in items], True, False
            ),
            "states": torch.stack([x["state"] for x in items]),
            "raw_states": torch.stack([x["raw_state"] for x in items]),
            "future_states": torch.stack([x["future_state"] for x in items]),
            "raw_future_states": torch.stack([x["raw_future_state"] for x in items]),
            "log_images": torch.stack([x["log_image"] for x in items]),
        }


def compute_loss(
    logits,
    labels,
    action_mask,
    future_state_mask,
    future_states,
    predicted_future_states: Optional[torch.Tensor],
    text_weight: float,
    action_weight: float,
    future_state_weight: float,
    future_state_regression_weight: float,
):
    logits = logits[:, :-1].contiguous()
    labels = labels[:, 1:].contiguous()
    action_mask = action_mask[:, 1:].contiguous()
    future_state_mask = future_state_mask[:, 1:].contiguous()
    valid = labels != IGNORE_INDEX
    losses = F.cross_entropy(
        logits.view(-1, logits.shape[-1]), labels.view(-1),
        ignore_index=IGNORE_INDEX, reduction="none",
    ).view_as(labels)
    text_targets = valid & ~action_mask & ~future_state_mask
    action_targets = valid & action_mask
    if not action_targets.any():
        raise RuntimeError("No valid supervised action tokens in this batch")
    text_loss = losses[text_targets].mean() if text_targets.any() else logits.new_zeros(())
    action_loss = losses[action_targets].mean()
    if text_weight > 0.0 and not text_targets.any():
        raise RuntimeError("text_loss_weight>0 but this batch has no supervised text tokens")
    if not torch.isfinite(text_loss) or not torch.isfinite(action_loss):
        raise RuntimeError("Non-finite supervised text/action loss")
    predictions = logits.argmax(-1)
    accuracy = (predictions[valid & action_mask] == labels[valid & action_mask]).float().mean()
    if predicted_future_states is not None:
        expected_count = logits.shape[0] * 2
        actual_count = int(future_state_mask.sum().item())
        if actual_count != expected_count:
            raise RuntimeError(
                f"Expected {expected_count} future-state targets, found {actual_count}"
            )
        future_state_loss = losses[future_state_mask].mean()
        future_state_regression_loss = F.smooth_l1_loss(
            predicted_future_states.float(), future_states.float(), beta=0.05
        )
        future_state_accuracy = (
            predictions[future_state_mask] == labels[future_state_mask]
        ).float().mean()
        denominator = text_weight + action_weight + future_state_weight
        total = (
            text_weight * text_loss
            + action_weight * action_loss
            + future_state_weight * future_state_loss
        ) / denominator
        total = total + future_state_regression_weight * future_state_regression_loss
    else:
        zero = logits.new_zeros(())
        future_state_loss = zero
        future_state_regression_loss = zero
        future_state_accuracy = zero
        total = (text_weight * text_loss + action_weight * action_loss) / (
            text_weight + action_weight
        )
    return (
        total,
        text_loss.detach(),
        action_loss.detach(),
        future_state_loss.detach(),
        future_state_regression_loss.detach(),
        accuracy.detach(),
        future_state_accuracy.detach(),
    )


@torch.no_grad()
def validate_loss(
    model, loader, device: int, cfg: "FinetuneConfig"
) -> Dict[str, float]:
    model.eval()
    totals = []
    accuracies = []
    future_losses = []
    future_regression_losses = []
    future_accuracies = []
    for index, batch in enumerate(loader):
        if index >= cfg.val_batches:
            break
        output = model(
            input_ids=batch["input_ids"].to(device),
            states=batch["states"].to(device),
            attention_mask=batch["attention_mask"].to(device),
            pixel_values=batch["pixel_values"].to(device, dtype=torch.bfloat16),
            image_grid_thw=batch["image_grid_thw"].to(device),
        )
        loss, _, _, future_loss, future_regression_loss, accuracy, future_accuracy = compute_loss(
            output.logits,
            batch["labels"].to(device),
            batch["action_token_mask"].to(device),
            batch["future_state_token_mask"].to(device),
            batch["future_states"].to(device),
            output.get("predicted_future_states"),
            cfg.text_loss_weight,
            cfg.action_loss_weight,
            cfg.future_state_loss_weight,
            cfg.future_state_regression_weight,
        )
        totals.append(loss.item())
        accuracies.append(accuracy.item())
        if cfg.predict_future_state:
            future_losses.append(future_loss.item())
            future_regression_losses.append(future_regression_loss.item())
            future_accuracies.append(future_accuracy.item())
    model.train()
    metrics = {
        "val_loss": float(np.mean(totals)),
        "val_action_token_accuracy": float(np.mean(accuracies)),
    }
    if cfg.predict_future_state:
        metrics.update({
            "val_future_state_token_loss": float(np.mean(future_losses)),
            "val_future_state_regression_loss": float(np.mean(future_regression_losses)),
            "val_future_state_token_accuracy": float(np.mean(future_accuracies)),
        })
    return metrics


@dataclass
class FinetuneConfig:
    base_model: str = "qwen3vl2b"
    vla_path: str = "/mnt/82_store/lyh/Qwen_weights/Qwen3-VL-2B"
    g05_checkpoint_path: str = (
        "/mnt/82_store/lyh/VLA/GalaxeaVLA/checkpoints/"
        "g05-base/checkpoints/model_state_dict.pt"
    )
    actioncodec_model_id: str = "ZibinDong/ActionCodec-Base-RVQft"
    actioncodec_path: str = str(
        PROJECT_ROOT / "actioncodec_libero" / "weights" / "ActionCodec-Base-RVQft"
    )
    data_root_dir: str = "/mnt/82_store/vla/Franka_vr/Task0"
    data_mix: str = "franka_task0"
    joint_task_names: str = ""
    state_stats_path: str = str(
        PROJECT_ROOT / "droid_qwen3vl_actioncodec" / "droid_eef_axis_angle_state_stats.json"
    )
    action_embodiment_id: int = 2
    action_normalization: str = "droid"
    action_gain: float = 1.0
    action_mulaw_mu: float = 0.0
    action_norm_stats_path: str = ""
    reset_action_token_embeddings: bool = False
    reset_new_token_embeddings: bool = False
    action_prediction_mode: str = ACTION_PREDICTION_MODE
    predict_future_state: bool = True
    unsupervised_prefix_tokens: bool = False
    prediction_order: str = "auto"
    standard_action_only: bool = False
    task_weighting: str = "none"
    gripper_transition_weight: int = 3
    validation_episodes: int = 2
    train_all_episodes: bool = False
    trajectory_sampling: bool = False
    tasks_path: str = "/mnt/82_store/vla/Franka_vr/tasks.json"
    future_state_loss_weight: float = 1.0
    future_state_regression_weight: float = 0.25
    resume: bool = False
    resume_path: str = ""
    init_checkpoint_path: str = ""
    batch_size: int = 1
    grad_accumulation_steps: int = 4
    dataloader_num_workers: int = 0
    learning_rate: float = 6e-5
    min_learning_rate: float = 5e-6
    max_steps: int = 4000
    save_steps: int = 1000
    val_steps: int = 1000
    val_batches: int = 50
    shuffle_buffer_size: int = 10000
    val_shuffle_buffer_size: int = 500
    use_image_aug: bool = True
    use_random_resized_crop: bool = True
    use_robust_image_aug: bool = False
    primary_only_random_resized_crop: bool = False
    image_size: int = 256
    text_loss_weight: float = 1.0
    action_loss_weight: float = 1.0
    seed: int = 42
    run_root_dir: Path = Path("runs")
    wandb_project: str = "franka_task0_qwen_actioncodec"
    wandb_entity: str = ""
    run_id_note: str = "droid_pretrained_bar_eef01"


def validate_and_normalize_training_protocol(cfg: FinetuneConfig) -> None:
    if not 0.0 < cfg.min_learning_rate <= cfg.learning_rate:
        raise ValueError(
            "min_learning_rate must satisfy 0 < min_learning_rate <= learning_rate"
        )
    if cfg.action_prediction_mode not in {"token_ar", "block_ar_16"}:
        raise ValueError(
            "action_prediction_mode must be token_ar or block_ar_16, got "
            f"{cfg.action_prediction_mode!r}"
        )
    if cfg.action_prediction_mode == "block_ar_16" and cfg.standard_action_only:
        raise ValueError("LIBERO BAR state-first mode cannot use standard_action_only")
    if cfg.predict_future_state and cfg.unsupervised_prefix_tokens:
        raise ValueError(
            "predict_future_state and unsupervised_prefix_tokens are mutually exclusive"
        )
    expected_order = (
        "state_then_action"
        if cfg.predict_future_state
        else "latent_then_action" if cfg.unsupervised_prefix_tokens else "action_only"
    )
    if cfg.prediction_order == "auto":
        cfg.prediction_order = expected_order
    if cfg.prediction_order != expected_order:
        raise ValueError(
            f"predict_future_state={cfg.predict_future_state}, "
            f"unsupervised_prefix_tokens={cfg.unsupervised_prefix_tokens} require "
            f"prediction_order={expected_order!r}, got {cfg.prediction_order!r}"
        )
    if not cfg.standard_action_only:
        return
    if cfg.base_model.lower() != "qwen3vl2b":
        raise ValueError("Standard LIBERO SFT is fixed to base_model=qwen3vl2b")
    if (
        cfg.predict_future_state
        or cfg.unsupervised_prefix_tokens
        or cfg.prediction_order != "action_only"
    ):
        raise ValueError("Standard LIBERO SFT must predict action tokens only")
    if cfg.text_loss_weight != 0.0 or cfg.action_loss_weight <= 0.0:
        raise ValueError(
            "Standard LIBERO SFT requires text_loss_weight=0 and action_loss_weight>0"
        )


@draccus.wrap()
def finetune(cfg: FinetuneConfig) -> None:
    random.seed(cfg.seed)
    np.random.seed(cfg.seed)
    torch.manual_seed(cfg.seed)
    codec_path = resolve_actioncodec(cfg.actioncodec_model_id, cfg.actioncodec_path)
    distributed = PartialState()
    device = distributed.local_process_index
    torch.cuda.set_device(device)

    # Draccus can deserialize an explicitly supplied empty optional CLI value
    # as the literal string "None". Treat common null spellings as absent so a
    # fresh run cannot accidentally resolve them to PROJECT_ROOT/None.
    def normalize_optional_path(value: Optional[str]) -> str:
        if value is None:
            return ""
        normalized = str(value).strip()
        return "" if normalized.lower() in {"", "none", "null"} else normalized

    cfg.resume_path = normalize_optional_path(cfg.resume_path)
    cfg.init_checkpoint_path = normalize_optional_path(cfg.init_checkpoint_path)

    if cfg.resume and not cfg.resume_path:
        raise ValueError("resume=true requires resume_path")
    if cfg.resume and cfg.init_checkpoint_path:
        raise ValueError("init_checkpoint_path and resume=true are mutually exclusive")
    validate_and_normalize_training_protocol(cfg)
    action_normalization_stats = None
    action_arm_scale = None
    if cfg.action_normalization in {"task0_maxabs", "franka_maxabs", "franka_mulaw", "franka_quantile"}:
        from train.franka_action_normalization import validate_stats
        if FRANKA_CACHE_VERSION != "v5" or not cfg.action_norm_stats_path:
            raise ValueError("task0_maxabs requires v5 and action_norm_stats_path")
        action_normalization_stats = json.loads(Path(cfg.action_norm_stats_path).read_text())
        action_arm_scale = validate_stats(action_normalization_stats)
        if cfg.action_normalization == "franka_quantile" and action_normalization_stats.get("method") != "franka_real_q01q99_symmetric_v1":
            raise ValueError("franka_quantile requires q01/q99 statistics")
        if cfg.joint_task_names and not set(cfg.joint_task_names.split(',')).issubset(set(action_normalization_stats.get("tasks", []))):
            raise ValueError("Joint normalization statistics do not cover all selected tasks")
        if cfg.action_normalization == "franka_mulaw":
            if not np.isfinite(cfg.action_mulaw_mu) or cfg.action_mulaw_mu <= 0:
                raise ValueError("franka_mulaw requires a finite positive action_mulaw_mu")
            action_normalization_stats = {
                **action_normalization_stats,
                "companding": {"type": "mu_law", "mu": cfg.action_mulaw_mu, "arm_dimensions": 6},
                "decode_order": "clip_codec_output_then_inverse_mulaw_then_multiply_arm_scale",
            }
        print(f"[ActionNormalization] {cfg.action_normalization}: scale={action_arm_scale.tolist()}, mu={cfg.action_mulaw_mu}; state normalization unchanged")
    elif cfg.action_normalization == "droid_gain":
        from train.franka_action_normalization import droid_gain_stats
        action_normalization_stats = droid_gain_stats(
            np.maximum(np.abs(DROID_Q01[:6]), np.abs(DROID_Q99[:6])), cfg.action_gain
        )
        action_arm_scale = np.asarray(action_normalization_stats["arm_scale"], dtype=np.float32)
        print(f"[ActionNormalization] DROID arm gain={cfg.action_gain}, effective scale={action_arm_scale.tolist()}; gripper/state unchanged")
    elif cfg.action_normalization != "droid":
        raise ValueError(f"Unknown action_normalization={cfg.action_normalization}")
    if cfg.action_normalization != "droid_gain" and cfg.action_gain != 1.0:
        raise ValueError("action_gain is only valid for action_normalization=droid_gain")
    if cfg.action_normalization != "franka_mulaw" and cfg.action_mulaw_mu != 0.0:
        raise ValueError("action_mulaw_mu is only valid for action_normalization=franka_mulaw")
    resume_dir = Path(cfg.resume_path).expanduser().resolve() if cfg.resume else None
    if resume_dir is not None:
        previous = json.loads((resume_dir / "training_config.json").read_text())
        if previous.get("action_normalization", "droid") != cfg.action_normalization:
            raise ValueError("Cannot resume with different action normalization; initialize a fresh run instead")
        if action_normalization_stats is not None and previous.get("action_normalization_stats") != action_normalization_stats:
            raise ValueError("Resume action normalization statistics differ from checkpoint")
    init_dir = Path(cfg.init_checkpoint_path).expanduser().resolve() if cfg.init_checkpoint_path else None
    if init_dir is not None and not init_dir.is_dir():
        raise FileNotFoundError(f"Initialization checkpoint does not exist: {init_dir}")
    if resume_dir is not None and not resume_dir.is_dir():
        raise FileNotFoundError(f"Resume checkpoint does not exist: {resume_dir}")
    if init_dir is not None:
        droid_config_path = init_dir / "training_config.json"
        if not droid_config_path.is_file():
            raise FileNotFoundError(f"Missing DROID training config: {droid_config_path}")
        droid_config = json.loads(droid_config_path.read_text(encoding="utf-8"))
        expected_droid = {
            "base_model": "qwen3vl2b",
            "state_protocol": "droid_eef_axis_angle6_gripper_open1_singleturn_v1",
            "output_protocol": "future_state_then_action_v1",
            "action_prediction_mode": "block_ar_16",
            "action_embodiment_id": 2,
            "action_horizon": 15,
            "action_representation": ACTION_REPRESENTATION,
        }
        mismatches = {
            key: (droid_config.get(key), expected)
            for key, expected in expected_droid.items()
            if droid_config.get(key) != expected
        }
        if mismatches:
            raise ValueError(f"Incompatible DROID initialization checkpoint: {mismatches}")
        module_candidates = {
            "state encoder": ("state_encoder.pth", "franka_state_encoder.pth"),
            "future-state predictor": (
                "future_state_predictor.pth", "franka_future_state_predictor.pth"
            ),
        }
        for label, filenames in module_candidates.items():
            if not any((init_dir / filename).is_file() for filename in filenames):
                raise FileNotFoundError(
                    f"Missing {label} in {init_dir}; expected one of {filenames}"
                )
    if cfg.standard_action_only and resume_dir is not None:
        resume_config_path = resume_dir / "training_config.json"
        if not resume_config_path.is_file():
            raise FileNotFoundError(f"Missing resume training config: {resume_config_path}")
        resume_config = json.loads(resume_config_path.read_text(encoding="utf-8"))
        if not bool(resume_config.get("standard_action_only", False)):
            raise ValueError("Standard SFT may only resume a standard_action_only checkpoint")
        if resume_config.get("prediction_order") != "action_only" or bool(
            resume_config.get("predict_future_state", False)
        ):
            raise ValueError("Resume checkpoint is not an action-only policy")
    initial_step = checkpoint_step(resume_dir) if resume_dir is not None else 0
    if initial_step >= cfg.max_steps:
        raise ValueError(
            f"Checkpoint is already at step {initial_step}, but max_steps={cfg.max_steps}; "
            "max_steps is the total target step and must be larger"
        )
    base_model = cfg.base_model.lower()
    if base_model not in {"qwen3vl2b", "qwen35_g05"}:
        raise ValueError(
            f"Unsupported base_model={cfg.base_model!r}; expected qwen3vl2b or qwen35_g05"
        )
    model_source_dir = resume_dir if resume_dir is not None else init_dir
    model_source = str(model_source_dir) if model_source_dir is not None else cfg.vla_path
    if base_model == "qwen35_g05":
        try:
            from transformers import AutoModelForMultimodalLM
        except ImportError as exc:
            raise RuntimeError(
                "base_model=qwen35_g05 requires Transformers with "
                "AutoModelForMultimodalLM/Qwen3.5 support"
            ) from exc
        processor = AutoProcessor.from_pretrained(
            model_source, trust_remote_code=True, local_files_only=True
        )
        if not getattr(processor, "chat_template", None):
            tokenizer_template = getattr(processor.tokenizer, "chat_template", None)
            if tokenizer_template:
                processor.chat_template = tokenizer_template
        if resume_dir is None:
            vla = load_g05_qwen35_backbone(
                cfg.vla_path, cfg.g05_checkpoint_path
            )
        else:
            vla = AutoModelForMultimodalLM.from_pretrained(
                model_source,
                dtype=torch.bfloat16,
                low_cpu_mem_usage=True,
                trust_remote_code=True,
                local_files_only=True,
            )
    else:
        processor = AutoProcessor.from_pretrained(
            model_source, trust_remote_code=True, local_files_only=True
        )
        vla = AutoModelForImageTextToText.from_pretrained(
            model_source, dtype=torch.bfloat16, low_cpu_mem_usage=True,
            trust_remote_code=True, local_files_only=True,
        )
    vla = vla.to(device)
    vla.gradient_checkpointing_enable(gradient_checkpointing_kwargs={"use_reentrant": False})

    codec = AutoModel.from_pretrained(codec_path, trust_remote_code=True, local_files_only=True)
    codec_buffers = restore_actioncodec_derived_buffers(codec)
    if distributed.is_main_process:
        print(
            "[ActionCodec] restored derived buffers: "
            f"freq={codec_buffers['_freq']}, duration={codec_buffers['_duration']}, "
            f"action_dim={codec_buffers['_action_dim']}"
        )
    action_tokens = [f"<|action_{index}|>" for index in range(int(codec.config.vq_codebook_size))]
    extra_tokens = ["<|action_start|>", "<|action_end|>"] + action_tokens + STATE_TOKENS
    if cfg.action_prediction_mode == "block_ar_16":
        extra_tokens += [BAR_BOS_TOKEN]
    if cfg.predict_future_state:
        extra_tokens += [FUTURE_STATE_START_TOKEN, FUTURE_STATE_END_TOKEN] + FUTURE_STATE_TOKENS
    if cfg.unsupervised_prefix_tokens:
        extra_tokens += LATENT_PREFIX_TOKENS
    first_new_token_id = len(processor.tokenizer)
    processor.tokenizer.add_tokens(extra_tokens)
    vla.resize_token_embeddings(len(processor.tokenizer))
    if base_model == "qwen35_g05" and resume_dir is None:
        initialize_new_token_rows(vla, first_new_token_id, len(processor.tokenizer))
    if model_source_dir is not None and restore_legacy_wrapped_embeddings(vla, model_source_dir):
        print(f"[Checkpoint] restored legacy wrapped token embeddings from {model_source_dir}")
    if cfg.reset_new_token_embeddings:
        if model_source_dir is not None:
            raise ValueError("reset_new_token_embeddings is only for a fresh base VLM initialization")
        initialize_new_token_rows(vla, first_new_token_id, len(processor.tokenizer))
        if distributed.is_main_process:
            print(f"[FreshRobotModules] randomized all newly added token rows {first_new_token_id}:{len(processor.tokenizer)}; no robot checkpoint loaded")
    if cfg.reset_action_token_embeddings:
        if cfg.resume:
            raise ValueError("Cannot reset action vocabulary while resuming a trained checkpoint")
        from train.actioncodec_token_reset import reset_action_rows
        reset_count = reset_action_rows(vla, processor.tokenizer.convert_tokens_to_ids(action_tokens), cfg.seed)
        if distributed.is_main_process:
            print(f"[NewActionCodec] reset {reset_count} action token embedding/output rows; model source={model_source}")
    action_tokenizer = ActionTokenizer(
        codec, processor.tokenizer.convert_tokens_to_ids(action_tokens), cfg.action_embodiment_id,
        arm_scale=action_arm_scale,
        mulaw_mu=cfg.action_mulaw_mu,
    )
    action_bos_token_id = (
        processor.tokenizer.convert_tokens_to_ids(BAR_BOS_TOKEN)
        if cfg.action_prediction_mode == "block_ar_16"
        else None
    )
    if cfg.action_prediction_mode == "block_ar_16":
        from train.droid_qwen_actioncodec_state_first_bar import (
            build_bar_attention_mask,
            build_bar_input_ids,
            compute_bar_loss,
            validate_bar_loss,
        )

    state_token_ids = processor.tokenizer.convert_tokens_to_ids(STATE_TOKENS)
    future_state_token_ids = (
        processor.tokenizer.convert_tokens_to_ids(FUTURE_STATE_TOKENS)
        if cfg.predict_future_state
        else []
    )
    vla_wrapper = VLAWithState(
        vla, state_token_ids, future_state_token_ids
    ).to(device=device, dtype=torch.bfloat16)
    def first_existing(directory: Optional[Path], filenames: Tuple[str, ...]) -> Optional[Path]:
        if directory is None:
            return None
        for filename in filenames:
            candidate = directory / filename
            if candidate.is_file():
                return candidate
        return directory / filenames[0]

    state_encoder_path = first_existing(
        model_source_dir, ("franka_state_encoder.pth", "state_encoder.pth")
    )
    if state_encoder_path is not None and state_encoder_path.is_file():
        vla_wrapper.state_encoder.load_state_dict(
            torch.load(state_encoder_path, map_location="cpu", weights_only=True)
        )
    elif model_source_dir is not None:
        raise FileNotFoundError(f"Missing state encoder checkpoint: {state_encoder_path}")
    future_predictor_path = first_existing(
        model_source_dir,
        ("franka_future_state_predictor.pth", "future_state_predictor.pth"),
    )
    if cfg.predict_future_state and future_predictor_path is not None:
        if future_predictor_path.is_file():
            vla_wrapper.future_state_predictor.load_state_dict(
                torch.load(future_predictor_path, map_location="cpu", weights_only=True)
            )
        elif distributed.is_main_process:
            print("[Resume] future-state predictor is new and will be initialized from scratch")
    vla_ddp = DDP(
        vla_wrapper, device_ids=[device], find_unused_parameters=False,
        gradient_as_bucket_view=True,
    )
    optimizer = AdamW(vla_ddp.parameters(), lr=cfg.learning_rate, weight_decay=0.01)
    min_lr_ratio = cfg.min_learning_rate / cfg.learning_rate

    def lr_multiplier(current_step: int) -> float:
        progress = min(1.0, max(0.0, current_step / max(1, cfg.max_steps)))
        cosine = 0.5 * (1.0 + math.cos(math.pi * progress))
        return min_lr_ratio + (1.0 - min_lr_ratio) * cosine

    scheduler = LambdaLR(optimizer, lr_lambda=lr_multiplier)
    if initial_step:
        position_scheduler(scheduler, initial_step)
    task_names = cfg.joint_task_names.split(',') if cfg.joint_task_names else []
    if task_names and (len(set(task_names)) != len(task_names) or any(not re.fullmatch(r'Task\d+', name) for name in task_names)):
        raise ValueError("Invalid/duplicate joint_task_names")
    roots = [Path(cfg.data_root_dir) / name for name in task_names] if task_names else [Path(cfg.data_root_dir)]
    from train.franka_trajectory_sampling import merge_task_datasets
    validation_parts = []
    def build_dataset(train):
        datasets = [FrankaTask0Dataset(
            root, processor, action_tokenizer, train, cfg.predict_future_state,
            cfg.seed, cfg.gripper_transition_weight if train else 1,
            cfg.validation_episodes, Path(cfg.state_stats_path), Path(cfg.tasks_path),
            cfg.image_size, cfg.use_image_aug if train else False,
            cfg.use_random_resized_crop if train else False,
            cfg.use_robust_image_aug if train else False,
            cfg.primary_only_random_resized_crop,
            train_all_episodes=cfg.train_all_episodes,
            trajectory_sampling=cfg.trajectory_sampling,
        ) for root in roots]
        summary = [(name.name, len(ds.episodes), len(ds.valid_starts)) for name, ds in zip(roots, datasets)]
        if not train:
            validation_parts.extend(zip((root.name for root in roots), datasets))
        result = merge_task_datasets(datasets)
        print(f"[DataMix] train={train}, natural windows with gripper reweighting; tasks/episodes/windows={summary}")
        return result
    train_dataset = build_dataset(True)
    val_dataset = build_dataset(False)
    collator = Collator(processor.tokenizer.pad_token_id)
    train_loader = DataLoader(
        train_dataset, batch_size=cfg.batch_size, collate_fn=collator,
        num_workers=cfg.dataloader_num_workers, drop_last=True,
    )
    val_loader = DataLoader(val_dataset, batch_size=1, collate_fn=collator, num_workers=0)
    val_loaders = [(name, DataLoader(ds, batch_size=1, collate_fn=collator, num_workers=0))
                   for name, ds in validation_parts] if task_names else [("single", val_loader)]

    model_label = "G05-Qwen3.5-2B" if base_model == "qwen35_g05" else "Qwen3VL2B"
    data_label = "Franka-Joint" if task_names else "Franka-Task0"
    run_name = f"{model_label}+{data_label}+lr{cfg.learning_rate}+{cfg.run_id_note}"
    run_dir = cfg.run_root_dir / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    if distributed.is_main_process:
        codec_name = list(codec.config.embodiment_config)[cfg.action_embodiment_id]
        print(f"[ActionCodec] path={codec_path}, embodiment={codec_name}, chunk=15x7")
        print(f"[BaseModel] type={base_model}, source={model_source}")
        if base_model == "qwen35_g05" and resume_dir is None:
            print(f"[BaseModel] G0.5 backbone={cfg.g05_checkpoint_path}")
        print(f"[Task0Data] root={cfg.data_root_dir}, natural episode/frame ratio")
        print(
            f"[Task0DataV4] cache={FRANKA_V4_CACHE_NAME}, "
            f"action={FRANKA_V4_ACTION_REPRESENTATION}, "
            f"rotation={FRANKA_V4_ROTATION_EXECUTION}"
        )
        print(f"[TaskWeighting] gripper_transition={cfg.gripper_transition_weight}x")
        print(
            f"[State] EEF(6)+gripper_openness(1), "
            f"normalization q01/q99={cfg.state_stats_path}"
        )
        print(
            f"[FutureState] enabled={cfg.predict_future_state}, tokens=2, "
            f"CE_weight={cfg.future_state_loss_weight}, "
            f"regression_weight={cfg.future_state_regression_weight}"
        )
        print(
            f"[Protocol] prediction_order={cfg.prediction_order}, "
            f"standard_action_only={cfg.standard_action_only}, "
            f"unsupervised_prefix_tokens={cfg.unsupervised_prefix_tokens}, "
            f"supervised_action_tokens={action_tokenizer.num_tokens}"
        )
        print(
            f"[ActionPrediction] mode={cfg.action_prediction_mode}"
            + (
                ", block_size=16, forwards=3, input=[BAR_BOSx16,GT0:32]"
                if cfg.action_prediction_mode == "block_ar_16"
                else ", standard one-token causal shift"
            )
        )
        print(
            f"[LearningRate] peak={cfg.learning_rate:.8g}, "
            f"minimum={cfg.min_learning_rate:.8g}, cosine=true, "
            "warmup_steps=0"
        )
        print(
            f"[ImageAug] enabled={cfg.use_image_aug}, "
            f"random_resized_crop={cfg.use_random_resized_crop}, "
            f"primary_only_crop={cfg.primary_only_random_resized_crop}, "
            f"robust_color_noise={cfg.use_robust_image_aug}"
        )
        if resume_dir is not None:
            print(
                f"[Resume] checkpoint={resume_dir}, initial_step={initial_step}, "
                f"target_step={cfg.max_steps}, optimizer=fresh, lr={scheduler.get_last_lr()[0]:.8g}"
            )
        wandb.init(
            project=cfg.wandb_project,
            entity=cfg.wandb_entity or None,
            name=run_name,
        )

    optimizer.zero_grad(set_to_none=True)
    recent_losses = deque(maxlen=cfg.grad_accumulation_steps)
    step = initial_step
    micro_step = 0
    progress = tqdm.tqdm(
        total=cfg.max_steps, initial=initial_step, disable=not distributed.is_main_process
    )
    vla_ddp.train()
    while step < cfg.max_steps:
        for batch in train_loader:
            input_ids = batch["input_ids"].to(device)
            action_mask = batch["action_token_mask"].to(device)
            if cfg.action_prediction_mode == "block_ar_16":
                model_input_ids = build_bar_input_ids(
                    input_ids, action_mask, action_bos_token_id
                )
                model_attention_mask = build_bar_attention_mask(
                    batch["attention_mask"].to(device), action_mask
                )
            else:
                model_input_ids = input_ids
                model_attention_mask = batch["attention_mask"].to(device)
            with torch.autocast("cuda", dtype=torch.bfloat16):
                output = vla_ddp(
                    input_ids=model_input_ids,
                    states=batch["states"].to(device),
                    attention_mask=model_attention_mask,
                    pixel_values=batch["pixel_values"].to(device, dtype=torch.bfloat16),
                    image_grid_thw=batch["image_grid_thw"].to(device),
                )
                (
                    loss,
                    text_loss,
                    action_loss,
                    future_state_loss,
                    future_state_regression_loss,
                    accuracy,
                    future_state_accuracy,
                ) = (
                    compute_bar_loss
                    if cfg.action_prediction_mode == "block_ar_16"
                    else compute_loss
                )(
                    output.logits,
                    batch["labels"].to(device),
                    action_mask,
                    batch["future_state_token_mask"].to(device),
                    batch["future_states"].to(device),
                    output.get("predicted_future_states"),
                    cfg.text_loss_weight,
                    cfg.action_loss_weight,
                    cfg.future_state_loss_weight,
                    cfg.future_state_regression_weight,
                )
            (loss / cfg.grad_accumulation_steps).backward()
            recent_losses.append(loss.item())
            micro_step += 1
            if micro_step % cfg.grad_accumulation_steps:
                continue
            torch.nn.utils.clip_grad_norm_(vla_ddp.parameters(), 1.0)
            optimizer.step()
            scheduler.step()
            optimizer.zero_grad(set_to_none=True)
            step += 1
            progress.update()
            if distributed.is_main_process:
                smooth_loss = sum(recent_losses) / len(recent_losses)
                progress.set_postfix(loss=f"{smooth_loss:.3f}", acc=f"{accuracy.item():.3f}")
                if step % 10 == 0:
                    log_metrics = {
                        "train_loss": smooth_loss,
                        "text_loss": text_loss.item(),
                        "action_loss": action_loss.item(),
                        "action_token_accuracy": accuracy.item(),
                        "images/main_wrist": wandb.Image(
                            batch["log_images"][0].permute(1, 2, 0).cpu().numpy()
                        ),
                    }
                    if cfg.predict_future_state:
                        log_metrics.update({
                            "future_state_token_loss": future_state_loss.item(),
                            "future_state_regression_loss": future_state_regression_loss.item(),
                            "future_state_token_accuracy": future_state_accuracy.item(),
                        })
                    wandb.log(log_metrics, step=step)

            if cfg.save_steps > 0 and step % cfg.save_steps == 0:
                barrier()
                if distributed.is_main_process:
                    output_dir = run_dir / f"save_{step}_chkpt"
                    if output_dir.exists():
                        if (output_dir / "_SUCCESS").is_file():
                            raise FileExistsError(
                                f"Refusing to overwrite complete checkpoint: {output_dir}"
                            )
                        print(f"[Checkpoint] removing incomplete checkpoint: {output_dir}")
                        shutil.rmtree(output_dir)
                    output_dir.mkdir(parents=True, exist_ok=False)
                    processor.save_pretrained(output_dir)
                    save_unwrapped_vla(vla_ddp.module.vla, output_dir)
                    torch.save(
                        vla_ddp.module.state_encoder.state_dict(),
                        output_dir / "franka_state_encoder.pth",
                    )
                    if vla_ddp.module.future_state_predictor is not None:
                        torch.save(
                            vla_ddp.module.future_state_predictor.state_dict(),
                            output_dir / "franka_future_state_predictor.pth",
                        )
                    (output_dir / "trainer_state.json").write_text(
                        json.dumps({"global_step": step}, indent=2), encoding="utf-8"
                    )
                    (output_dir / "training_config.json").write_text(
                        json.dumps({
                            **asdict(cfg),
                            "action_normalization_stats": action_normalization_stats,
                            "action_arm_scale": action_tokenizer.arm_scale.tolist(),
                            "franka_cache_version": FRANKA_CACHE_VERSION,
                            "run_root_dir": str(cfg.run_root_dir),
                            "state_protocol": "droid_eef_axis_angle6_gripper_open1_singleturn_v1",
                            "output_protocol": "future_state_then_action_v1",
                            "action_horizon": 15,
                            "source_frequency_hz": 30,
                            "action_frequency_hz": 15,
                            "action_representation": ACTION_REPRESENTATION,
                            "source_action_conversion": (
                                "compose_two_30hz_base_frame_target_deltas_then_divide_"
                                "linear_0.075_rotation_0.15_v1"
                            ),
                        }, indent=2),
                        encoding="utf-8",
                    )
                    if action_normalization_stats is not None:
                        (output_dir / "action_norm_stats.json").write_text(
                            json.dumps(action_normalization_stats, indent=2), encoding="utf-8"
                        )
                    (output_dir / "checkpoint_complete.json").write_text(
                        json.dumps(
                            {
                                "global_step": step,
                                "checkpoint_kind": "persistent",
                                "prediction_order": cfg.prediction_order,
                                "standard_action_only": cfg.standard_action_only,
                                "batch_size": cfg.batch_size,
                                "grad_accumulation_steps": cfg.grad_accumulation_steps,
                            },
                            indent=2,
                        ),
                        encoding="utf-8",
                    )
                    (output_dir / "_SUCCESS").write_text(
                        f"global_step={step}\ncheckpoint_kind=persistent\n",
                        encoding="utf-8",
                    )
                    print(f"[Checkpoint] type=persistent, step={step}, path={output_dir}")
                barrier()

            if cfg.val_steps > 0 and step % cfg.val_steps == 0:
                barrier()
                if distributed.is_main_process:
                    per_task_metrics = {}
                    for task_name, loader in val_loaders:
                        per_task_metrics[task_name] = (
                            validate_bar_loss(vla_ddp.module, loader, device, cfg, action_bos_token_id)
                            if cfg.action_prediction_mode == "block_ar_16"
                            else validate_loss(vla_ddp.module, loader, device, cfg)
                        )
                    metrics = {key: float(np.mean([m[key] for m in per_task_metrics.values()]))
                               for key in next(iter(per_task_metrics.values()))}
                    if task_names:
                        metrics.update({f"{name}/{key}": value for name, values in per_task_metrics.items()
                                        for key, value in values.items()})
                    print(f"[Validation] step={step}: {metrics}")
                    wandb.log(metrics, step=step)
                barrier()
            if step >= cfg.max_steps:
                break
    progress.close()


if __name__ == "__main__":
    finetune()
