"""Control for hidden_probe.py: same ridge probe on stock Qwen3-VL-2B-Instruct image-token features.

Stock Qwen can point at the tube cap (~10 px at 384x384). If the ridge probe on its features is
also poor, the probe (45 scenes, linear) is too weak to read location and says little about the VLA.
Writes probe_stock{size}.npz in hidden_probe.py format (img* keys only).
"""
import sys

import numpy as np
import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

sys.path.insert(0, "/home/xmu/offline_eval")
from eef_eval_common import DATA_ROOT, Episode  # noqa: E402

MODEL = "/mnt/82_store/lyh/Qwen_weights/Qwen3-VL-2B"
LAYERS = (8, 14, 20, 28)
FRACTIONS = (0.0, 0.15, 0.3, 0.45)
size = int(sys.argv[1])
processor = AutoProcessor.from_pretrained(MODEL)
model = AutoModelForImageTextToText.from_pretrained(MODEL, dtype=torch.bfloat16, device_map="cuda").eval()
pad = processor.tokenizer.convert_tokens_to_ids("<|image_pad|>")
names = sorted(p.name for p in (DATA_ROOT / "Task0").iterdir() if p.name.startswith("TS_PB"))
rows, meta = {f"img{l}": [] for l in LAYERS}, []
for name in names:
    ep = Episode(name)
    for f in FRACTIONS:
        t = int(ep.close_index * f)
        image = Image.fromarray(ep.left[t]).resize((size, size), Image.Resampling.BILINEAR)
        messages = [{"role": "user", "content": [{"type": "image", "image": image},
                                                 {"type": "text", "text": "Where is the red cap of the test tube?"}]}]
        inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                               return_dict=True, return_tensors="pt").to("cuda")
        with torch.inference_mode():
            out = model(**inputs, output_hidden_states=True)
        is_pad = (inputs["input_ids"][0] == pad).cpu().numpy()
        for l in LAYERS:
            rows[f"img{l}"].append(out.hidden_states[l][0][is_pad].float().cpu().numpy().reshape(-1).astype(np.float16))
        meta.append((name, f, *ep.grasp_position[:2]))
    print(name, int(is_pad.sum()), flush=True)
np.savez(f"probe_stock{size}.npz", names=np.array([m[0] for m in meta]), frac=np.array([m[1] for m in meta]),
         grasp=np.array([m[2:4] for m in meta]), **{k: np.stack(v) for k, v in rows.items()})
