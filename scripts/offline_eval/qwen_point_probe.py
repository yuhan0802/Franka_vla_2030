"""Can stock Qwen3-VL-2B-Instruct point at the tube cap? Compare with the colour detector, full res vs 256x256."""
import glob, json, re, sys

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

sys.path.insert(0, __import__("os").path.dirname(__file__))
from tube_targets import detect  # noqa: E402

MODEL = "/mnt/82_store/lyh/Qwen_weights/Qwen3-VL-2B"
ROOT = "/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0/"
PROMPT = ("Point to the red cap of the small test tube standing in the white rack. "
          "Answer with the point as JSON: {\"point_2d\": [x, y]}.")
processor = AutoProcessor.from_pretrained(MODEL)
model = AutoModelForImageTextToText.from_pretrained(MODEL, dtype=torch.bfloat16, device_map="cuda").eval()


def point(img_rgb):
    messages = [{"role": "user", "content": [{"type": "image", "image": Image.fromarray(img_rgb)},
                                             {"type": "text", "text": PROMPT}]}]
    inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                           return_dict=True, return_tensors="pt").to("cuda")
    with torch.no_grad():
        out = model.generate(**inputs, max_new_tokens=40, do_sample=False)
    text = processor.batch_decode(out[:, inputs["input_ids"].shape[1]:], skip_special_tokens=True)[0]
    nums = [float(v) for v in re.findall(r"-?\d+\.?\d*", text)[:2]]
    return text, nums


rows = []
for d in sorted(glob.glob(ROOT + "*/")):
    ok, bgr = cv2.VideoCapture(d + "videos/chunk-000/observation.images.left/episode_000000.mp4").read()
    cap, _ = detect(bgr)
    if cap is None:
        continue
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    row = dict(episode=d.split("/")[-2], cap=[float(cap[0]), float(cap[1])])
    for tag, img in (("full", rgb), ("r256", cv2.resize(rgb, (256, 256), interpolation=cv2.INTER_AREA))):
        text, nums = point(img)
        # Qwen3-VL grounds in 0-1000 relative coordinates; map back to the 640x480 frame.
        px = None if len(nums) < 2 else [nums[0] / 1000 * w, nums[1] / 1000 * h]
        row[tag] = dict(text=text, px=px, err=None if px is None else float(np.hypot(px[0] - cap[0], px[1] - cap[1])))
    rows.append(row)
    print(row["episode"], [round(v) for v in cap], "full", row["full"]["text"][:50], "| r256", row["r256"]["text"][:50], flush=True)
for tag in ("full", "r256"):
    errs = np.array([r[tag]["err"] if r[tag]["err"] is not None else 999 for r in rows])
    print(f"{tag}: median {np.median(errs):.1f} px, within 10 px {int((errs <= 10).sum())}/{len(errs)}, "
          f"within 25 px {int((errs <= 25).sum())}/{len(errs)}, unparsed {int((errs == 999).sum())}")
json.dump(rows, open("/home/xmu/claude_tube/qwen_point_probe.json", "w"), indent=1)
