"""FRANKA_POINT_FIRST=1: the assistant says "Target: (u, v)" before the action instead of after it.

The action tokens then attend to the model's own estimate of where the tube (or basket) is, so
localization is on the action's path rather than a side objective. Deploy detects the run by the
point_first marker the launcher writes next to points.json, generates the Target line greedily,
then generates the action as before.
"""
import shutil
from pathlib import Path

P = "/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA/"
TRAIN = P + "train/franka_task0_qwen_actioncodec_droid_pretrained_bar_stateaug.py"
LAUNCH = P + "train-scripts/run_franka_task0_actv2_continue_20.sh"
DEPLOY = str(Path.home() / "offline_eval/deploy_franka_task0_qwen_actioncodec_eef.py")
BACKUP = Path.home() / "claude_tube"


def patch(path, pairs, backup):
    s = open(path).read()
    if not (BACKUP / backup).exists():
        shutil.copy(path, BACKUP / backup)
    for old, new in pairs:
        if new in s:
            continue
        assert s.count(old) == 1, (path, old[:60])
        s = s.replace(old, new)
    open(path, "w").write(s)
    print("patched", path)


patch(TRAIN, [
    ("POINTS = json.load(open(os.environ[\"FRANKA_POINTS_JSON\"])) if os.environ.get(\"FRANKA_POINTS_JSON\") else None\n",
     "POINTS = json.load(open(os.environ[\"FRANKA_POINTS_JSON\"])) if os.environ.get(\"FRANKA_POINTS_JSON\") else None\n"
     "# FRANKA_POINT_FIRST=1 puts the Target line before the action, so the action conditions on the\n"
     "# model's own localization; deploy then generates the Target line first.\n"
     "POINT_FIRST = os.environ.get(\"FRANKA_POINT_FIRST\") == \"1\"\n"),
    ("        if sample.get(\"point_text\"):\n"
     "            assistant_content += \"\\n\" + sample[\"point_text\"]\n",
     "        if sample.get(\"point_text\") and POINT_FIRST:\n"
     "            assistant_content = sample[\"point_text\"] + \"\\n\" + assistant_content\n"
     "        elif sample.get(\"point_text\"):\n"
     "            assistant_content += \"\\n\" + sample[\"point_text\"]\n"),
], "stateaug_before_pointfirst.py")

patch(LAUNCH, [
    ("[[ -z \"${FRANKA_POINTS_JSON:-}\" ]] || { cp \"$FRANKA_POINTS_JSON\" \"$RUN_ROOT/points.json\"; export FRANKA_POINTS_JSON; echo \"Points: $FRANKA_POINTS_JSON\"; }\n",
     "[[ -z \"${FRANKA_POINTS_JSON:-}\" ]] || { cp \"$FRANKA_POINTS_JSON\" \"$RUN_ROOT/points.json\"; export FRANKA_POINTS_JSON; echo \"Points: $FRANKA_POINTS_JSON\"; }\n"
     "# Deploy reads this marker to generate the Target line before the action.\n"
     "[[ \"${FRANKA_POINT_FIRST:-0}\" != 1 ]] || { touch \"$RUN_ROOT/point_first\"; export FRANKA_POINT_FIRST; echo \"Point first\"; }\n"),
], "launcher_before_pointfirst.sh")

patch(DEPLOY, [
    ("        self.dummy_action = \"<|action_0|>\" * BAR_ACTION_TOKENS\n",
     "        self.dummy_action = \"<|action_0|>\" * BAR_ACTION_TOKENS\n"
     "        # Point-first runs (marker written by the launcher) say \"Target: (u, v)\" before the action.\n"
     "        self.point_first = (checkpoint.parent.parent / \"point_first\").exists()\n"
     "        self.last_point = \"\"\n"),
    ("    def _prompt(self, raw):\n",
     "    def _prompt(self, raw, point_text=\"\"):\n"),
    ("        assistant = (f\"Next state:\\n{FUTURE_STATE_START_TOKEN}",
     "        assistant = (f\"{point_text}Next state:\\n{FUTURE_STATE_START_TOKEN}"),
    ("    @torch.inference_mode()\n"
     "    def predict(self, raw) -> np.ndarray:\n"
     "        encoded, end, state, _ = self._prompt(raw)\n",
     "    @torch.inference_mode()\n"
     "    def _point(self, raw) -> str:\n"
     "        \"\"\"Greedy-generate the Target line that point-first runs emit before the action.\"\"\"\n"
     "        marker = \"<|im_start|>assistant\\n\"\n"
     "        encoded, _, state, _ = self._prompt(raw)\n"
     "        ids = encoded.input_ids[0].tolist()\n"
     "        marker_ids = self.processor.tokenizer.encode(marker, add_special_tokens=False)\n"
     "        start = next(i for i in range(len(ids) - len(marker_ids), -1, -1)\n"
     "                     if ids[i:i + len(marker_ids)] == marker_ids) + len(marker_ids)\n"
     "        prefix = encoded.input_ids[:, :start].to(self.device)\n"
     "        pixels = encoded.pixel_values.to(self.device, dtype=torch.bfloat16)\n"
     "        grid = encoded.image_grid_thw.to(self.device)\n"
     "        text = \"\"\n"
     "        for _ in range(20):\n"
     "            ones = torch.ones_like(prefix)\n"
     "            with torch.autocast(\"cuda\", dtype=torch.bfloat16, enabled=self.device.type == \"cuda\"):\n"
     "                logits = self.model(input_ids=prefix, states=state.to(self.device),\n"
     "                                    attention_mask=build_bar_attention_mask(ones, torch.zeros_like(ones, dtype=torch.bool)),\n"
     "                                    pixel_values=pixels, image_grid_thw=grid, use_cache=False).logits\n"
     "            token = logits[:, -1].argmax(dim=-1, keepdim=True)\n"
     "            prefix = torch.cat((prefix, token), dim=1)\n"
     "            text += self.processor.tokenizer.decode(token[0])\n"
     "            if \"\\n\" in text:\n"
     "                break\n"
     "        text = text.split(\"\\n\")[0]\n"
     "        if not text.startswith(\"Target: (\"):\n"
     "            raise RuntimeError(f\"point-first checkpoint generated {text!r} instead of a Target line\")\n"
     "        return text + \"\\n\"\n"
     "\n"
     "    @torch.inference_mode()\n"
     "    def predict(self, raw) -> np.ndarray:\n"
     "        self.last_point = self._point(raw) if self.point_first else \"\"\n"
     "        encoded, end, state, _ = self._prompt(raw, self.last_point)\n"),
    ("from train.droid_qwen_actioncodec_state_first_bar import BAR_ACTION_TOKENS, generate_action_codes_bar  # noqa: E402\n",
     "from train.droid_qwen_actioncodec_state_first_bar import (  # noqa: E402\n"
     "    BAR_ACTION_TOKENS, build_bar_attention_mask, generate_action_codes_bar)\n"),
], "deploy_before_pointfirst.py")
