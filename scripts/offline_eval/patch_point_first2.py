"""Point-first deploy fix: the model forward needs the future-state token present, so append the rest of
the assistant template after the generated prefix; causal attention keeps it from affecting the logits read."""
from pathlib import Path
path = Path.home() / "offline_eval/deploy_franka_task0_qwen_actioncodec_eef.py"
s = path.read_text()
pairs = [
    ("        prefix = encoded.input_ids[:, :start].to(self.device)\n",
     "        prefix = encoded.input_ids[:, :start].to(self.device)\n"
     "        # The forward needs the future-state token, so the template tail follows the generated text;\n"
     "        # under causal attention it cannot change the logits at the last generated position.\n"
     "        tail = encoded.input_ids[:, start:].to(self.device)\n"),
    ("            ones = torch.ones_like(prefix)\n",
     "            current = torch.cat((prefix, tail), dim=1)\n"
     "            ones = torch.ones_like(current)\n"),
    ("                logits = self.model(input_ids=prefix, states=state.to(self.device),\n",
     "                logits = self.model(input_ids=current, states=state.to(self.device),\n"),
    ("            token = logits[:, -1].argmax(dim=-1, keepdim=True)\n",
     "            token = logits[:, prefix.shape[1] - 1].argmax(dim=-1, keepdim=True)\n"),
]
for old, new in pairs:
    if new in s:
        continue
    assert s.count(old) == 1, old
    s = s.replace(old, new)
path.write_text(s)
print("patched")
