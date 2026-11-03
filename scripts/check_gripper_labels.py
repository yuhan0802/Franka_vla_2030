#!/usr/bin/env python3
"""Read-only check of the gripper command vs. the measured gripper width in raw Franka_vr episodes.

Flags frames where the recorded command (gripper_target_width, valid) says CLOSED (< 0.03 m) while the
gripper is physically fully OPEN (gripper_width > 0.06 m). Such frames are training labels that
contradict what the robot actually did.

Usage: check_gripper_labels.py TASK_DIR [--frames EPISODE:START:END:STEP]
  e.g. check_gripper_labels.py /mnt/82_store/vla/Franka_vr/Task3 --frames Task_3_001_cxc:540:930:30
"""
import argparse
import glob
import os

import numpy as np
import pandas as pd

CLOSED_CMD_M, OPEN_WIDTH_M = 0.03, 0.06
COLUMNS = ["timestamp", "gripper_width", "gripper_target_width", "gripper_target_width_valid", "action"]


def load(episode_dir):
    df = pd.read_parquet(os.path.join(episode_dir, "data/chunk-000/episode_000000.parquet"), columns=COLUMNS)
    col = lambda c: np.stack(df[c].values)[:, 0]
    return (df.timestamp.values, col("gripper_width"), col("gripper_target_width"),
            col("gripper_target_width_valid") > 0, np.stack(df.action.values)[:, 7])


parser = argparse.ArgumentParser()
parser.add_argument("task_dir")
parser.add_argument("--frames", help="EPISODE:START:END:STEP frame dump")
args = parser.parse_args()

if args.frames:
    name, start, end, step = args.frames.split(":")
    t, width, command, valid, action = load(os.path.join(args.task_dir, name))
    print("| frame | time (s) | gripper_width (m) | gripper_target_width | valid | action[7] |")
    print("|---|---|---|---|---|---|")
    for i in range(int(start), int(end), int(step)):
        print(f"| {i} | {t[i] - t[0]:.1f} | {width[i]:.3f} | {command[i]:.3f} | {int(valid[i])} | {action[i]:.3f} |")
    raise SystemExit

bad = 0
episodes = sorted(glob.glob(os.path.join(args.task_dir, "*/data/chunk-000/episode_000000.parquet")))
print("| episode | frames | fps | contradicting frames | longest run (s) |")
print("|---|---|---|---|---|")
for path in episodes:
    episode_dir = path.split("/data/chunk-000/")[0]
    t, width, command, valid, _ = load(episode_dir)
    fps = 1 / np.median(np.diff(t))
    mask = valid & (command < CLOSED_CMD_M) & (width > OPEN_WIDTH_M)
    run = best = 0
    for flag in mask:
        run = run + 1 if flag else 0
        best = max(best, run)
    bad += best / fps > 2
    print(f"| {os.path.basename(episode_dir)} | {len(t)} | {fps:.0f} | {int(mask.sum())} | {best / fps:.1f} |")
print(f"\n{bad}/{len(episodes)} episodes contain a contradiction longer than 2 s")
