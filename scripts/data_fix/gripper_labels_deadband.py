"""Gripper open/close labels from measured width with a deadband.

v5 labels the measured t->t+1 width direction with a 1e-4 (0.007 mm) threshold, so
the 0.2-1 mm settling right after the gripper reaches full open flips the label
back to "close" (e.g. Task3 lid release). Here a direction change counts only after
the width has moved DEADBAND_M away from the last extreme; the label switches at
the last stationary frame before that movement (v5 semantics: label at t is the
direction of t->t+1).
"""
import numpy as np

DEADBAND_M = 0.005
STEP_M = 3e-4      # per-frame change that counts as movement
MAX_STALE = 4      # repeated samples allowed inside one movement (stale feedback)


def movement_onset(width, i, sign, lower):
    """Last stationary frame before the movement (direction sign) that reaches frame i."""
    k, flat = i, 0
    while k > lower:
        d = (width[k] - width[k - 1]) * sign
        if d > STEP_M:
            flat, k = 0, k - 1
        elif abs(d) <= STEP_M and flat < MAX_STALE:
            flat, k = flat + 1, k - 1
        else:
            break
    return k + flat


def deadband_labels(width, deadband=DEADBAND_M, initial_open=1.0):
    width = np.asarray(width, dtype=np.float64).reshape(-1)
    out = np.full(len(width), initial_open, dtype=np.float32)
    label, extreme, last_flip = initial_open, 0, 0
    for i in range(1, len(width)):
        sign = -1.0 if label == 1.0 else 1.0   # direction that would flip the label
        if (width[i] - width[extreme]) * sign <= 0:
            extreme = i                          # new max while open / new min while closed
        elif (width[i] - width[extreme]) * sign > deadband:
            label = 1.0 - label
            onset = movement_onset(width, i, sign, last_flip + 1)
            out[onset:] = label
            extreme, last_flip = i, onset
            continue
        out[i] = label
    return out
