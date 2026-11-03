#!/usr/bin/env python3
"""v5 preprocessing with deadband gripper labels (otherwise identical to v5).

v5's measured_gripper_actions flips to "close" on the 0.2-1 mm settling after the
gripper opens. Labels here come from gripper_labels_deadband (5 mm). The protocol /
action_representation strings are kept so the training loader accepts the cache;
the label method is recorded as gripper_label_method in the npz.
Usage: same as preprocess_franka_task_measured_eef_v5.py
"""
import os, sys
import numpy as np
sys.path.insert(0, "/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA/train")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import preprocess_franka_task_measured_eef_v5  # noqa: F401  (configures v4 as v5)
import preprocess_franka_task_measured_eef_v4 as impl
from gripper_labels_deadband import DEADBAND_M, deadband_labels

SPAN = impl.GRIPPER_OPEN_WIDTH - impl.GRIPPER_CLOSED_WIDTH
impl.measured_gripper_actions = lambda state_open: deadband_labels(np.asarray(state_open) * SPAN, deadband=DEADBAND_M)
impl.WINDOW_FILTER_SETTINGS["gripper_label_deadband_m"] = DEADBAND_M  # saved into the npz
_filter = impl.WINDOW_FILTER
impl.WINDOW_FILTER = lambda *a, **k: _filter(*a, **{x: y for x, y in k.items() if x != "gripper_label_deadband_m"})

if __name__ == "__main__":
    impl.main()
