import numpy as np
from gripper_labels_deadband import deadband_labels
w = np.array([70]*5 + [60, 45, 45, 30, 15, 2, 0.2, 1.8] + [1.9]*20 + [15, 27, 38, 38, 38, 62, 71, 71, 70.2, 70.2] + [70.2]*5) / 1000
l = deadband_labels(w)
ch = [(int(i), int(l[i])) for i in np.flatnonzero(np.diff(l)) + 1]
print(ch); assert ch == [(4, 0), (32, 1)], ch   # close from frame 4 (last 70 before 60), open from 32 (last 1.9)
print("unit test ok")
