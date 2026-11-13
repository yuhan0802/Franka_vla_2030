import json, sys, numpy as np
np.set_printoptions(precision=1, suppress=True)
R = json.load(open(sys.argv[1]))
P = np.array([r['pred'] for r in R]) * 1000; G = np.array([r['gt'] for r in R]) * 1000
def phase(r):
    c = r['close_idx']
    if c < 0: return 'noclose'
    if r['t'] < c - 30: return 'approach'
    if r['t'] < c: return 'pre_grasp(<1s)'
    return 'after_grasp'
ph = np.array([phase(r) for r in R])
moving = np.linalg.norm(G, axis=1) > 10
print(f"N={len(R)}  moving(>10mm/1s)={moving.sum()}  latency={np.mean([r['lat'] for r in R]):.2f}s")
for name in ['ALL', 'approach', 'pre_grasp(<1s)', 'after_grasp']:
    m = moving if name == 'ALL' else (moving & (ph == name))
    if m.sum() == 0: continue
    e = P[m] - G[m]
    cos = (P[m] * G[m]).sum(1) / (np.linalg.norm(P[m], axis=1) * np.linalg.norm(G[m], axis=1) + 1e-9)
    ratio = np.linalg.norm(P[m], axis=1) / np.linalg.norm(G[m], axis=1)
    print(f"[{name}] n={m.sum()}  |err| mean {np.linalg.norm(e,axis=1).mean():.1f}mm median {np.median(np.linalg.norm(e,axis=1)):.1f}  "
          f"bias xyz {e.mean(0)}  |gt| {np.linalg.norm(G[m],axis=1).mean():.1f}mm  cos med {np.median(cos):.2f}  speed ratio med {np.median(ratio):.2f}")
# per-episode approach: cumulative predicted vs gt displacement summed over approach windows (rough endpoint drift)
print("\nper-episode (approach+pre_grasp, moving windows): bias xyz mm, cos")
for ep in sorted(set(r['ep'] for r in R)):
    m = np.array([r['ep'] == ep for r in R]) & moving & np.isin(ph, ['approach', 'pre_grasp(<1s)'])
    if m.sum() == 0: continue
    e = P[m] - G[m]; cos = (P[m]*G[m]).sum(1)/(np.linalg.norm(P[m],axis=1)*np.linalg.norm(G[m],axis=1)+1e-9)
    print(f"  {ep}: n={m.sum()} bias {e.mean(0)} cos {np.median(cos):.2f}")
# gripper timing: windows where gt closes within chunk
print("\ngripper: windows whose GT closes inside the 1 s chunk")
hit = []
for r in R:
    c = r['close_idx']
    if c >= 0 and r['t'] <= c < r['t'] + 30:
        gt_step = (c - r['t']) // 2; hit.append((gt_step, r['pred_close']))
for g, p in hit: print(f"  gt close step {g:2d}  pred close step {p:2d}")
