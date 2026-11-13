import json,glob,os,math
L='/media/mac/Elements2/franka_vr_docker/logs'
n=lambda v: math.sqrt(sum(x*x for x in v))
runs=sorted(glob.glob(L+'/eef_task1_*_20261030_*'))
for d in runs:
    p=d+'/execute/policy.jsonl'
    if not os.path.exists(p): print(os.path.basename(d),'NO EXECUTE'); continue
    R=[json.loads(l) for l in open(p)]
    st=[r for r in R if r['event']=='start'][0]['args']
    acts=[r for r in R if r['event']=='action']
    chunks=[r for r in R if r['event']=='chunk']
    faults=[r['reason'] for r in R if r['event']=='fault']
    lat=[c['policy_latency'] for c in chunks]
    print(f"== {os.path.basename(d)} H={st['execute_horizon']} gain={st['action_gain']} chunks={len(chunks)} acts={len(acts)} lat={sum(lat)/max(1,len(lat)):.2f}s")
    if acts:
        dp=[n(a['delta_position']) for a in acts]; dr=[n(a['delta_rotvec']) for a in acts]
        tc=sum(bool(a.get('translation_clipped')) for a in acts); rc=sum(bool(a.get('rotation_clipped')) for a in acts)
        A=[a['action'] for a in acts]
        amax=[round(max(abs(x[i]) for x in A),4) for i in range(6)]
        g=[(i,a.get('gripper_request')) for i,a in enumerate(acts) if a.get('gripper_request') not in (None,'none','',False)]
        pos=[a['measured_position'] for a in acts]
        print(f"  dpos/step mm mean {1000*sum(dp)/len(dp):.2f} max {1000*max(dp):.2f} | drot/step mean {sum(dr)/len(dr):.4f} max {max(dr):.4f} | clipT {tc} clipR {rc}")
        print('  action absmax', amax, 'grip range', round(min(x[6] for x in A),3), round(max(x[6] for x in A),3))
        print('  start', [round(x,3) for x in pos[0]], 'end', [round(x,3) for x in pos[-1]], 'minz', round(min(q[2] for q in pos),3))
        print('  gripper events', g[:6])
    print('  faults:', faults)
