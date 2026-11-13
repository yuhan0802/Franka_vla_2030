import json,glob,os
L='/media/mac/Elements2/franka_vr_docker/logs'
for r in ['eef_task1_joint8-newcodec-20k_20261030_152754_56208','eef_task1_task01-newcodec-10k_20261030_153212_64316']:
    R=[json.loads(l) for l in open(f'{L}/{r}/execute/policy.jsonl')]
    obs=0; ai=0
    for x in R:
        if x['event']=='observation': obs+=1
        if x['event']=='action':
            ai+=1
            if x.get('gripper_request') or x.get('gripper_suppressed'):
                print(r[10:40], 'act',ai,'obs#',obs-1,x.get('gripper_request'),x.get('gripper_suppressed'),'pos',[round(v,4) for v in x['measured_position']])
        if x['event'] in ('gripper_verified','fault','finish','gripper_result'): print('  ',x['event'],{k:v for k,v in x.items() if k not in('event','monotonic_s')})
    print('  last pos',[round(v,4) for v in [a for a in R if a['event']=='action'][-1]['measured_position']], 'events', sorted(set(x['event'] for x in R)))
