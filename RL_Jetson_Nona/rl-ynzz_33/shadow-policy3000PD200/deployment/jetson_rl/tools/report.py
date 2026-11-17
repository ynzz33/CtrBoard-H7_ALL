"""Export recorded policy/state CSV and diagnostic plots; does not control hardware.
Run on the development PC with mjlab_training/.venv/bin/python.
"""
import argparse
import csv
import json
from pathlib import Path
import struct
import zlib
import numpy as np

STATE_BYTES=224
CONTRACT=1767902

def read_records(path):
    with path.open('rb') as f:
        while True:
            h=f.read(16)
            if not h:return
            if len(h)!=16:
                yield 4,0,b'TRUNCATED_LOG: record header';return
            kind,n,us=struct.unpack('<IIQ',h)
            if n>4096:raise ValueError('invalid record size')
            data=f.read(n)
            if len(data)!=n:
                yield 4,us,b'TRUNCATED_LOG: record payload';return
            yield kind,us,data

def analyze(run,out):
    out.mkdir(parents=True,exist_ok=False)
    buffer=bytearray();states=[];policies=[];events=[];errors=0;gaps=0;last=None
    for kind,us,data in read_records(run/'records.bin'):
        if kind==4:events.append(dict(us=us,event=data.decode('utf-8',errors='replace')))
        elif kind==3:
            if len(data)!=640:raise ValueError('invalid inference record')
            reset,seq,infer,warmup=struct.unpack_from('<4I',data)
            v=np.frombuffer(data,dtype='<f4',offset=16).copy()
            policies.append([us,reset,seq,infer,warmup,*v.tolist()])
        elif kind==1:
            buffer.extend(data)
            while len(buffer)>=8:
                at=buffer.find(b'JRL2')
                if at<0:del buffer[:-3];break
                if at:del buffer[:at]
                if len(buffer)<8:break
                n=struct.unpack_from('<H',buffer,6)[0]+36
                if buffer[4]!=2 or n>384:del buffer[0];errors+=1;continue
                if len(buffer)<n:break
                frame=bytes(buffer[:n])
                if zlib.crc32(frame[:-4])!=struct.unpack_from('<I',frame,n-4)[0]:del buffer[0];errors+=1;continue
                del buffer[:n]
                if frame[5]!=2 or n!=STATE_BYTES+36:continue
                session,reset,seq,h7_us=struct.unpack_from('<QIIQ',frame,8)
                flags,fault,ack,warmup,applied,latency,contract,rejected=struct.unpack_from('<8I',frame,32)
                if contract!=CONTRACT:raise ValueError('unknown contract')
                key=(session,seq)
                if last and session==last[0]:
                    delta=(seq-last[1])&0xffffffff
                    if delta>1:gaps+=delta-1
                last=key
                v=np.frombuffer(frame,dtype='<f4',count=41,offset=64).copy()
                applied_us=struct.unpack_from('<Q',frame,228)[0]
                tx_drops,out_seq,out_policy,out_flags,out_valid=struct.unpack_from('<5I',frame,236)
                states.append([us,h7_us,session,reset,seq,flags,fault,ack,warmup,applied,latency,rejected,applied_us,tx_drops,out_seq,out_policy,out_flags,out_valid,*v.tolist()])
    state_names=['host_us','h7_sample_us','session','reset','sequence','flags','fault','ack','warmup_pd_steps','applied','sample_to_first_pd_us','rejects','applied_us','tx_drops','output_sequence','output_policy','output_flags','output_valid']
    for prefix,n in [('gyro',3),('gravity',3),('q',4),('dq',6),('command',3),('accepted_action',6),('target',6),('torque_request',6),('quaternion_wxyz',4)]:state_names += [f'{prefix}_{i}' for i in range(n)]
    policy_names=['host_us','reset','sequence','inference_us','warmup']+[f'obs_{i}' for i in range(25)]+[f'history_{i}' for i in range(125)]+[f'action_{i}' for i in range(6)]
    for name,headers,rows in [('state.csv',state_names,states),('policy.csv',policy_names,policies)]:
        with (out/name).open('w',newline='') as f:w=csv.writer(f);w.writerow(headers);w.writerows(rows)
    unique={}
    for r in states:
        if r[9]:unique[(r[2],r[3],r[9])]=r[10]
    latency=list(unique.values())
    summary=dict(state_frames=len(states),policy_frames=len(policies),crc_or_header_errors=errors,sequence_gaps=gaps,
        trailing_serial_bytes=len(buffer),first_pd_samples=len(latency),max_sample_to_first_pd_us=max(latency,default=None),
        deadline_violations=sum(t>=10000 for t in latency),events=events,
        note='100Hz state snapshots; torque_request is a requested torque, not measured torque. No motion acceptance inferred.')
    if (run/'summary.txt').exists():summary['writer_summary']=(run/'summary.txt').read_text()
    writer_ok=summary.get('writer_summary','').find('dropped=0\nwrite_failed=0\n')>=0
    summary['complete']=bool(writer_ok and not errors and not gaps and not buffer and not any(e['event'].startswith('TRUNCATED_LOG:') for e in events))
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    if states:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        s=np.array(states,dtype=np.float64);t=(s[:,0]-s[0,0])/1e6
        col={name:i for i,name in enumerate(state_names)}
        valid=(s[:,col['flags']].astype(np.uint32)&1)!=0
        output_valid=(s[:,col['output_valid']]!=0)&((s[:,col['output_flags']].astype(np.uint32)&1)!=0)
        s[~valid,18:34]=np.nan
        s[~output_valid,col['target_0']:col['torque_request_5']+1]=np.nan
        fig,ax=plt.subplots(6,1,figsize=(13,16),sharex=True)
        gx=s[:,col['gravity_0']];gy=s[:,col['gravity_1']];gz=s[:,col['gravity_2']]
        ax[0].plot(t,np.rad2deg(np.arctan2(gx,np.sqrt(gy*gy+gz*gz))),label='pitch from gravity')
        ax[0].plot(t,np.rad2deg(np.arctan2(-gy,-gz)),label='roll from gravity');ax[0].set_ylabel('deg')
        for i in range(4):
            ax[1].plot(t,s[:,col[f'q_{i}']],label=f'q{i}');ax[1].plot(t,s[:,col[f'target_{i}']],ls='--',label=f'target{i}')
        ax[1].set_ylabel('rad')
        for i in (4,5):
            ax[2].plot(t,s[:,col[f'dq_{i}']],label=f'wheel{i-4}');ax[2].plot(t,s[:,col[f'target_{i}']],ls='--',label=f'target{i-4}')
        ax[2].set_ylabel('rad/s')
        for i in range(6):ax[3].plot(t,s[:,col[f'accepted_action_{i}']],label=f'action{i}')
        ax[3].set_ylabel('action')
        ax[4].plot(t,s[:,col['sample_to_first_pd_us']]/1000,label='sample to first PD')
        if policies:
            p=np.array(policies);ax[4].plot((p[:,0]-s[0,0])/1e6,p[:,3]/1000,label='inference')
        ax[4].axhline(10,color='r',ls='--',label='10ms deadline');ax[4].set_ylabel('ms')
        for name in ('fault','rejects','tx_drops'):ax[5].plot(t,s[:,col[name]],label=name)
        ax[5].set_xlabel('host receive time (s)')
        for a in ax:a.legend(loc='upper right',ncol=4,fontsize=8);a.grid(True)
        fig.tight_layout();fig.savefig(out/'diagnostics.png',dpi=140);plt.close(fig)
    return summary
if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();print(json.dumps(analyze(a.run,a.out),ensure_ascii=False,indent=2))
