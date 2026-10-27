"""Exercise basic -> full -> basic during one uninterrupted PC publisher."""
import argparse
from collections import Counter
import json
from pathlib import Path
import socket
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from video_file_sender import VideoFileSender,select_camera,set_processing_mode


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-ip',required=True);p.add_argument('--video',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    rows=[];errors=[];stop=threading.Event()
    def read():
        try:
            with socket.create_connection((args.device_ip,8557),timeout=10) as connection:
                connection.sendall(b'HELLO\n')
                with connection.makefile('rb') as stream:
                    while not stop.is_set():
                        line=stream.readline()
                        if not line:break
                        frame=json.loads(line)
                        if 'frame_seq' in frame:rows.append(frame)
        except (OSError,ValueError) as exc:
            if not stop.is_set():errors.append(str(exc))
    reader=threading.Thread(target=read,daemon=True)
    sender=VideoFileSender(args.device_ip,args.video,loop=True,fps=20,start_s=60,end_s=76)
    phases=[]
    def wait(predicate):
        end=time.monotonic()+20
        while not predicate() and time.monotonic()<end:time.sleep(.05)
        if not predicate():raise RuntimeError(f'Mode wait failed; {sender.snapshot()}, {errors}')
    try:
        set_processing_mode(args.device_ip,'basic');select_camera(args.device_ip);reader.start()
        wait(lambda:sum(f.get('input_source')=='camera' for f in rows)>=3)
        sender.start()
        for mode in ('basic','full','basic'):
            response=set_processing_mode(args.device_ip,mode);epoch=response['processing_epoch']
            wait(lambda:any(f.get('input_source')=='pc' and f.get('processing_epoch')==epoch for f in rows))
            time.sleep(8)
            phase=[f for f in rows if f.get('input_source')=='pc' and f.get('processing_epoch')==epoch]
            duration=phase[-1]['sent_monotonic_s']-phase[0]['sent_monotonic_s']
            latency=sorted((f['sent_monotonic_s']-f['capture_monotonic_est_s'])*1000 for f in phase)
            phases.append(dict(mode=mode,epoch=epoch,frames=len(phase),fps=(len(phase)-1)/duration,
                latency_p95_ms=latency[int((len(latency)-1)*.95)],
                boundary_states=dict(Counter(f['boundary_state'] for f in phase)),
                valid_boundaries=sum(bool(f['boundary_valid']) for f in phase),
                sequence_mismatches=sum(f['frame_seq']!=f['nn_seq'] for f in phase)))
        report=dict(phases=phases,sender=sender.snapshot(),errors=errors,
            passed=not errors and all(not p['sequence_mismatches'] for p in phases)
                and phases[0]['valid_boundaries']==0 and phases[2]['valid_boundaries']==0
                and phases[1]['valid_boundaries']>0)
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        (args.output/'frames.jsonl').write_text('\n'.join(json.dumps(f) for f in rows))
        print(json.dumps(report),flush=True)
        if not report['passed']:raise RuntimeError('Mode verification failed')
    finally:
        sender.close();select_camera(args.device_ip);set_processing_mode(args.device_ip,'basic')
        stop.set()
        if reader.is_alive():reader.join(timeout=3)


if __name__=='__main__':main()
