"""Read-only device telemetry measurement; never starts or controls the app."""
import argparse
from collections import Counter
import json
from pathlib import Path
import socket
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device-ip', required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    frames, received = [], []
    with socket.create_connection((args.device_ip,8557),timeout=10) as connection:
        connection.settimeout(10)
        connection.sendall(b'HELLO\n')
        with connection.makefile('rb') as stream, (args.output/'frames.jsonl').open('w') as out:
            deadline=time.monotonic()+args.seconds
            while time.monotonic()<deadline:
                line=stream.readline()
                if not line:
                    raise RuntimeError('Device disconnected before measurement completed')
                frame=json.loads(line)
                if 'frame_seq' not in frame:
                    continue
                received.append(time.monotonic())
                frames.append(frame)
                out.write(json.dumps(frame)+'\n')
    if len(frames)<2 or len({f['session'] for f in frames})!=1:
        raise RuntimeError('Need at least two frames from a single running app session')
    elapsed=frames[-1]['sent_monotonic_s']-frames[0]['sent_monotonic_s']
    moves=[]
    rates=[]
    for a,b in zip(frames,frames[1:]):
        move=max(abs(y-x) for side in ('left_boundary_dev','right_boundary_dev')
                 for x,y in zip(a[side],b[side]))
        dt=b['nn_timestamp_s']-a['nn_timestamp_s']
        moves.append(move)
        if dt>0:
            rates.append(move/dt)
    report=dict(frames=len(frames),device_elapsed_s=elapsed,
                processed_output_fps=(len(frames)-1)/elapsed,
                received_fps=(len(frames)-1)/(received[-1]-received[0]),
                source_sequence_gaps=sum(max(0,b['frame_seq']-a['frame_seq']-1)
                                         for a,b in zip(frames,frames[1:])),
                states=dict(Counter(f['boundary_state'] for f in frames)),
                references=dict(Counter(f.get('boundary_reference_side','unknown') for f in frames)),
                invalid_pairs=sum(not f['boundary_pair_valid'] for f in frames),
                centerline_valid_count=sum(bool(f['centerline_valid']) for f in frames),
                width_settings=sorted({f['corridor_width_m'] for f in frames}),
                left_anchors=sorted({f['left_boundary_coefficients'][0] for f in frames}),
                right_anchors=sorted({f['right_boundary_coefficients'][0] for f in frames}),
                max_normal_width_error=max(f['normal_width_error_dev'] for f in frames),
                max_frame_lateral_step=max(moves),
                max_lateral_rate=max(rates) if rates else None)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)


if __name__=='__main__':
    main()
