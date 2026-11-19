"""Passively save received frames and telemetry; never select/start/seek a source."""
from pathlib import Path
import argparse,concurrent.futures,json,socket,sys,time,shutil
import cv2
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from video_transport import send_message,read_message
from frame_source_clock import SourceClock

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ip',default='192.168.201.2');p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not 1<=a.seconds<=120:p.error('seconds must be 1..120')
    a.output.mkdir(parents=True,exist_ok=False)
    if shutil.disk_usage(a.output).free<3*1024**3:raise RuntimeError('At least 3GiB free space required')
    def status():
        with socket.create_connection((a.ip,8558),timeout=4) as s:
            s.settimeout(4);send_message(s,dict(op='status'))
            with s.makefile('rb') as f:r,_=read_message(f)
            return r
    before=status();(a.output/'status_before.json').write_text(json.dumps(before,indent=2))
    deadline=time.monotonic()+a.seconds+15
    def telemetry():
        rows=[];error=None
        try:
            with socket.create_connection((a.ip,8557),timeout=4) as s:
                s.settimeout(1);s.sendall(b'HELLO\n');pending=b''
                with (a.output/'telemetry.jsonl').open('w') as f:
                    while time.monotonic()<deadline:
                        try:data=s.recv(1048576)
                        except TimeoutError:continue
                        if not data:break
                        pending+=data
                        while b'\n' in pending:
                            line,pending=pending.split(b'\n',1);r=json.loads(line)
                            if 'frame_seq' in r:
                                rows.append(r);f.write(json.dumps(r)+'\n');f.flush()
        except Exception as exc:error=str(exc)
        return dict(rows=rows,error=error)
    def stream(key,port,path):
        folder=a.output/key;folder.mkdir();rows=[];error=None;clock=SourceClock()
        cap=cv2.VideoCapture(f'rtsp://{a.ip}:{port}/{path}',cv2.CAP_FFMPEG,
            [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,15000,cv2.CAP_PROP_READ_TIMEOUT_MSEC,2500])
        try:
            if not cap.isOpened():raise RuntimeError('RTSP open failed')
            with (folder/'index.jsonl').open('w') as f:
                while time.monotonic()<deadline:
                    ok,img=cap.read()
                    if not ok:raise RuntimeError('RTSP read failed')
                    i=len(rows);name=f'{i:06d}.png';received=time.time()
                    decoded=None if key=='rgb' else clock.read(img,12 if key=='result' else 10)
                    r=dict(index=i,path=f'{key}/{name}',received_wall_s=received,
                        source_time_s=decoded[0] if decoded else None,clock_error=decoded[1] if decoded else None)
                    if not cv2.imwrite(str(folder/name),img,[cv2.IMWRITE_PNG_COMPRESSION,1]):raise OSError('PNG write failed')
                    rows.append(r);f.write(json.dumps(r)+'\n');f.flush()
        except Exception as exc:error=str(exc)
        finally:cap.release()
        return dict(frames=rows,error=error)
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        tf=pool.submit(telemetry)
        futures={k:pool.submit(stream,k,port,path) for k,port,path in
            [('rgb',8555,'rgb'),('result',8554,'yolopv2'),('top',8553,'bev')]}
        streams={k:f.result() for k,f in futures.items()};tel=tf.result()
    by_time={}
    for r in tel['rows']:
        t=r.get('source_media_time_s')
        if t is not None:by_time.setdefault(round(t,1),{})[(r.get('source_epoch'),r.get('frame_seq'))]=r
    matched=[]
    for key,s in streams.items():
        for frame in s['frames']:
            candidates=list(by_time.get(frame['source_time_s'],{}).values())
            if len(candidates)==1:
                r=candidates[0]
                matched.append(dict(**frame,stream=key,frame_seq=r['frame_seq'],source_epoch=r.get('source_epoch'),
                    left_state=r.get('left_boundary_state'),right_state=r.get('right_boundary_state'),
                    diagnostics=r.get('boundary_detection_diagnostics'),match='unique_burned_source_time'))
    (a.output/'matched.json').write_text(json.dumps(matched,indent=2))
    after=status();(a.output/'status_after.json').write_text(json.dumps(after,indent=2))
    report=dict(streams={k:dict(frames=len(v['frames']),error=v['error']) for k,v in streams.items()},
        telemetry_frames=len(tel['rows']),telemetry_error=tel['error'],matched_frames=len(matched),
        matched_inferred=sum('inferred' in (r['left_state'],r['right_state']) for r in matched),
        notes=['PNG preserves the received decoded frame, not an uncompressed camera sensor original.',
            'RGB has no burned source clock and is not paired by arrival order.',
            'No raw NN masks are present in these streams.',
            'Source time OCR matches require visual verification; unmatched frames are retained.'])
    (a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)

if __name__=='__main__':main()
