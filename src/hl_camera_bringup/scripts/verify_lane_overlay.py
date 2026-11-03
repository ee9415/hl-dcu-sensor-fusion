"""Send one fixed video image repeatedly for preview-only threshold verification.

This is a deterministic mask/preview comparison, NEVER a throughput benchmark.
Restores camera/full/normal preview after testing.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import sys
import threading
import time

os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS','rtsp_transport;tcp')
import cv2

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from video_file_sender import select_camera,set_processing_mode,set_lane_overlay,fit_frame
from video_transport import send_message,read_message


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-ip',default='192.168.201.2')
    p.add_argument('--video',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--start',type=float,default=60)
    p.add_argument('--reference-views',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    deadline=time.monotonic()+45
    while True:
        try:
            with socket.create_connection((a.device_ip,8558),timeout=2):pass
            break
        except OSError:
            if time.monotonic()>deadline:raise
            time.sleep(1)
    cap=cv2.VideoCapture(str(a.video));cap.set(cv2.CAP_PROP_POS_MSEC,a.start*1000)
    ok,bgr=cap.read();cap.release()
    if not ok:raise RuntimeError('Cannot decode fixed road frame')
    image=fit_frame(bgr);ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,85])
    if not ok:raise RuntimeError('JPEG encoding failed')
    cv2.imwrite(str(a.output/'fixed_input.jpg'),image)
    stop=threading.Event();frames=[];errors=[];phases=[]
    def read():
        try:
            with socket.create_connection((a.device_ip,8557),timeout=5) as s:
                s.settimeout(5);s.sendall(b'HELLO\n')
                with s.makefile('rb') as f:
                    while not stop.is_set():
                        line=f.readline()
                        if not line:break
                        row=json.loads(line)
                        if row.get('input_source')=='pc':frames.append(row)
        except OSError as exc:
            if not stop.is_set():errors.append(str(exc))
    def send():
        try:
            with socket.create_connection((a.device_ip,8558),timeout=5) as s:
                s.settimeout(5)
                with s.makefile('rb') as f:
                    send_message(s,dict(op='select',source='pc'));read_message(f)
                    i=0;began=time.monotonic()
                    while not stop.is_set():
                        send_message(s,dict(op='frame',frame_index=i,media_time_s=a.start,
                                           media_epoch=0,file=a.video.name+'#fixed_frame'),jpeg.tobytes())
                        read_message(f);i+=1
                        stop.wait(max(0,began+i*.1-time.monotonic()))
        except OSError as exc:
            if not stop.is_set():errors.append(str(exc))
    def wait(condition):
        end=time.monotonic()+20
        while not condition() and time.monotonic()<end and not errors:time.sleep(.05)
        if not condition():raise RuntimeError(f'Timed out: {errors}')
    select_camera(a.device_ip);set_processing_mode(a.device_ip,'full')
    set_lane_overlay(a.device_ip,'normal',.5)
    reader=threading.Thread(target=read,daemon=True);sender=threading.Thread(target=send,daemon=True)
    reader.start();sender.start()
    report={}
    try:
        wait(lambda:len(frames)>=8)
        initial=frames[-1]
        modes=[('lane_mask',.2),('lane_mask',.8),('lane_score',.5)]
        if a.reference_views:
            modes += [('raw_rgb',.5),('lane_bands',.05),('lane_bands_rgb',.05),('lane_compare',.05)]
        modes.append(('normal',.5))
        for mode,threshold in modes:
            before=len(frames);ack=set_lane_overlay(a.device_ip,mode,threshold)
            assert ack['source_epoch']==initial['source_epoch']
            assert ack['processing_epoch']==initial['processing_epoch']
            def matching():
                return [r for r in frames[before:] if r.get('lane_overlay_mode')==mode and
                        r.get('lane_overlay_threshold')==threshold]
            wait(lambda:len(matching())>=6)
            rows=matching()
            assert all(r.get('lane_overlay_error') is None for r in rows)
            assert all(r.get('lane_perception_threshold')==.5 for r in rows)
            if mode not in ('normal','raw_rgb'):
                assert all(0<=r['lane_preview_score_min']<=r['lane_preview_score_max']<=1 for r in rows)
                assert all(r['lane_preview_selected_pixels'] is not None for r in rows)
            entry=dict(mode=mode,threshold=threshold,frames=len(rows),
                preview_selected_pixels=rows[-1].get('lane_preview_selected_pixels'),
                perception_pixels=rows[-1]['lane_perception_pixels'],
                score_min=rows[-1].get('lane_preview_score_min'),score_max=rows[-1].get('lane_preview_score_max'))
            stream=cv2.VideoCapture(f'rtsp://{a.device_ip}:8554/yolopv2',cv2.CAP_FFMPEG)
            ok=False
            drain_until=time.monotonic()+3
            # RTSP may initially yield frames from before the control change.
            # Drain to live video while the selected mode/input stay unchanged.
            for _ in range(90):
                ok,frame=stream.read()
                if not ok:break
                if time.monotonic()>=drain_until:break
            stream.release()
            entry['saved']=bool(ok and cv2.imwrite(str(a.output/f'{mode}_{threshold:.2f}.jpg'),frame))
            phases.append(entry)
        base_counts={r['lane_perception_pixels'] for r in frames}
        assert len(base_counts)==1,base_counts
        assert phases[0]['preview_selected_pixels']>=phases[1]['preview_selected_pixels']
        assert not errors
        report=dict(passed=True,fixed_image=True,not_fps_benchmark=True,frames=len(frames),
                    perception_pixels=sorted(base_counts),phases=phases,errors=errors)
    finally:
        stop.set();sender.join(timeout=6);reader.join(timeout=6)
        select_camera(a.device_ip);set_processing_mode(a.device_ip,'full')
        set_lane_overlay(a.device_ip,'normal',.5)
        (a.output/'report.json').write_text(json.dumps(report,indent=2))
        (a.output/'frames.jsonl').write_text('\n'.join(json.dumps(r) for r in frames))
    print(json.dumps(report))


if __name__=='__main__':main()
