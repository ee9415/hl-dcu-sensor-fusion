"""Verify PC->OAK->telemetry/RTSP, pause, and explicit camera recovery."""
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
from video_file_sender import VideoFileSender,select_camera
from frame_csv_logger import FrameCsvLogger


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-ip',required=True)
    p.add_argument('--video',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=15)
    p.add_argument('--start',type=float,default=0.)
    p.add_argument('--end',type=float)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    frames=[];stop=threading.Event();errors=[]
    def read():
        try:
            with socket.create_connection((args.device_ip,8557),timeout=5) as connection:
                # PC pause publishes nothing for ~2 s once no backlog is queued;
                # idle tolerance must exceed it or the reader silently stops.
                connection.settimeout(10);connection.sendall(b'HELLO\n')
                with connection.makefile('rb') as stream:
                    while not stop.is_set():
                        line=stream.readline()
                        if not line:break
                        payload=json.loads(line)
                        if 'frame_seq' in payload:frames.append(payload)
        except (OSError,ValueError) as exc:
            if not stop.is_set():errors.append(str(exc))
    select_camera(args.device_ip)
    reader=threading.Thread(target=read,daemon=True);reader.start()
    sender=VideoFileSender(args.device_ip,args.video,loop=True,start_s=args.start,end_s=args.end)
    logger=FrameCsvLogger(args.device_ip,args.output/'telemetry')
    def wait_for(condition,seconds=15):
        end=time.monotonic()+seconds
        while not condition() and time.monotonic()<end:time.sleep(.05)
        if not condition():raise RuntimeError(f'Condition timed out; sender={sender.snapshot()}, errors={errors}')
    try:
        wait_for(lambda:sum(f.get('input_source')=='camera' for f in frames)>=3)
        replay_begin=len(frames)
        sender.start()
        wait_for(lambda:any(f.get('input_source')=='pc' for f in frames[replay_begin:]))
        logger.start()
        time.sleep(args.seconds)
        sender.toggle_pause()
        wait_for(lambda:sender.snapshot()['state']=='일시정지')
        time.sleep(1)
        paused_count=len(frames);time.sleep(1)
        paused_no_camera=all(f.get('input_source')!='camera' for f in frames[paused_count:])
        sender.toggle_pause()
        previous=len(frames)
        wait_for(lambda:len(frames)>previous+5)
        logger.close();sender.close()
        select_camera(args.device_ip)
        after_switch=len(frames)
        wait_for(lambda:any(f.get('input_source')=='camera' for f in frames[after_switch:]))
        pc=[f for f in frames[replay_begin:] if f.get('input_source')=='pc']
        # Exclude the intentionally paused interval from the continuous FPS window.
        begin=pc[0]['nn_timestamp_s']
        continuous=[f for f in pc if f['nn_timestamp_s']-begin<args.seconds-1]
        input_clock_fps=((len(continuous)-1)/(continuous[-1]['nn_timestamp_s']-continuous[0]['nn_timestamp_s'])
             if len(continuous)>1 else 0.)
        # NN timestamps preserve capture time. Use output publication time for
        # throughput so a growing queue cannot make the measured FPS optimistic.
        fps=((len(continuous)-1)/(continuous[-1]['sent_monotonic_s']-continuous[0]['sent_monotonic_s'])
             if len(continuous)>1 else 0.)
        report=dict(pc_frames=len(pc),source_file=str(args.video),output_fps=fps,
            input_clock_fps=input_clock_fps,measurement_seconds=args.seconds,
            requested_start_s=args.start,requested_end_s=args.end,
            outside_range=sum(f['source_media_time_s']<args.start or
                              (args.end is not None and f['source_media_time_s']>=args.end) for f in pc),
            media_epochs=sorted(set(f.get('source_media_epoch') for f in pc)),
            sender=sender.snapshot(),source_states=dict(Counter(f.get('input_source') for f in frames)),
            depth_available_in_pc=sum(bool(f['depth_available']) for f in pc),
            depth_valid_objects_in_pc=sum(bool(o['depth_valid']) for f in pc for o in f['objects']),
            pc_sequence_mismatches=sum(f['frame_seq']!=f['nn_seq'] for f in pc),
            centerline_valid_in_pc=sum(bool(f['centerline_valid']) for f in pc),
            pause_never_fell_back_to_camera=paused_no_camera,
            camera_recovered=True,csv_frames=logger.frames,csv_path=str(logger.path),errors=errors)
        report['passed']=(len(pc)>=20 and not report['outside_range'] and not report['depth_available_in_pc'] and
            not report['depth_valid_objects_in_pc'] and not report['pc_sequence_mismatches'] and
            not report['centerline_valid_in_pc'] and paused_no_camera and logger.frames>0 and not errors)
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        (args.output/'frames.jsonl').write_text('\n'.join(json.dumps(f) for f in frames))
        print(json.dumps(report),flush=True)
        if not report['passed']:raise RuntimeError('Device replay verification failed')
    finally:
        logger.close();sender.close();stop.set();reader.join(timeout=3)
        select_camera(args.device_ip)


if __name__=='__main__':main()
