"""Measure real on-device contour tracking over camera input or recorded footage.

No mask injection or runtime monkey patching. Coordinates come from the deployed
telemetry. RTSP is captured independently; it is not claimed frame-synchronous.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import socket
import sys
import threading
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from oak_recorder import CaptureSource
from video_file_sender import (VideoFileSender, select_camera, set_processing_mode,
                               set_boundary_display, set_contour_display, set_lane_overlay)


TRACKED_MODELS = ('mask_contour_temporal','persistent_skeleton')


def analyze(frames):
    transitions, movements, violations = [], [], []
    retained_ages = []
    curve_updates = []
    for frame in frames:
        diagnostics = frame.get('boundary_detection_diagnostics') or [{},{}]
        left = diagnostics[0]
        if len(frame.get('left_boundary_image',[])) >= 8 and left.get('retained'):
            if left.get('missing_s') is not None:
                retained_ages.append(left['missing_s'])
            if left.get('opposite_curve_applied_px',0.) > 1e-6:
                curve_updates.append(dict(media_time_s=frame.get('source_media_time_s'),
                    missing_s=left.get('missing_s'),movement_px=left['opposite_curve_applied_px']))
    for previous, current in zip(frames,frames[1:]):
        if current.get('boundary_model') not in TRACKED_MODELS:
            continue
        diagnostics = current.get('boundary_detection_diagnostics') or [{},{}]
        if (current.get('source_epoch') != previous.get('source_epoch')
                or diagnostics[0].get('reset') or diagnostics[1].get('reset')):
            continue
        dt = current['nn_timestamp_s']-previous['nn_timestamp_s']
        if not 0 < dt <= 1.:
            continue
        state = current['left_boundary_state']
        if state not in ('held','inferred'):
            continue
        old = {round(y,6):x for x,y in previous.get('left_boundary_image',[])}
        new = {round(y,6):x for x,y in current.get('left_boundary_image',[])}
        rows = old.keys() & new.keys()
        if len(rows) < 8:
            continue
        movement = max(abs(new[y]-old[y]) for y in rows)
        # Bound applies to an uninterrupted missed-side interval. If the
        # telemetry skipped source frames, intervening observations are unknown.
        source_gap = (current.get('source_frame_index',0) or 0)-(previous.get('source_frame_index',0) or 0)
        record = dict(frame_seq=current['frame_seq'], media_time_s=current.get('source_media_time_s'),
                      dt_s=dt, max_movement_px=movement, rate_px_s=movement/dt,
                      previous_state=previous['left_boundary_state'], state=state,
                      source_gap=source_gap, uninterrupted_source=source_gap <= 1)
        movements.append(record)
        if previous['left_boundary_state'] in ('observed','aligning','partial'):
            transitions.append(record)
        if movement > 20.*dt+1e-5 and source_gap <= 1:
            violations.append(record)
    return dict(frames=len(frames), left_states=dict(Counter(f.get('left_boundary_state') for f in frames)),
                max_retained_missing_s=max(retained_ages,default=None),
                long_retained_frames=sum(age>1.5 for age in retained_ages),
                opposite_curve_updates=curve_updates,
                models=dict(Counter(f.get('boundary_model') for f in frames)),
                loss_transitions=transitions, missed_side_samples=len(movements),
                uninterrupted_missed_samples=sum(r['uninterrupted_source'] for r in movements),
                max_uninterrupted_rate_px_s=max((r['rate_px_s'] for r in movements
                    if r['uninterrupted_source']),default=None),
                max_missed_movement_px=max((r['max_movement_px'] for r in movements),default=None),
                max_missed_rate_px_s=max((r['rate_px_s'] for r in movements),default=None),
                max_transition_px=max((r['max_movement_px'] for r in transitions),default=None),
                limit_violations=violations, movements=movements)


def run(args):
    args.output.mkdir(parents=True,exist_ok=True)
    frames, errors, images, writers = [], [], [], {}
    done = threading.Event()

    def receive():
        try:
            with socket.create_connection((args.device_ip,8557),timeout=6) as connection:
                connection.settimeout(4)
                connection.sendall(b'HELLO\n')
                with connection.makefile('rb') as stream:
                    while not done.is_set():
                        line = stream.readline()
                        if not line:
                            if not done.is_set():
                                errors.append('Telemetry connection closed unexpectedly')
                            break
                        payload = json.loads(line)
                        if 'frame_seq' in payload:
                            frames.append(payload)
        except (OSError,ValueError) as exc:
            if not done.is_set():
                errors.append(str(exc))

    set_processing_mode(args.device_ip,'full')
    set_boundary_display(args.device_ip,True)
    set_contour_display(args.device_ip,False)
    set_lane_overlay(args.device_ip,'normal',.5)
    select_camera(args.device_ip)
    reader = threading.Thread(target=receive,daemon=True)
    reader.start()
    sources = {key:CaptureSource(key,key,f'rtsp://{args.device_ip}:{port}/{name}')
               for key,port,name in [('result',8554,'yolopv2'),('top',8553,'bev')]}
    for source in sources.values():
        source.start()
    sender = None
    began = time.monotonic()
    try:
        if args.video:
            sender = VideoFileSender(args.device_ip,args.video,loop=False,fps=args.fps,
                                     start_s=args.start,end_s=args.end)
            sender.start()
        next_image = began+2.
        next_video = began
        while time.monotonic()-began < args.seconds:
            if sender and sender.snapshot().get('error'):
                raise RuntimeError(sender.snapshot()['error'])
            if sender and sender.thread and not sender.thread.is_alive():
                break
            if time.monotonic() >= next_video:
                for key, source in sources.items():
                    connected, image = source.snapshot()
                    if not connected or image is None:
                        continue
                    if key not in writers:
                        writers[key] = cv2.VideoWriter(str(args.output/f'{key}.mp4'),
                            cv2.VideoWriter_fourcc(*'mp4v'),10.,(image.shape[1],image.shape[0]))
                        if not writers[key].isOpened():
                            raise OSError('Could not open diagnostic video writer')
                    writers[key].write(image)
                next_video = time.monotonic()+.1
            if time.monotonic() >= next_image:
                row = frames[-1] if frames else {}
                paths = {}
                for key, source in sources.items():
                    connected, image = source.snapshot()
                    if connected and image is not None:
                        name = f'{key}_{len(images):04d}.jpg'
                        cv2.imwrite(str(args.output/name),image)
                        paths[key] = name
                if paths:
                    images.append(dict(paths=paths,telemetry_frame=row.get('frame_seq'),
                        media_time_s=row.get('source_media_time_s'),left_state=row.get('left_boundary_state')))
                next_image = time.monotonic()+1.
            time.sleep(.05)
        time.sleep(.5)
    finally:
        if sender:
            sender.close()
        done.set()
        reader.join(timeout=5)
        video_report = {}
        for writer in writers.values():
            writer.release()
        for key,source in sources.items():
            video_report[key] = dict(frames=source.frames_received,reconnects=source.reconnects,
                                     received=source.snapshot()[0],
                                     last_frame_age_s=source.info()['last_frame_age_s'])
            source.close()
        # Leave the installed app running on its real camera, in full mode.
        try:
            select_camera(args.device_ip)
        except (OSError,RuntimeError,ValueError) as exc:
            errors.append('Camera recovery: '+str(exc))
    relevant = [f for f in frames if f.get('input_source') == ('pc' if args.video else 'camera')]
    if args.video:
        relevant = [f for f in relevant if f.get('source_file') == args.video.name]
    # The asynchronous NN queue may still publish frames captured BEFORE the
    # full-mode command. Discard that prefix only; a later fallback/restart must
    # remain visible and fail the runtime identity check.
    startup_frames = 0
    while relevant and relevant[0].get('boundary_model') not in TRACKED_MODELS:
        relevant.pop(0)
        startup_frames += 1
    report = analyze(relevant)
    report.update(source_video=str(args.video) if args.video else None, injected_masks=False,
                  startup_frames_excluded=startup_frames,
                  sequence_mismatches=sum(f.get('frame_seq') != f.get('nn_seq') for f in relevant),
                  telemetry_sessions=sorted(set(f.get('session','unknown') for f in relevant)),
                  errors=errors,streams=video_report,images=images,sender=sender.snapshot() if sender else None,
                  note='RTSP images and telemetry are asynchronous; coordinate measurements use telemetry only.')
    report['runtime_ok'] = (len(relevant) >= 20 and not errors
                            and set(report['models']).issubset(TRACKED_MODELS)
                            and not report['sequence_mismatches'] and len(report['telemetry_sessions']) == 1
                            and all(s['frames'] >= 5 and s['received']
                                    and s['last_frame_age_s'] <= 3. for s in video_report.values()))
    report['loss_continuity_observed'] = (any(r['uninterrupted_source'] for r in report['loss_transitions'])
                                          and not report['limit_violations'])
    (args.output/'frames.jsonl').write_text('\n'.join(json.dumps(f) for f in relevant),encoding='utf-8')
    (args.output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('images','movements','loss_transitions')},indent=2),flush=True)
    if not report['runtime_ok'] or report['limit_violations']:
        raise RuntimeError('Deployed runtime or measured continuity contract failed; inspect report')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-ip',default='192.168.201.2')
    parser.add_argument('--video',type=Path)
    parser.add_argument('--start',type=float,default=0.)
    parser.add_argument('--end',type=float)
    parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--fps',type=float,default=10.)
    parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args())
