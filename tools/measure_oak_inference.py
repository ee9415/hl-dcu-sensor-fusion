"""Count actual post-NN frame completions against 10FPS video submissions."""
import argparse
import json
import math
from pathlib import Path
import socket
import threading
import time

from video_file_sender import VideoFileSender, select_camera


def arrival_statistics(rows):
    times = [row['received_monotonic_s'] for row in rows]
    span = times[-1]-times[0] if len(times)>1 else 0
    gaps = sorted((b-a)*1000 for a,b in zip(times,times[1:]))
    return dict(completion_notification_receive_span_s=span,
        completion_fps_over_host_receive_span=(len(rows)-1)/span if span else None,
        completion_notification_gap_ms=dict(mean=sum(gaps)/len(gaps),
            p95=gaps[max(0,math.ceil(len(gaps)*.95)-1)], max=max(gaps)) if gaps else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--device-ip', default='169.254.100.228')
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--summarize-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.summarize_only:
        rows = json.loads((args.output/'frames.json').read_text())
        report = json.loads((args.output/'report.json').read_text())
        if 'completion_fps_over_capture_span' in report:
            report['capture_timestamp_span_fps'] = report.pop('completion_fps_over_capture_span')
        report.update(arrival_statistics(rows))
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(arrival_statistics(rows),indent=2))
        return 0
    rows = []
    errors = []
    stop = threading.Event()
    ready = threading.Event()
    connection = socket.create_connection((args.device_ip,8557),timeout=3)
    connection.settimeout(3)
    def receive():
        try:
            connection.sendall(b'HELLO\n')
            with connection.makefile('rb') as stream:
                json.loads(stream.readline())
                ready.set()
                while not stop.is_set():
                    line = stream.readline()
                    if not line:
                        break
                    item = json.loads(line)
                    if (item.get('input_source') == 'pc' and
                            item.get('source_file') == args.video.name and 'nn_seq' in item):
                        rows.append(dict(source_frame_index=item['source_frame_index'],
                            media_time_s=item['source_media_time_s'], frame_seq=item['frame_seq'],
                            nn_seq=item['nn_seq'], capture_timestamp_s=item['capture_timestamp_s'],
                            received_monotonic_s=time.monotonic()))
        except (OSError,ValueError) as exc:
            if not stop.is_set():
                errors.append(str(exc))
            ready.set()
    worker = threading.Thread(target=receive,daemon=True)
    worker.start()
    sender = VideoFileSender(args.device_ip,args.video,loop=False,fps=10,end_s=args.seconds)
    report = {}
    try:
        if not ready.wait(5) or errors:
            raise RuntimeError('Telemetry unavailable: '+repr(errors))
        sender.start()
        deadline = time.monotonic()+args.seconds+20
        while time.monotonic() < deadline:
            status = sender.snapshot()
            if status['state']=='오류':
                raise RuntimeError(status['error'])
            if status['state']=='재생 완료':
                break
            time.sleep(.05)
        else:
            raise RuntimeError('Video replay timeout')
        time.sleep(2)
        unique = {row['source_frame_index']:row for row in rows}
        indices = sorted(unique)
        expected = list(range(round(args.seconds*10)))
        missing = sorted(set(expected)-set(indices))
        timestamps = [unique[index]['capture_timestamp_s'] for index in indices]
        observed_span = timestamps[-1]-timestamps[0] if len(timestamps)>1 else 0
        report = dict(input_fps=10, duration_s=args.seconds, sender=sender.snapshot(),
            telemetry_frames=len(rows), unique_nn_completions=len(unique),
            missing_source_indices=missing,
            nn_rgb_sequence_mismatches=sum(row['nn_seq']!=row['frame_seq'] for row in rows),
            first_media_s=unique[indices[0]]['media_time_s'] if indices else None,
            last_media_s=unique[indices[-1]]['media_time_s'] if indices else None,
            capture_timestamp_span_fps=(len(unique)-1)/observed_span if observed_span else None,
            completions_per_input_duration=len(unique)/args.seconds, errors=errors,
            passed=not missing and len(unique)==len(expected) and not errors,
            **arrival_statistics(rows))
    finally:
        sender.close()
        report['camera_restore'] = select_camera(args.device_ip)
        stop.set()
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        worker.join(timeout=4)
        connection.close()
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        (args.output/'frames.json').write_text(json.dumps(rows,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report.get('passed') else 1


if __name__=='__main__':
    raise SystemExit(main())
